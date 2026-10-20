/*
 * feature_stream.c - template: FAST corner (feature point) detection on the
 * board, streamed live to a browser as an MJPEG feature map.
 *
 * Build on the board:   zig cc -O2 feature_stream.c -o feature_stream -lm
 * Run:                  ./feature_stream [camera|auto] [port]      (defaults: auto, 8080)
 * View on the laptop:   http://<board-ip>:8080/
 *
 * Stream options (query string on /stream, also links on the page):
 *   /stream            feature points over a dimmed camera image
 *   /stream?map        feature points only, on black
 *   /stream?t=30       FAST threshold (default 20; higher = fewer, stronger corners)
 *
 * Pipeline per frame:
 *   camera YUYV 640x480 -> take Y (grayscale, no decoding needed)
 *   -> FAST-9 corner test (16-pixel circle, 9 contiguous brighter/darker)
 *   -> 3x3 non-maximum suppression on the corner score
 *   -> draw points -> JPEG (stb_image_write) -> multipart HTTP to the browser
 *
 * Put your own use of the keypoints in on_features().
 */
#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <linux/videodev2.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STBI_WRITE_NO_STDIO
#include "third_party/stb_image_write.h"

#define WIDTH           640
#define HEIGHT          480
#define NBUFFERS        4
#define BOUNDARY        "featureframe"
#define FRAME_TIMEOUT_S 2
#define JPEG_QUALITY    70
#define MAX_FEATURES    4000

struct buffer {
    void *start;
    size_t length;
};

struct keypoint {
    int16_t x, y;
    uint16_t score;
};

/* ---------- your code goes here ---------- */

/* Called once per frame with the detected keypoints (sorted top-left to bottom-right). */
static void on_features(const struct keypoint *kp, int n, const uint8_t *gray)
{
    (void)kp;
    (void)n;
    (void)gray;
}

/* ---------- FAST-9 corner detector ---------- */

/* Bresenham circle of radius 3: 16 offsets, clockwise from 12 o'clock. */
static const int CIRCLE_DX[16] = {0, 1, 2, 3, 3, 3, 2, 1, 0, -1, -2, -3, -3, -3, -2, -1};
static const int CIRCLE_DY[16] = {-3, -3, -2, -1, 0, 1, 2, 3, 3, 3, 2, 1, 0, -1, -2, -3};

/* True if `mask` (16 bits, circular) has at least 9 consecutive set bits. */
static int has_arc9(unsigned mask)
{
    unsigned m = mask | (mask << 16); /* unroll the circle so arcs can wrap */
    m &= m >> 1; /* after these steps, a bit survives only if 9 in a row were set */
    m &= m >> 2;
    m &= m >> 4;
    m &= m >> 1;
    return m != 0;
}

static uint16_t *score_map;

/* Detect corners in an 8-bit grayscale image. Returns the number of keypoints written to `out`. */
static int fast9_detect(const uint8_t *img, int w, int h, int t, struct keypoint *out, int max_out)
{
    int off[16];
    for (int i = 0; i < 16; i++)
        off[i] = CIRCLE_DY[i] * w + CIRCLE_DX[i];

    memset(score_map, 0, (size_t)w * h * sizeof *score_map);

    for (int y = 3; y < h - 3; y++) {
        const uint8_t *row = img + y * w;
        for (int x = 3; x < w - 3; x++) {
            const uint8_t *p = row + x;
            int c = *p, hi = c + t, lo = c - t;

            /* Any 9-pixel arc covers at least 2 of the 4 compass points (0, 4, 8, 12): cheap early reject. */
            int b = (p[off[0]] > hi) + (p[off[4]] > hi) + (p[off[8]] > hi) + (p[off[12]] > hi);
            int d = (p[off[0]] < lo) + (p[off[4]] < lo) + (p[off[8]] < lo) + (p[off[12]] < lo);
            if (b < 2 && d < 2)
                continue;

            unsigned bright = 0, dark = 0;
            int sb = 0, sd = 0;
            for (int i = 0; i < 16; i++) {
                int v = p[off[i]];
                if (v > hi) {
                    bright |= 1u << i;
                    sb += v - hi;
                } else if (v < lo) {
                    dark |= 1u << i;
                    sd += lo - v;
                }
            }
            if (has_arc9(bright) || has_arc9(dark)) {
                int s = (sb > sd ? sb : sd) + 1;
                score_map[y * w + x] = s > 65535 ? 65535 : (uint16_t)s;
            }
        }
    }

    /* Keep only corners that beat all 8 neighbours (ties go to the earlier pixel). */
    int n = 0;
    for (int y = 3; y < h - 3 && n < max_out; y++) {
        for (int x = 3; x < w - 3 && n < max_out; x++) {
            const uint16_t *s = score_map + y * w + x;
            uint16_t v = *s;
            if (!v)
                continue;
            if (v <= s[-w - 1] || v <= s[-w] || v <= s[-w + 1] || v <= s[-1] ||
                v < s[1] || v < s[w - 1] || v < s[w] || v < s[w + 1])
                continue;
            out[n++] = (struct keypoint){(int16_t)x, (int16_t)y, v};
        }
    }
    return n;
}

/* ---------- drawing + JPEG ---------- */

static void put_px(uint8_t *rgb, int w, int h, int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
    if (x < 0 || y < 0 || x >= w || y >= h)
        return;
    uint8_t *p = rgb + 3 * (y * w + x);
    p[0] = r;
    p[1] = g;
    p[2] = b;
}

/* Background (dimmed camera image or black) plus one marker per keypoint, colored by strength. */
static void render(uint8_t *rgb, const uint8_t *gray, int w, int h, const struct keypoint *kp, int n, int map_only)
{
    if (map_only) {
        memset(rgb, 0, (size_t)w * h * 3);
    } else {
        for (int i = 0; i < w * h; i++)
            rgb[3 * i] = rgb[3 * i + 1] = rgb[3 * i + 2] = gray[i] >> 1;
    }

    for (int i = 0; i < n; i++) {
        /* weak = yellow, strong = green */
        int s = kp[i].score > 400 ? 400 : kp[i].score;
        uint8_t r = (uint8_t)(255 - s * 255 / 400), g = 255, b = 0;
        int x = kp[i].x, y = kp[i].y;
        for (int d = -2; d <= 2; d++) { /* small plus sign */
            put_px(rgb, w, h, x + d, y, r, g, b);
            put_px(rgb, w, h, x, y + d, r, g, b);
        }
    }
}

struct membuf {
    uint8_t *data;
    size_t len, cap;
};

static void membuf_write(void *ctx, void *data, int size)
{
    struct membuf *m = ctx;
    if (m->len + size > m->cap) {
        m->cap = (m->len + size) * 2;
        m->data = realloc(m->data, m->cap);
    }
    memcpy(m->data + m->len, data, size);
    m->len += size;
}

/* ---------- camera ---------- */

static int xioctl(int fd, unsigned long req, void *arg)
{
    int r;
    do {
        r = ioctl(fd, req, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

static int camera_start(const char *dev, struct buffer *bufs, unsigned *nbufs)
{
    char path[256];
    if (strcmp(dev, "auto") == 0) {
        glob_t g;
        if (glob("/dev/v4l/by-id/*-video-index0", 0, NULL, &g) != 0)
            return -1;
        snprintf(path, sizeof path, "%s", g.gl_pathv[0]);
        globfree(&g);
        dev = path;
    }

    int fd = open(dev, O_RDWR);
    if (fd < 0) {
        if (errno != ENOENT)
            perror(dev);
        return -1;
    }

    /* Raw YUYV: the Y bytes are already a grayscale image, so there's nothing to decode. */
    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = WIDTH;
    fmt.fmt.pix.height = HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0 || fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV ||
        fmt.fmt.pix.width != WIDTH || fmt.fmt.pix.height != HEIGHT) {
        fprintf(stderr, "camera does not support YUYV %dx%d\n", WIDTH, HEIGHT);
        close(fd);
        return -1;
    }

    struct v4l2_requestbuffers req = {0};
    req.count = NBUFFERS;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("VIDIOC_REQBUFS");
        close(fd);
        return -1;
    }
    *nbufs = req.count;
    for (unsigned i = 0; i < req.count; i++) {
        struct v4l2_buffer b = {0};
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index = i;
        xioctl(fd, VIDIOC_QUERYBUF, &b);
        bufs[i].length = b.length;
        bufs[i].start = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
        xioctl(fd, VIDIOC_QBUF, &b);
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        perror("VIDIOC_STREAMON");
        close(fd);
        return -1;
    }
    printf("camera %s: %dx%d YUYV\n", dev, WIDTH, HEIGHT);
    fflush(stdout);
    return fd;
}

static void camera_stop(int fd, struct buffer *bufs, unsigned nbufs)
{
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(fd, VIDIOC_STREAMOFF, &type);
    for (unsigned i = 0; i < nbufs; i++)
        munmap(bufs[i].start, bufs[i].length);
    close(fd);
}

/* Wait for a frame, then skip ahead to the newest one queued so the stream never lags behind. */
static int latest_frame(int cam, struct v4l2_buffer *b)
{
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(cam, &fds);
    struct timeval tv = {FRAME_TIMEOUT_S, 0};
    if (select(cam + 1, &fds, NULL, NULL, &tv) <= 0)
        return -1;
    memset(b, 0, sizeof *b);
    b->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    b->memory = V4L2_MEMORY_MMAP;
    if (xioctl(cam, VIDIOC_DQBUF, b) < 0)
        return -1;

    for (;;) {
        struct timeval zero = {0, 0};
        FD_ZERO(&fds);
        FD_SET(cam, &fds);
        if (select(cam + 1, &fds, NULL, NULL, &zero) <= 0)
            return 0;
        struct v4l2_buffer newer = {0};
        newer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        newer.memory = V4L2_MEMORY_MMAP;
        if (xioctl(cam, VIDIOC_DQBUF, &newer) < 0)
            return 0;
        xioctl(cam, VIDIOC_QBUF, b); /* give the stale frame back */
        *b = newer;
    }
}

/* ---------- HTTP ---------- */

static int write_all(int fd, const void *buf, size_t len)
{
    const char *p = buf;
    while (len > 0) {
        ssize_t n = write(fd, p, len);
        if (n <= 0)
            return -1;
        p += n;
        len -= n;
    }
    return 0;
}

static double now_sec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

static int camera_wait(int client, const char *dev, struct buffer *bufs, unsigned *nbufs)
{
    for (int tries = 0;; tries++) {
        int cam = camera_start(dev, bufs, nbufs);
        if (cam >= 0)
            return cam;
        if (tries == 0) {
            printf("waiting for camera...\n");
            fflush(stdout);
        }
        if (send(client, "", 0, MSG_NOSIGNAL) < 0)
            return -1;
        sleep(1);
    }
}

static void serve_stream(int client, const char *dev, int threshold, int map_only)
{
    const char *hdr = "HTTP/1.0 200 OK\r\nCache-Control: no-cache\r\n"
                      "Content-Type: multipart/x-mixed-replace; boundary=" BOUNDARY "\r\n\r\n";
    if (write_all(client, hdr, strlen(hdr)) < 0)
        return;

    struct buffer bufs[NBUFFERS];
    unsigned nbufs = 0;
    int cam = camera_wait(client, dev, bufs, &nbufs);
    if (cam < 0)
        return;

    static uint8_t gray[WIDTH * HEIGHT], rgb[WIDTH * HEIGHT * 3];
    static struct keypoint kp[MAX_FEATURES];
    struct membuf jpg = {0};
    unsigned frames = 0;
    double t0 = now_sec(), t_detect = 0, t_encode = 0;
    long total_kp = 0;

    printf("streaming features (threshold %d, %s)\n", threshold, map_only ? "map only" : "overlay");
    fflush(stdout);

    for (;;) {
        struct v4l2_buffer b;
        if (latest_frame(cam, &b) < 0) {
            printf("camera lost, reopening\n");
            fflush(stdout);
            camera_stop(cam, bufs, nbufs);
            cam = camera_wait(client, dev, bufs, &nbufs);
            if (cam < 0)
                break;
            continue;
        }

        /* YUYV = Y0 U Y1 V ...: every other byte is luma. */
        const uint8_t *yuyv = bufs[b.index].start;
        for (int i = 0; i < WIDTH * HEIGHT; i++)
            gray[i] = yuyv[2 * i];
        xioctl(cam, VIDIOC_QBUF, &b);

        double ta = now_sec();
        int n = fast9_detect(gray, WIDTH, HEIGHT, threshold, kp, MAX_FEATURES);
        double tb = now_sec();
        on_features(kp, n, gray);
        render(rgb, gray, WIDTH, HEIGHT, kp, n, map_only);
        jpg.len = 0;
        stbi_write_jpg_to_func(membuf_write, &jpg, WIDTH, HEIGHT, 3, rgb, JPEG_QUALITY);
        double tc = now_sec();
        t_detect += tb - ta;
        t_encode += tc - tb;
        total_kp += n;

        char part[160];
        int len = snprintf(part, sizeof part,
                           "--" BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\nX-Features: %d\r\n\r\n",
                           jpg.len, n);
        if (write_all(client, part, len) < 0 || write_all(client, jpg.data, jpg.len) < 0 ||
            write_all(client, "\r\n", 2) < 0)
            break;

        if (++frames % 30 == 0) {
            printf("%u frames, %.1f fps | avg %ld features | detect %.0f ms, draw+jpeg %.0f ms\n",
                   frames, frames / (now_sec() - t0), total_kp / frames,
                   1000 * t_detect / frames, 1000 * t_encode / frames);
            fflush(stdout);
        }
    }
    free(jpg.data);
    if (cam >= 0)
        camera_stop(cam, bufs, nbufs);
}

static const char PAGE[] =
    "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n"
    "<!doctype html><meta name=viewport content=\"width=device-width\"><title>mp157 features</title>"
    "<style>body{margin:0;background:#111;color:#ccc;font:14px system-ui;display:grid;place-items:center;min-height:100vh}"
    "img{max-width:100%;max-height:90vh}a{color:#8f8;margin:0 .5em}</style>"
    "<body><div><img id=v src=\"/stream\"><p style=text-align:center>"
    "<a href=# onclick=\"v.src='/stream?'+Date.now()\">overlay</a>"
    "<a href=# onclick=\"v.src='/stream?map&'+Date.now()\">map only</a>"
    "threshold: <a href=# onclick=\"v.src='/stream?t=10&'+Date.now()\">10</a>"
    "<a href=# onclick=\"v.src='/stream?t=20&'+Date.now()\">20</a>"
    "<a href=# onclick=\"v.src='/stream?t=40&'+Date.now()\">40</a></p></div></body>";

int main(int argc, char **argv)
{
    const char *dev = argc > 1 ? argv[1] : "auto";
    int port = argc > 2 ? atoi(argv[2]) : 8080;

    signal(SIGPIPE, SIG_IGN);
    score_map = malloc((size_t)WIDTH * HEIGHT * sizeof *score_map);

    int srv = socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(srv, (struct sockaddr *)&addr, sizeof addr) < 0 || listen(srv, 4) < 0) {
        perror("bind/listen");
        return 1;
    }
    printf("feature stream on http://0.0.0.0:%d/\n", port);
    fflush(stdout);

    for (;;) {
        int client = accept(srv, NULL, NULL);
        if (client < 0)
            continue;

        char req[1024] = {0};
        ssize_t n = read(client, req, sizeof req - 1);
        if (n > 0 && strncmp(req, "GET /stream", 11) == 0) {
            char *eol = strstr(req, " HTTP/");
            if (eol)
                *eol = '\0';
            int t = 20;
            char *tp = strstr(req, "t=");
            if (tp)
                t = atoi(tp + 2);
            if (t < 1)
                t = 1;
            printf("viewer connected\n");
            fflush(stdout);
            serve_stream(client, dev, t, strstr(req, "map") != NULL);
            printf("viewer disconnected\n");
            fflush(stdout);
        } else if (n > 0 && strncmp(req, "GET / ", 6) == 0) {
            write_all(client, PAGE, sizeof PAGE - 1);
        } else {
            const char *nf = "HTTP/1.0 404 Not Found\r\n\r\n";
            write_all(client, nf, strlen(nf));
        }
        close(client);
    }
}
