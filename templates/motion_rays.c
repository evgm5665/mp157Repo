/*
 * motion_rays.c - template: detect moving objects, box them, and turn each one
 * into a 3D viewing ray from the camera. Streams the result live to a browser.
 *
 * Build on the board:   zig cc -O2 motion_rays.c -o motion_rays -lm
 * Run:                  ./motion_rays [camera|auto] [port] [udp-host udp-port]
 *   e.g.                ./motion_rays auto 8080 10.176.87.219 5000
 * View on the laptop:   http://<board-ip>:8080/
 *
 * Pipeline per frame:
 *   camera YUYV 640x480 -> grayscale (Y bytes) -> 4x4 downsample to 160x120
 *   -> running-average background model -> |frame - background| > threshold
 *   -> clean up the mask (3x3 open) -> connected components = moving blobs
 *   -> merge blobs within MERGE_GAP px (one body often moves in several pieces)
 *   -> bounding boxes (blobs smaller than min area are dropped as noise)
 *   -> nearest-centroid tracking so each object keeps an ID
 *   -> ray through each box centre via the pinhole camera model
 *
 * Rays: unit vector (dx, dy, dz) in camera coordinates. +x right, +y down,
 * +z out of the lens. One camera only gives a direction; intersect rays from
 * two or more calibrated cameras to get 3D points (what Braid does).
 *
 * CALIBRATION: the intrinsics below assume a 60 degree horizontal field of view,
 * principal point at the image centre, and no lens distortion. Replace
 * FX/FY/CX/CY with values from a real calibration (e.g. OpenCV
 * calibrateCamera with a checkerboard) for accurate rays.
 *
 * UDP (if udp-host/port given), one line per object per frame:
 *   frame,id,u,v,dx,dy,dz,box_x,box_y,box_w,box_h
 *
 * Stream options: /stream?thr=25&min=12  (difference threshold, min blob area in 4x4 cells)
 *
 * Put your own use of the rays in on_rays().
 */
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <linux/videodev2.h>
#include <math.h>
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
#define SCALE           4 /* motion runs on a 160x120 grid of 4x4 blocks */
#define GW              (WIDTH / SCALE)
#define GH              (HEIGHT / SCALE)
#define NBUFFERS        4
#define BOUNDARY        "motionframe"
#define FRAME_TIMEOUT_S 2
#define JPEG_QUALITY    70
#define MAX_OBJECTS     32

/* Camera intrinsics (pixels). Assumed: 60 deg HFOV, centred, no distortion. Replace after calibrating. */
#define HFOV_DEG 60.0
#define FX       ((WIDTH / 2.0) / tan(HFOV_DEG * M_PI / 360.0))
#define FY       FX
#define CX       (WIDTH / 2.0)
#define CY       (HEIGHT / 2.0)

#define DEFAULT_THRESHOLD 25 /* gray levels of change that count as motion */
#define DEFAULT_MIN_AREA  12 /* blob area in grid cells (1 cell = 4x4 pixels) */
#define BG_RATE_SHIFT     4  /* background adapts 1/16 per frame (~1.3 s at 12 fps) */
#define BG_RATE_FG_SHIFT  7  /* ...and 1/128 where something is moving */
#define MATCH_DIST        60 /* px: max centroid jump to keep the same object ID */
#define MERGE_GAP         24 /* px: blobs closer than this are treated as one object */
#define MAX_BLOBS         256

struct object {
    int id;
    int x, y, w, h;   /* bounding box, full-res pixels */
    double u, v;      /* centroid, full-res pixels */
    double ray[3];    /* unit direction, camera coordinates */
    int area;         /* grid cells */
};

/* ---------- your code goes here ---------- */

/* Called once per frame with every moving object and its ray. */
static void on_rays(unsigned frame, const struct object *obj, int n)
{
    (void)frame;
    (void)obj;
    (void)n;
}

/* ---------- motion detection ---------- */

static uint16_t bg[GW * GH];    /* background, 8.8 fixed point */
static uint8_t small[GW * GH];  /* current frame, downsampled */
static uint8_t mask[GW * GH];   /* 1 = moving */
static uint8_t tmp[GW * GH];
static int bg_ready;

static void downsample(const uint8_t *gray)
{
    for (int gy = 0; gy < GH; gy++)
        for (int gx = 0; gx < GW; gx++) {
            int sum = 0;
            for (int dy = 0; dy < SCALE; dy++) {
                const uint8_t *p = gray + (gy * SCALE + dy) * WIDTH + gx * SCALE;
                for (int dx = 0; dx < SCALE; dx++)
                    sum += p[dx];
            }
            small[gy * GW + gx] = (uint8_t)(sum / (SCALE * SCALE));
        }
}

/* 3x3 erode (op=0) or dilate (op=1) of `src` into `dst`. */
static void morph(const uint8_t *src, uint8_t *dst, int dilate)
{
    for (int y = 0; y < GH; y++)
        for (int x = 0; x < GW; x++) {
            int any = 0, all = 1;
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++) {
                    int xx = x + dx, yy = y + dy;
                    int v = (xx >= 0 && yy >= 0 && xx < GW && yy < GH) ? src[yy * GW + xx] : 0;
                    any |= v;
                    all &= v;
                }
            dst[y * GW + x] = (uint8_t)(dilate ? any : all);
        }
}

/* Build the motion mask; returns the fraction of the grid that changed. */
static double update_motion(int threshold)
{
    if (!bg_ready) {
        for (int i = 0; i < GW * GH; i++)
            bg[i] = (uint16_t)(small[i] << 8);
        bg_ready = 1;
        memset(mask, 0, sizeof mask);
        return 0;
    }

    int changed = 0;
    for (int i = 0; i < GW * GH; i++) {
        int b = bg[i] >> 8;
        int fg = abs((int)small[i] - b) > threshold;
        mask[i] = (uint8_t)fg;
        changed += fg;
        int target = small[i] << 8;
        int shift = fg ? BG_RATE_FG_SHIFT : BG_RATE_SHIFT;
        bg[i] = (uint16_t)(bg[i] + ((target - (int)bg[i]) >> shift));
    }

    /* Opening (erode then dilate) removes speckle noise; a second dilate joins nearby parts of one object. */
    morph(mask, tmp, 0);
    morph(tmp, mask, 1);
    morph(mask, tmp, 1);
    memcpy(mask, tmp, sizeof mask);
    return (double)changed / (GW * GH);
}

/* Connected components on the mask -> bounding boxes. */
static int find_blobs(struct object *out, int max_out, int min_area)
{
    static uint8_t seen[GW * GH];
    static int stack[GW * GH];
    memset(seen, 0, sizeof seen);
    int n = 0;

    for (int start = 0; start < GW * GH; start++) {
        if (!mask[start] || seen[start])
            continue;
        int sp = 0, area = 0;
        int x0 = GW, y0 = GH, x1 = -1, y1 = -1;
        long sx = 0, sy = 0;
        stack[sp++] = start;
        seen[start] = 1;
        while (sp) {
            int i = stack[--sp], x = i % GW, y = i / GW;
            area++;
            sx += x;
            sy += y;
            if (x < x0) x0 = x;
            if (x > x1) x1 = x;
            if (y < y0) y0 = y;
            if (y > y1) y1 = y;
            const int nb[4] = {i - 1, i + 1, i - GW, i + GW};
            const int ok[4] = {x > 0, x < GW - 1, y > 0, y < GH - 1};
            for (int k = 0; k < 4; k++)
                if (ok[k] && mask[nb[k]] && !seen[nb[k]]) {
                    seen[nb[k]] = 1;
                    stack[sp++] = nb[k];
                }
        }
        if (n >= max_out)
            continue;
        struct object *o = &out[n++];
        o->x = x0 * SCALE;
        o->y = y0 * SCALE;
        o->w = (x1 - x0 + 1) * SCALE;
        o->h = (y1 - y0 + 1) * SCALE;
        o->u = (sx / (double)area + 0.5) * SCALE; /* mask centroid, not box centre: follows the object's mass */
        o->v = (sy / (double)area + 0.5) * SCALE;
        o->area = area;
        o->id = -1;
    }

    /* A moving body often shows up as several pieces (only some parts change between frames).
       Merge blobs whose boxes are within MERGE_GAP px of each other, until nothing changes. */
    for (int merged = 1; merged;) {
        merged = 0;
        for (int i = 0; i < n; i++)
            for (int j = i + 1; j < n; j++) {
                struct object *a = &out[i], *b = &out[j];
                int gap_x = (a->x > b->x + b->w ? a->x - (b->x + b->w) : b->x > a->x + a->w ? b->x - (a->x + a->w) : 0);
                int gap_y = (a->y > b->y + b->h ? a->y - (b->y + b->h) : b->y > a->y + a->h ? b->y - (a->y + a->h) : 0);
                if (gap_x > MERGE_GAP || gap_y > MERGE_GAP)
                    continue;
                int x0 = a->x < b->x ? a->x : b->x, y0 = a->y < b->y ? a->y : b->y;
                int x1 = a->x + a->w > b->x + b->w ? a->x + a->w : b->x + b->w;
                int y1 = a->y + a->h > b->y + b->h ? a->y + a->h : b->y + b->h;
                int area = a->area + b->area;
                a->u = (a->u * a->area + b->u * b->area) / area;
                a->v = (a->v * a->area + b->v * b->area) / area;
                a->x = x0;
                a->y = y0;
                a->w = x1 - x0;
                a->h = y1 - y0;
                a->area = area;
                out[j--] = out[--n];
                merged = 1;
            }
    }

    /* Drop what's still too small to be an object (noise, flicker). */
    int kept = 0;
    for (int i = 0; i < n; i++)
        if (out[i].area >= min_area)
            out[kept++] = out[i];
    return kept;
}

/* Pinhole model: pixel (u, v) -> unit ray in camera coordinates. */
static void pixel_to_ray(double u, double v, double ray[3])
{
    double x = (u - CX) / FX, y = (v - CY) / FY, z = 1.0;
    double len = sqrt(x * x + y * y + z * z);
    ray[0] = x / len;
    ray[1] = y / len;
    ray[2] = z / len;
}

/* Greedy nearest-centroid matching against last frame's objects so IDs persist. */
static void assign_ids(struct object *cur, int n)
{
    static struct object prev[MAX_OBJECTS];
    static int nprev, next_id = 1;
    int used[MAX_OBJECTS] = {0};

    for (int i = 0; i < n; i++) {
        int best = -1;
        double best_d = MATCH_DIST;
        for (int j = 0; j < nprev; j++) {
            if (used[j])
                continue;
            double d = hypot(cur[i].u - prev[j].u, cur[i].v - prev[j].v);
            if (d < best_d) {
                best_d = d;
                best = j;
            }
        }
        if (best >= 0) {
            used[best] = 1;
            cur[i].id = prev[best].id;
        } else {
            cur[i].id = next_id++;
        }
    }
    memcpy(prev, cur, n * sizeof *cur);
    nprev = n;
}

/* ---------- drawing ---------- */

static uint8_t rgb[WIDTH * HEIGHT * 3];

static void put_px(int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
    if (x < 0 || y < 0 || x >= WIDTH || y >= HEIGHT)
        return;
    uint8_t *p = rgb + 3 * (y * WIDTH + x);
    p[0] = r;
    p[1] = g;
    p[2] = b;
}

static void draw_line(int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b)
{
    int dx = abs(x1 - x0), dy = -abs(y1 - y0), sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1, err = dx + dy;
    for (;;) {
        put_px(x0, y0, r, g, b);
        if (x0 == x1 && y0 == y1)
            break;
        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

static void draw_rect(int x, int y, int w, int h, uint8_t r, uint8_t g, uint8_t b)
{
    for (int t = 0; t < 2; t++) { /* 2 px thick */
        draw_line(x + t, y + t, x + w - 1 - t, y + t, r, g, b);
        draw_line(x + t, y + h - 1 - t, x + w - 1 - t, y + h - 1 - t, r, g, b);
        draw_line(x + t, y + t, x + t, y + h - 1 - t, r, g, b);
        draw_line(x + w - 1 - t, y + t, x + w - 1 - t, y + h - 1 - t, r, g, b);
    }
}

/* 5x7 bitmap font for the few characters labels need. */
static const char GLYPH_CHARS[] = "0123456789-.#, ";
static const uint8_t GLYPHS[][7] = {
    {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E}, {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E},
    {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}, {0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E},
    {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}, {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E},
    {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}, {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08},
    {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}, {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C},
    {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00}, {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C},
    {0x0A, 0x0A, 0x1F, 0x0A, 0x1F, 0x0A, 0x0A}, {0x00, 0x00, 0x00, 0x00, 0x0C, 0x04, 0x08},
    {0, 0, 0, 0, 0, 0, 0},
};

static void draw_text(int x, int y, const char *s, uint8_t r, uint8_t g, uint8_t b)
{
    for (; *s; s++, x += 12) {
        const char *c = strchr(GLYPH_CHARS, *s);
        if (!c)
            continue;
        const uint8_t *gl = GLYPHS[c - GLYPH_CHARS];
        for (int row = 0; row < 7; row++)
            for (int col = 0; col < 5; col++)
                if (gl[row] & (0x10 >> col))
                    for (int k = 0; k < 4; k++) /* 2x scale */
                        put_px(x + col * 2 + (k & 1), y + row * 2 + (k >> 1), r, g, b);
    }
}

static void render(const uint8_t *gray, const struct object *obj, int n)
{
    /* Dimmed camera image, with moving areas tinted red. */
    for (int y = 0; y < HEIGHT; y++)
        for (int x = 0; x < WIDTH; x++) {
            int i = y * WIDTH + x, g = gray[i] >> 1;
            uint8_t *p = rgb + 3 * i;
            if (mask[(y / SCALE) * GW + x / SCALE]) {
                p[0] = (uint8_t)(g + 90 > 255 ? 255 : g + 90);
                p[1] = p[2] = (uint8_t)g;
            } else {
                p[0] = p[1] = p[2] = (uint8_t)g;
            }
        }

    /* Principal point: where the camera's optical axis hits the image. */
    int pcx = (int)CX, pcy = (int)CY;
    draw_line(pcx - 6, pcy, pcx + 6, pcy, 255, 255, 255);
    draw_line(pcx, pcy - 6, pcx, pcy + 6, 255, 255, 255);

    for (int i = 0; i < n; i++) {
        const struct object *o = &obj[i];
        draw_rect(o->x, o->y, o->w, o->h, 0, 255, 0);
        /* The ray seen end-on projects to the centroid; draw it from the optical axis for a sense of direction. */
        draw_line(pcx, pcy, (int)o->u, (int)o->v, 0, 200, 255);
        for (int d = -3; d <= 3; d++) {
            put_px((int)o->u + d, (int)o->v, 0, 200, 255);
            put_px((int)o->u, (int)o->v + d, 0, 200, 255);
        }
        char label[64];
        snprintf(label, sizeof label, "#%d %.2f,%.2f,%.2f", o->id, o->ray[0], o->ray[1], o->ray[2]);
        int ty = o->y - 18 < 0 ? o->y + o->h + 4 : o->y - 18;
        draw_text(o->x, ty, label, 255, 255, 0);
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

struct buffer {
    void *start;
    size_t length;
};

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
        xioctl(cam, VIDIOC_QBUF, b);
        *b = newer;
    }
}

/* ---------- HTTP + main loop ---------- */

static int udp_sock = -1;
static struct sockaddr_in udp_dst;

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

static void serve_stream(int client, const char *dev, int threshold, int min_area)
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

    static uint8_t gray[WIDTH * HEIGHT];
    static struct object obj[MAX_BLOBS];
    struct membuf jpg = {0};
    unsigned frames = 0;
    double t0 = now_sec(), t_detect = 0, t_encode = 0;
    bg_ready = 0; /* fresh background for each viewer */

    printf("streaming motion rays (threshold %d, min area %d, fx %.0f)\n", threshold, min_area, FX);
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
            bg_ready = 0;
            continue;
        }

        const uint8_t *yuyv = bufs[b.index].start;
        for (int i = 0; i < WIDTH * HEIGHT; i++)
            gray[i] = yuyv[2 * i];
        xioctl(cam, VIDIOC_QBUF, &b);

        double ta = now_sec();
        downsample(gray);
        double changed = update_motion(threshold);
        int n = 0;
        if (changed > 0.5) {
            /* Most of the image changed at once: auto-exposure jump or the camera moved. Start over. */
            bg_ready = 0;
            memset(mask, 0, sizeof mask);
        } else {
            n = find_blobs(obj, MAX_BLOBS, min_area);
            if (n > MAX_OBJECTS)
                n = MAX_OBJECTS;
            assign_ids(obj, n);
            for (int i = 0; i < n; i++)
                pixel_to_ray(obj[i].u, obj[i].v, obj[i].ray);
        }
        on_rays(frames, obj, n);

        if (udp_sock >= 0)
            for (int i = 0; i < n; i++) {
                char msg[160];
                int len = snprintf(msg, sizeof msg, "%u,%d,%.1f,%.1f,%.5f,%.5f,%.5f,%d,%d,%d,%d\n", frames,
                                   obj[i].id, obj[i].u, obj[i].v, obj[i].ray[0], obj[i].ray[1], obj[i].ray[2],
                                   obj[i].x, obj[i].y, obj[i].w, obj[i].h);
                sendto(udp_sock, msg, len, 0, (struct sockaddr *)&udp_dst, sizeof udp_dst);
            }

        double tb = now_sec();
        render(gray, obj, n);
        jpg.len = 0;
        stbi_write_jpg_to_func(membuf_write, &jpg, WIDTH, HEIGHT, 3, rgb, JPEG_QUALITY);
        double tc = now_sec();
        t_detect += tb - ta;
        t_encode += tc - tb;

        char part[160];
        int len = snprintf(part, sizeof part,
                           "--" BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\nX-Objects: %d\r\n\r\n",
                           jpg.len, n);
        if (write_all(client, part, len) < 0 || write_all(client, jpg.data, jpg.len) < 0 ||
            write_all(client, "\r\n", 2) < 0)
            break;

        frames++;
        if (n > 0 && frames % 6 == 0) {
            for (int i = 0; i < n; i++)
                printf("  #%d box %dx%d at (%d,%d)  ray %.3f %.3f %.3f\n", obj[i].id, obj[i].w, obj[i].h,
                       obj[i].x, obj[i].y, obj[i].ray[0], obj[i].ray[1], obj[i].ray[2]);
        }
        if (frames % 30 == 0) {
            printf("%u frames, %.1f fps | motion %.0f ms, draw+jpeg %.0f ms\n", frames,
                   frames / (now_sec() - t0), 1000 * t_detect / frames, 1000 * t_encode / frames);
            fflush(stdout);
        }
    }
    free(jpg.data);
    if (cam >= 0)
        camera_stop(cam, bufs, nbufs);
}

static const char PAGE[] =
    "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n"
    "<!doctype html><meta name=viewport content=\"width=device-width\"><title>mp157 motion rays</title>"
    "<style>body{margin:0;background:#111;color:#ccc;font:14px system-ui;display:grid;place-items:center;min-height:100vh}"
    "img{max-width:100%;max-height:90vh}a{color:#8f8;margin:0 .5em}</style>"
    "<body><div><img id=v src=\"/stream\"><p style=text-align:center>"
    "red = moving, green = bounding box, blue = ray from the optical axis (+), label = #id ray dx,dy,dz<br>"
    "sensitivity: <a href=# onclick=\"v.src='/stream?thr=15&'+Date.now()\">high</a>"
    "<a href=# onclick=\"v.src='/stream?thr=25&'+Date.now()\">normal</a>"
    "<a href=# onclick=\"v.src='/stream?thr=40&'+Date.now()\">low</a></p></div></body>";

static int query_int(const char *req, const char *key, int def)
{
    const char *p = strstr(req, key);
    return p ? atoi(p + strlen(key)) : def;
}

int main(int argc, char **argv)
{
    const char *dev = argc > 1 ? argv[1] : "auto";
    int port = argc > 2 ? atoi(argv[2]) : 8080;

    if (argc == 5) {
        udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
        udp_dst.sin_family = AF_INET;
        udp_dst.sin_port = htons(atoi(argv[4]));
        if (inet_pton(AF_INET, argv[3], &udp_dst.sin_addr) != 1) {
            fprintf(stderr, "bad UDP IP: %s\n", argv[3]);
            return 2;
        }
        printf("sending rays to UDP %s:%s\n", argv[3], argv[4]);
    }

    signal(SIGPIPE, SIG_IGN);

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
    printf("motion rays on http://0.0.0.0:%d/\n", port);
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
            int thr = query_int(req, "thr=", DEFAULT_THRESHOLD);
            int min_area = query_int(req, "min=", DEFAULT_MIN_AREA);
            printf("viewer connected\n");
            fflush(stdout);
            serve_stream(client, dev, thr < 1 ? 1 : thr, min_area < 1 ? 1 : min_area);
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
