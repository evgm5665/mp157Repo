/*
 * mjpeg_stream.c - template: live USB camera stream to a browser over HTTP.
 *
 * Build on the board:   zig cc -O2 mjpeg_stream.c -o mjpeg_stream
 * Run:                  ./mjpeg_stream [camera|auto] [port]      (defaults: auto, 8080)
 * View on the laptop:   http://<board-ip>:8080/        (any browser, or: mpv http://<board-ip>:8080/stream)
 *
 * The camera already produces JPEG frames (MJPEG), so they are sent as-is with
 * no re-encoding: cheap on the CPU. One viewer at a time; when the viewer
 * disconnects the camera is released and the server waits for the next one.
 *
 * "auto" picks the first USB camera by its stable /dev/v4l/by-id name, so
 * replugging it (which can change /dev/videoN) doesn't break anything. If the
 * camera is unplugged mid-stream, the viewer stays connected and the picture
 * resumes when it's plugged back in.
 */
#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <sys/select.h>
#include <linux/videodev2.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define WIDTH    640
#define HEIGHT   480
#define NBUFFERS 4
#define BOUNDARY "mjpegframe"
#define FRAME_TIMEOUT_S 2 /* no frame for this long = camera unplugged/stalled */

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

/* Open the camera in MJPEG mode and start streaming. Returns fd or -1 (quietly if it's just not plugged in). */
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
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0 || fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG) {
        fprintf(stderr, "camera does not support MJPEG\n");
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
    printf("camera %s: %ux%u MJPEG\n", dev, fmt.fmt.pix.width, fmt.fmt.pix.height);
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

static const char PAGE[] =
    "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n"
    "<!doctype html><title>mp157 camera</title>"
    "<body style=\"margin:0;background:#111;display:grid;place-items:center;height:100vh\">"
    "<img src=\"/stream\" style=\"max-width:100%;max-height:100vh\"></body>";

/* Wait up to FRAME_TIMEOUT_S for the next frame. Returns 0 with `b` filled, or -1 if the camera stalled or vanished. */
static int next_frame(int cam, struct v4l2_buffer *b)
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
    return xioctl(cam, VIDIOC_DQBUF, b);
}

/* (Re)open the camera, retrying every second until it shows up or the viewer leaves. */
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
        /* Probe the viewer: a zero-length send fails once the browser has closed. */
        if (send(client, "", 0, MSG_NOSIGNAL) < 0)
            return -1;
        sleep(1);
    }
}

static void serve_stream(int client, const char *dev)
{
    const char *hdr = "HTTP/1.0 200 OK\r\n"
                      "Cache-Control: no-cache\r\n"
                      "Content-Type: multipart/x-mixed-replace; boundary=" BOUNDARY "\r\n\r\n";
    if (write_all(client, hdr, strlen(hdr)) < 0)
        return;

    struct buffer bufs[NBUFFERS];
    unsigned nbufs = 0;
    int cam = camera_wait(client, dev, bufs, &nbufs);
    if (cam < 0)
        return;

    {
        unsigned frames = 0;
        double t0 = now_sec();
        for (;;) {
            struct v4l2_buffer b;
            if (next_frame(cam, &b) < 0) {
                /* Camera unplugged or stalled: release it and wait for it to come back. */
                printf("camera lost, reopening\n");
                fflush(stdout);
                camera_stop(cam, bufs, nbufs);
                cam = camera_wait(client, dev, bufs, &nbufs);
                if (cam < 0)
                    return;
                continue;
            }

            char part[128];
            int n = snprintf(part, sizeof part,
                             "--" BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                             b.bytesused);
            int failed = write_all(client, part, n) < 0 ||
                         write_all(client, bufs[b.index].start, b.bytesused) < 0 ||
                         write_all(client, "\r\n", 2) < 0;
            xioctl(cam, VIDIOC_QBUF, &b);
            if (failed)
                break; /* viewer went away */

            if (++frames % 100 == 0) {
                printf("streamed %u frames (%.1f fps)\n", frames, frames / (now_sec() - t0));
                fflush(stdout);
            }
        }
    }
    camera_stop(cam, bufs, nbufs);
}

int main(int argc, char **argv)
{
    const char *dev = argc > 1 ? argv[1] : "auto";
    int port = argc > 2 ? atoi(argv[2]) : 8080;

    signal(SIGPIPE, SIG_IGN); /* a closed browser tab must not kill the server */

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
    printf("streaming %s on http://0.0.0.0:%d/\n", dev, port);
    fflush(stdout);

    for (;;) {
        int client = accept(srv, NULL, NULL);
        if (client < 0)
            continue;

        char req[1024] = {0};
        ssize_t n = read(client, req, sizeof req - 1);
        if (n > 0 && strncmp(req, "GET /stream", 11) == 0) {
            printf("viewer connected\n");
            fflush(stdout);
            serve_stream(client, dev);
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
