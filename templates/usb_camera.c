/*
 * usb_camera.c - template: grab frames from a USB (UVC) webcam with V4L2.
 *
 * Build on the board:   zig cc -O2 usb_camera.c -o usb_camera
 * Run:                  ./usb_camera [/dev/video0] [frames]
 *
 * Captures `frames` frames (default 30), prints the frame rate, and saves the
 * last frame to frame.jpg (MJPEG cameras) or frame.yuyv (raw YUYV cameras).
 * View a raw frame on a PC with:
 *   ffmpeg -f rawvideo -pix_fmt yuyv422 -s 640x480 -i frame.yuyv frame.png
 *
 * Find your camera with: ls /dev/video*   (plug it in first; the uvcvideo
 * driver loads automatically). Put your per-frame processing in process_frame().
 */
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#define WIDTH    640
#define HEIGHT   480
#define NBUFFERS 4

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

static void die(const char *what)
{
    perror(what);
    exit(1);
}

/* Called for every captured frame. `data` is MJPEG or YUYV depending on `fourcc`. */
static void process_frame(const void *data, size_t len, unsigned fourcc, unsigned frame_no)
{
    (void)data;
    (void)fourcc;
    if (frame_no % 10 == 0)
        printf("frame %u: %zu bytes\n", frame_no, len);
}

static double now_sec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

int main(int argc, char **argv)
{
    const char *dev = argc > 1 ? argv[1] : "/dev/video0";
    unsigned nframes = argc > 2 ? (unsigned)atoi(argv[2]) : 30;

    int fd = open(dev, O_RDWR);
    if (fd < 0)
        die(dev);

    struct v4l2_capability cap = {0};
    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) < 0)
        die("VIDIOC_QUERYCAP");
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) || !(cap.capabilities & V4L2_CAP_STREAMING)) {
        fprintf(stderr, "%s is not a streaming capture device\n", dev);
        return 1;
    }
    printf("camera: %s (%s)\n", cap.card, cap.driver);

    /* Prefer MJPEG (smaller, most webcams support it at higher resolutions), fall back to YUYV. */
    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = WIDTH;
    fmt.fmt.pix.height = HEIGHT;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0 || fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG) {
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0)
            die("VIDIOC_S_FMT");
    }
    unsigned fourcc = fmt.fmt.pix.pixelformat;
    printf("format: %ux%u %.4s\n", fmt.fmt.pix.width, fmt.fmt.pix.height, (char *)&fourcc);

    /* Memory-mapped buffers shared with the driver. */
    struct v4l2_requestbuffers req = {0};
    req.count = NBUFFERS;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0)
        die("VIDIOC_REQBUFS");

    struct buffer bufs[NBUFFERS];
    for (unsigned i = 0; i < req.count; i++) {
        struct v4l2_buffer b = {0};
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index = i;
        if (xioctl(fd, VIDIOC_QUERYBUF, &b) < 0)
            die("VIDIOC_QUERYBUF");
        bufs[i].length = b.length;
        bufs[i].start = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
        if (bufs[i].start == MAP_FAILED)
            die("mmap");
        if (xioctl(fd, VIDIOC_QBUF, &b) < 0)
            die("VIDIOC_QBUF");
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_STREAMON, &type) < 0)
        die("VIDIOC_STREAMON");

    double t0 = now_sec();
    struct v4l2_buffer last = {0};
    for (unsigned n = 0; n < nframes; n++) {
        struct v4l2_buffer b = {0};
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd, VIDIOC_DQBUF, &b) < 0) /* blocks until a frame is ready */
            die("VIDIOC_DQBUF");

        process_frame(bufs[b.index].start, b.bytesused, fourcc, n);

        if (n == nframes - 1) {
            last = b; /* keep the last frame to save; don't requeue it */
        } else if (xioctl(fd, VIDIOC_QBUF, &b) < 0) {
            die("VIDIOC_QBUF");
        }
    }
    double dt = now_sec() - t0;
    printf("%u frames in %.2fs (%.1f fps)\n", nframes, dt, nframes / dt);

    if (nframes > 0) {
        const char *out = fourcc == V4L2_PIX_FMT_MJPEG ? "frame.jpg" : "frame.yuyv";
        FILE *f = fopen(out, "wb");
        if (f) {
            fwrite(bufs[last.index].start, 1, last.bytesused, f);
            fclose(f);
            printf("saved %s\n", out);
        }
    }

    xioctl(fd, VIDIOC_STREAMOFF, &type);
    for (unsigned i = 0; i < req.count; i++)
        munmap(bufs[i].start, bufs[i].length);
    close(fd);
    return 0;
}
