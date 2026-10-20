/*
 * opencv_camera.cpp - template: read a USB camera with OpenCV and process frames.
 *
 * OpenCV 4 has no C API, so this one is C++.
 *
 * Board setup (one time): ST's package feed has the OpenCV 4.5 libraries but no
 * C++ compiler or headers, so this needs a build setup before it compiles.
 * The quickest way to use OpenCV on the board is Python instead:
 *     apt-get install python3-opencv
 * (same API: cv2.VideoCapture, cv2.cvtColor, ...).
 *
 * Build (once a C++ toolchain + OpenCV headers are set up):
 *     g++ -O2 opencv_camera.cpp -o opencv_camera \
 *         -lopencv_core -lopencv_imgproc -lopencv_videoio -lopencv_imgcodecs
 * Run:
 *     ./opencv_camera [camera-index] [frames]
 *
 * Grabs frames, converts to grayscale, runs edge detection, and saves the last
 * result as edges.png. Put your own processing in process_frame().
 */
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

/* Called for every frame. Return the image you want saved/shown. */
static cv::Mat process_frame(const cv::Mat &frame)
{
    cv::Mat gray, edges;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 1.5);
    cv::Canny(gray, edges, 50, 150);
    return edges;
}

int main(int argc, char **argv)
{
    int index = argc > 1 ? std::atoi(argv[1]) : 0; /* 0 = /dev/video0 */
    int nframes = argc > 2 ? std::atoi(argv[2]) : 60;

    cv::VideoCapture cap(index, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::fprintf(stderr, "could not open /dev/video%d\n", index);
        return 1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    cv::Mat frame, result;
    auto t0 = std::chrono::steady_clock::now();
    for (int n = 0; n < nframes; n++) {
        if (!cap.read(frame) || frame.empty()) {
            std::fprintf(stderr, "frame grab failed\n");
            return 1;
        }
        result = process_frame(frame);
    }
    double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("%d frames %dx%d in %.2fs (%.1f fps)\n", nframes, frame.cols, frame.rows, dt, nframes / dt);

    if (!result.empty()) {
        cv::imwrite("edges.png", result);
        std::printf("saved edges.png\n");
    }
    return 0;
}
