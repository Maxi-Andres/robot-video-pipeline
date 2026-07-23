// go2_jpeg_stream — pull the Go2 front camera as JPEG via the Unitree SDK video
// service (GetImageSample, request/response over DDS) and write a concatenated MJPEG
// stream to stdout. Meant to be piped into ffmpeg to encode H.264 and push RTSP:
//
//   go2_jpeg_stream enp4s0 | ffmpeg -f mjpeg -i pipe:0 -c:v libx264 ... -f rtsp ...
//
// Why this and not the native H.264 topic (rt/frontvideostream): on this Go2 the
// large H.264 DDS samples do not deliver cleanly — the ROS2/cyclonedds bridge
// deserializes them corrupt, and a native SDK subscriber matches the writer but
// never reassembles a sample. The videohub JPEG path is small, reliable, and is what
// the AI-VL camera bridge already uses. Trade-off: one JPEG->H.264 re-encode.
//
// Usage:  go2_jpeg_stream [network_interface] [max_fps]
//   network_interface  NIC on the robot network (default "enp4s0")
//   max_fps            cap the poll rate (default 0 = as fast as the robot answers)

#include <unitree/robot/go2/video/video_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <cstring>
#include <string>
#include <vector>
#include <ctime>
#include <unistd.h>

using namespace unitree::robot;

static void nsleep(long ns) { timespec t{0, ns}; nanosleep(&t, nullptr); }

int main(int argc, char** argv) {
    signal(SIGPIPE, SIG_IGN);  // ffmpeg gone -> stop cleanly, don't crash

    const std::string nic = (argc > 1) ? argv[1] : "enp4s0";
    const double max_fps  = (argc > 2) ? atof(argv[2]) : 0.0;
    const long min_gap_ns = (max_fps > 0.0) ? (long)(1e9 / max_fps) : 0;

    fprintf(stderr, "[go2_jpeg_stream] nic=%s max_fps=%.1f\n", nic.c_str(), max_fps);

    ChannelFactory::Instance()->Init(0, nic);
    go2::VideoClient vc;
    vc.SetTimeout(1.0f);
    vc.Init();

    std::vector<uint8_t> img, prev;
    long sent = 0, empty = 0;
    while (true) {
        img.clear();
        int r = vc.GetImageSample(img);
        if (r != 0 || img.size() < 4 || img[0] != 0xFF || img[1] != 0xD8) {
            if (++empty % 30 == 0) fprintf(stderr, "[go2_jpeg_stream] no frame (ret=%d)\n", r);
            nsleep(20 * 1000 * 1000);  // 20 ms backoff on a miss
            continue;
        }
        // Skip byte-identical repeats (the service can return the same frame if we
        // poll faster than the camera updates) so we don't feed ffmpeg duplicates.
        if (img.size() == prev.size() && memcmp(img.data(), prev.data(), img.size()) == 0) {
            nsleep(2 * 1000 * 1000);
            continue;
        }
        if (fwrite(img.data(), 1, img.size(), stdout) != img.size()) return 0;
        fflush(stdout);
        prev = img;
        if (++sent % 100 == 0) fprintf(stderr, "[go2_jpeg_stream] %ld frames\n", sent);
        if (min_gap_ns) nsleep(min_gap_ns);
    }
    return 0;
}
