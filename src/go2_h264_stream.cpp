// go2_h264_stream — pull the Go2 front-camera native H.264 over the Unitree
// native DDS SDK and write the raw Annex-B elementary stream to stdout.
//
// This deliberately bypasses the ROS2/cyclonedds bridge (which corrupts the large
// H.264 byte arrays); the native DDS subscriber delivers the NALs intact. The
// stdout stream is meant to be piped straight into an RTSP muxer with NO transcode
// (e.g. `ffmpeg -f h264 -i pipe:0 -c copy -f rtsp ...`) for lowest latency.
//
// Usage:  go2_h264_stream [network_interface] [ladder]
//   network_interface  NIC on the robot network (default "enp4s0")
//   ladder             video720p | video360p | video180p (default video720p)

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/Go2FrontVideoData_.hpp>

#include <cstdio>
#include <cstring>
#include <csignal>
#include <string>
#include <unistd.h>

using namespace unitree::robot;
using VideoMsg = unitree_go::msg::dds_::Go2FrontVideoData_;

#define TOPIC_VIDEO "rt/frontvideostream"

static std::string g_ladder = "video720p";

// Pick the requested resolution ladder from one message.
static const std::vector<uint8_t>& pick(const VideoMsg* m) {
    if (g_ladder == "video360p") return m->video360p();
    if (g_ladder == "video180p") return m->video180p();
    return m->video720p();
}

static void on_video(const void* msg) {
    const auto* m = static_cast<const VideoMsg*>(msg);
    static long n = 0;
    if (getenv("GO2_DEBUG") && n < 10) {
        fprintf(stderr, "[go2_h264_stream] msg #%ld  720p=%zu 360p=%zu 180p=%zu\n",
                n, m->video720p().size(), m->video360p().size(), m->video180p().size());
    }
    ++n;
    const std::vector<uint8_t>& nal = pick(m);
    if (nal.empty()) return;
    // Write the H.264 payload straight through; the downstream muxer parses the
    // Annex-B start codes itself. A short write (e.g. broken pipe) ends the process.
    if (fwrite(nal.data(), 1, nal.size(), stdout) != nal.size()) {
        _exit(0);
    }
    fflush(stdout);
}

int main(int argc, char** argv) {
    // If the consumer (ffmpeg) goes away, exit quietly instead of dying on SIGPIPE.
    signal(SIGPIPE, SIG_IGN);

    const std::string nic = (argc > 1) ? argv[1] : "enp4s0";
    if (argc > 2) g_ladder = argv[2];

    // All H.264 goes to stdout; keep logs on stderr so the pipe stays clean.
    fprintf(stderr, "[go2_h264_stream] nic=%s ladder=%s topic=%s\n",
            nic.c_str(), g_ladder.c_str(), TOPIC_VIDEO);

    ChannelFactory::Instance()->Init(0, nic);

    ChannelSubscriber<VideoMsg> sub(TOPIC_VIDEO);
    sub.InitChannel(on_video, 1);  // queuelen 1: always the freshest frame

    while (true) sleep(1);
    return 0;
}
