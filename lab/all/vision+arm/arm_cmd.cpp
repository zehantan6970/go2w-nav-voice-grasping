#include <unitree/robot/channel/channel_publisher.hpp>
#include "msg/ArmString_.hpp"
#define TOPIC "rt/arm_Command"
using namespace unitree::robot;
using namespace unitree::common;
int main(int argc, char *argv[]) {
    int seq = 1; float a[7] = {0};
    if (argc < 8) { fprintf(stderr, "用法: arm_cmd <seq> <a0..a6>\n"); return 1; }
    seq = atoi(argv[1]);
    for (int i = 0; i < 7; i++) a[i] = atof(argv[i + 2]);
    ChannelFactory::Instance()->Init(0);
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC);
    publisher.InitChannel();
    char buf[512];
    snprintf(buf, sizeof(buf),
        "{\"seq\":%d,\"address\":1,\"funcode\":2,\"data\":{\"mode\":1"
        ",\"angle0\":%.2f,\"angle1\":%.2f,\"angle2\":%.2f"
        ",\"angle3\":%.2f,\"angle4\":%.2f,\"angle5\":%.2f,\"angle6\":%.2f}}",
        seq, a[0], a[1], a[2], a[3], a[4], a[5], a[6]);
    unitree_arm::msg::dds_::ArmString_ msg{};
    msg.data_() = buf;
    publisher.Write(msg);
    return 0;
}
