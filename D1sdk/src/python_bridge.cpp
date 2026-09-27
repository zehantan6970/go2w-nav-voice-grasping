#include <iostream>
#include <string>
#include <vector>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"

#define TOPIC "rt/arm_Command"

using namespace unitree::robot;
using namespace unitree::common;

int main(int argc, char* argv[]) {
    // 檢查參數數量 (程式名 + 7 個角度)
    if (argc < 8) {
        std::cerr << "錯誤: 需要 7 個角度參數" << std::endl;
        return -1;
    }

    // 指定網口 eth0（D1 機械臂與 Go2-W 主控共用同一 DDS 域）
    // 避免 CycloneDDS 自動猜測網口導致 discovery 失敗
    ChannelFactory::Instance()->Init(0, "eth0");
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC);
    publisher.InitChannel();

    // 等待 DDS discovery 完成（subscriber 發現我們的 publisher）
    // discovery 週期通常需要 100~500ms
    Sleep(0.3);

    // 組裝 JSON
    std::string json_cmd = "{\"seq\":4,\"address\":1,\"funcode\":2,\"data\":{\"mode\":1";
    
    for(int i = 0; i < 7; ++i) {
        json_cmd += ",\"angle" + std::to_string(i) + "\":" + std::string(argv[i+1]);
    }
    json_cmd += "}}";

    // 建立訊息
    unitree_arm::msg::dds_::ArmString_ msg{};
    msg.data_() = json_cmd;

    // 多次發送確保送達（BEST_EFFORT 下首次 write 可能因 discovery 未完成而丟失）
    for (int i = 0; i < 3; i++) {
        publisher.Write(msg);
        Sleep(0.05);
    }

    // 保持 process 活著，讓 DDS 有足夠時間完成傳輸
    // 避免 process 退出後未送達的 data 被丟棄
    Sleep(0.3);

    std::cout << "已發送: " << json_cmd << std::endl;

    return 0;
}
