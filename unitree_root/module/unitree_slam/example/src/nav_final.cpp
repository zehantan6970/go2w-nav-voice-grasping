#include <unitree/robot/client/client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/ros2/String_.hpp>
#include <json.hpp>
#include <termio.h>
#include <string>
#include <future>
#include <thread>
#include <fstream>
#include <dirent.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>

#define SlamInfoTopic "rt/slam_info"
#define SlamKeyInfoTopic "rt/slam_key_info"
#define TTS_API_KEY "sk-cf9…b7ca"
#define TTS_VOICE "Cherry"

using namespace unitree::robot;
using namespace unitree::common;

// ─── ANSI colors ───
#define GRN "\033[32m"
#define YEL "\033[33m"
#define CYN "\033[36m"
#define RED "\033[31m"
#define BLD "\033[1m"
#define RST "\033[0m"

class poseDate
{
public:
    float x = 0.0f, y = 0.0f, z = 0.0f;
    float q_x = 0.0f, q_y = 0.0f, q_z = 0.0f, q_w = 1.0f;
    int mode = 0;
    float speed = 0.6f;
    std::string tts_text = "";
    std::string action = "";

    std::string toJsonStr() const
    {
        nlohmann::json j;
        j["data"]["targetPose"]["x"] = x;
        j["data"]["targetPose"]["y"] = y;
        j["data"]["targetPose"]["z"] = z;
        j["data"]["targetPose"]["q_x"] = q_x;
        j["data"]["targetPose"]["q_y"] = q_y;
        j["data"]["targetPose"]["q_z"] = q_z;
        j["data"]["targetPose"]["q_w"] = q_w;
        j["data"]["mode"] = mode;
        j["data"]["speed"] = speed;
        return j.dump(4);
    }
    void printInfo() const
    {
        printf("  x:%.3f y:%.3f  yaw:%.0f°  %s%s%s\n",
               x, y, std::atan2(2*(q_w*q_z), 1-2*q_z*q_z)*180/M_PI,
               tts_text.empty() ? "" : "\"", tts_text.c_str(), tts_text.empty() ? "" : "\"");
    }
};

namespace unitree::robot::slam
{
    const std::string TEST_SERVICE_NAME = "slam_operate";
    const std::string TEST_API_VERSION = "1.0.0.1";

    const int32_t ROBOT_API_ID_STOP_NODE = 1901;
    const int32_t ROBOT_API_ID_START_MAPPING_PL = 1801;
    const int32_t ROBOT_API_ID_END_MAPPING_PL = 1802;
    const int32_t ROBOT_API_ID_START_RELOCATION_PL = 1804;
    const int32_t ROBOT_API_ID_POSE_NAV_PL = 1102;
    const int32_t ROBOT_API_ID_PAUSE_NAV = 1201;
    const int32_t ROBOT_API_ID_RESUME_NAV = 1202;

    class TestClient : public Client
    {
    private:
        ChannelSubscriberPtr<std_msgs::msg::dds_::String_> subSlamInfo, subSlamKeyInfo;
        void slamInfoHandler(const void *message);
        void slamKeyInfoHandler(const void *message);
        poseDate curPose;
        std::vector<poseDate> poseList;
        bool is_arrived = false, threadControl = false;
        std::future<void> futThread;
        std::promise<void> prom;
        std::thread controlThread;
        std::string currentMapName;

        void playTTS(const std::string& text);
        void saveTasks();
        void loadTasks();
        std::vector<std::string> listMaps();

    public:
        TestClient();
        ~TestClient();
        void Init();
        unsigned char keyDetection();
        unsigned char keyExecute();
        void stopNodeFun();
        void startMappingPlFun();
        void endMappingPlFun();
        void relocationPlFun();
        void taskLoopFun(std::promise<void> &prom, bool loopMode);
        void taskLoopFunSingle(std::promise<void> &prom);
        void pauseNavFun();
        void resumeNavFun();
        void taskThreadRun(bool loopMode);
        void taskThreadRunSingle();
        void taskThreadStop();
    };

    TestClient::TestClient() : Client(TEST_SERVICE_NAME, false)
    {
        subSlamInfo = ChannelSubscriberPtr<std_msgs::msg::dds_::String_>(
            new ChannelSubscriber<std_msgs::msg::dds_::String_>(SlamInfoTopic));
        subSlamInfo->InitChannel(std::bind(&TestClient::slamInfoHandler, this, std::placeholders::_1), 1);
        subSlamKeyInfo = ChannelSubscriberPtr<std_msgs::msg::dds_::String_>(
            new ChannelSubscriber<std_msgs::msg::dds_::String_>(SlamKeyInfoTopic));
        subSlamKeyInfo->InitChannel(std::bind(&TestClient::slamKeyInfoHandler, this, std::placeholders::_1), 1);

        printf("\n" BLD "==============================" RST "\n");
        printf("  🐕 小創導航 (基於 keyDemo)\n");
        printf(BLD "==============================" RST "\n");
        printf("  %sq%s: 開始建圖    %ss%s: 記錄位址     %sl%s: 列表\n", GRN, RST, GRN, RST, GRN, RST);
        printf("  %sw%s: 結束保存    %sc%s: 單次巡航     %sf%s: 清空\n", GRN, RST, GRN, RST, GRN, RST);
        printf("  %sa%s: 載入地圖    %sd%s: 循環巡航     %sr%s: 移除末點\n", GRN, RST, GRN, RST, GRN, RST);
        printf("  %sz%s: 暫停巡航    %sx%s: 恢復巡航\n", GRN, RST, GRN, RST);
        printf("  其他鍵: 停止\n");
        printf(BLD "==============================\n" RST);
    }

    TestClient::~TestClient() { stopNodeFun(); }

    void TestClient::Init()
    {
        SetApiVersion(TEST_API_VERSION);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_POSE_NAV_PL);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_PAUSE_NAV);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_RESUME_NAV);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_STOP_NODE);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_START_MAPPING_PL);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_END_MAPPING_PL);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_START_RELOCATION_PL);
    }

    // ════════════════════════════════════════════
    //  TTS — 阿里雲百煉
    // ════════════════════════════════════════════
    void TestClient::playTTS(const std::string& text)
    {
        if (text.empty() || std::string(TTS_API_KEY).empty()) return;

        std::cout << CYN "[TTS] " << text << RST << std::endl;

        // DJB2 hash for cache filename
        unsigned long hash = 5381;
        for (char c : text) hash = ((hash << 5) + hash) + (unsigned char)c;
        std::string wavPath = "/tmp/tts_nav_" + std::to_string(hash) + ".wav";

        // Check cache
        std::ifstream cacheCheck(wavPath, std::ios::binary | std::ios::ate);
        if (cacheCheck.is_open()) {
            long sz = static_cast<long>(cacheCheck.tellg()); cacheCheck.close();
            if (sz >= 1000) {
                std::system(("paplay " + wavPath + " >/dev/null 2>&1").c_str());
                return;
            }
        }

        // Generate bridge script if not exists
        static bool scriptReady = false;
        const std::string pyScript = "/tmp/aliyun_tts_bridge.py";
        if (!scriptReady) {
            std::ofstream pyf(pyScript);
            pyf << R"PY(#!/usr/bin/env python3
import websocket, json, base64, wave, sys, os
def synth(text, out, key, voice):
    try:
        ws = websocket.create_connection(
            "wss://dashscope-intl.aliyuncs.com/api-ws/v1/realtime?model=qwen3-tts-flash-realtime",
            header=[f"Authorization: Bearer {key}"], timeout=10)
    except: return False
    chunks = []
    ws.send(json.dumps({"type":"session.update","session":{"voice":voice,"response_format":"pcm","sample_rate":24000,"language_type":"Chinese"}}))
    ws.send(json.dumps({"type":"input_text_buffer.append","text":text}))
    ws.send(json.dumps({"type":"input_text_buffer.commit"}))
    while True:
        try:
            m = json.loads(ws.recv())
            t = m.get("type","")
            if t in ("response.audio.delta","output_audio.delta"):
                d = m.get("delta") or m.get("audio")
                if d: chunks.append(base64.b64decode(d))
            elif t in ("response.completed","response.done","session.finished"): break
        except: break
    ws.close()
    pcm = b"".join(chunks)
    if not pcm: return False
    with wave.open(out,"wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(24000); w.writeframes(pcm)
    return True
if __name__=="__main__":
    ok = synth(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4] if len(sys.argv)>4 else "Cherry")
    sys.exit(0 if ok else 1)
)PY";
            pyf.close();
            ::chmod(pyScript.c_str(), 0755);
            scriptReady = true;
        }

        std::string cmd = "python3 " + pyScript + " \"" + text + "\" " + wavPath
                         + " " + TTS_API_KEY + " " + TTS_VOICE + " 2>/dev/null";
        std::system(cmd.c_str());

        std::ifstream check(wavPath, std::ios::binary | std::ios::ate);
        if (!check.is_open() || check.tellg() < 1000) {
            std::cout << RED "[TTS] 合成失敗" RST << std::endl;
            check.close(); return;
        }
        check.close();
        std::system(("paplay " + wavPath + " >/dev/null 2>&1").c_str());
    }

    // ════════════════════════════════════════════
    //  Task file management
    // ════════════════════════════════════════════
    std::vector<std::string> TestClient::listMaps()
    {
        std::vector<std::string> maps;
        DIR* dir = opendir("/home/unitree/lab/map/");
        if (!dir) return maps;
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            std::string name = entry->d_name;
            if (name.size() > 4 && name.substr(name.size()-4) == ".pgm")
                maps.push_back(name.substr(0, name.size()-4));
        }
        closedir(dir);
        std::sort(maps.begin(), maps.end());
        return maps;
    }

    void TestClient::saveTasks()
    {
        if (currentMapName.empty()) return;
        std::string path = "/home/unitree/lab/map/" + currentMapName + "_task.json";
        nlohmann::json j;
        j["count"] = poseList.size();
        j["points"] = nlohmann::json::array();
        for (const auto& p : poseList) {
            nlohmann::json pt;
            pt["x"] = p.x; pt["y"] = p.y; pt["z"] = p.z;
            pt["q_x"] = p.q_x; pt["q_y"] = p.q_y; pt["q_z"] = p.q_z; pt["q_w"] = p.q_w;
            pt["mode"] = p.mode; pt["speed"] = p.speed;
            pt["tts_text"] = p.tts_text; pt["action"] = p.action;
            j["points"].push_back(pt);
        }
        std::ofstream f(path); f << j.dump(2); f.close();
        std::cout << GRN "[Saved] " << poseList.size() << " points -> " << path << RST << std::endl;
    }

    void TestClient::loadTasks()
    {
        if (currentMapName.empty()) { /* default from env */ return; }
        std::string path = "/home/unitree/lab/map/" + currentMapName + "_task.json";
        std::ifstream f(path);
        if (!f.is_open()) {
            std::cout << YEL "[No task file for map: " << currentMapName << "]" << RST << std::endl; return;
        }
        nlohmann::json j; f >> j; f.close();
        poseList.clear();
        for (const auto& pt : j["points"]) {
            poseDate p;
            p.x = pt["x"]; p.y = pt["y"]; p.z = pt["z"];
            p.q_x = pt["q_x"]; p.q_y = pt["q_y"]; p.q_z = pt["q_z"]; p.q_w = pt["q_w"];
            p.mode = pt.value("mode", 0);
            p.speed = pt.value("speed", 0.6f);
            p.tts_text = pt.value("tts_text", "");
            p.action = pt.value("action", "");
            poseList.push_back(p);
        }
        std::cout << GRN "[Loaded] " << poseList.size() << " points from " << path << RST << std::endl;
    }

    // ════════════════════════════════════════════
    //  Slam Info Handler
    // ════════════════════════════════════════════
    void TestClient::slamInfoHandler(const void *message)
    {
        auto& msg = *(std_msgs::msg::dds_::String_*)message;
        auto j = nlohmann::json::parse(msg.data());
        if (j["errorCode"] != 0 || j["type"] != "pos_info") return;
        auto& p = j["data"]["currentPose"];
        curPose.x = p["x"]; curPose.y = p["y"]; curPose.z = p["z"];
        curPose.q_x = p["q_x"]; curPose.q_y = p["q_y"];
        curPose.q_z = p["q_z"]; curPose.q_w = p["q_w"];

        // Write for web display
        static int writeCount = 0;
        if (++writeCount % 5 == 0) {
            std::ofstream pf("/tmp/robot_pose.json");
            pf << j["data"]["currentPose"].dump(); pf.close();
        }
    }

    void TestClient::slamKeyInfoHandler(const void *message)
    {
        auto& msg = *(std_msgs::msg::dds_::String_*)message;
        auto j = nlohmann::json::parse(msg.data());
        if (j["errorCode"] != 0) return;
        if (j["type"] == "task_result") {
            is_arrived = j["data"]["is_arrived"];
            std::string target = j["data"]["targetNodeName"];
            if (is_arrived) {
                printf(GRN "\n✅ 已到達 %s\n" RST, target.c_str());
                playTTS("已到達" + target);
            } else {
                printf(YEL "\n❌ 未到達 %s\n" RST, target.c_str());
            }
        }
    }

    // ════════════════════════════════════════════
    //  RPC functions
    // ════════════════════════════════════════════
    void TestClient::stopNodeFun()
    {
        std::string p = R"({"data": {}})", d;
        int32_t c = Call(ROBOT_API_ID_STOP_NODE, p, d);
        std::cout << "stopNode: " << c << std::endl;
    }
    void TestClient::startMappingPlFun()
    {
        std::string p = R"({"data": {"slam_type": "indoor"}})", d;
        int32_t c = Call(ROBOT_API_ID_START_MAPPING_PL, p, d);
        printf(GRN "[建圖] code=%d\n" RST, c);
    }
    void TestClient::endMappingPlFun()
    {
        std::string addr = "/home/unitree/lab/map/mymap_" + std::to_string(time(nullptr)) + ".pcd";
        nlohmann::json j; j["data"]["address"] = addr;
        std::string d;
        int32_t c = Call(ROBOT_API_ID_END_MAPPING_PL, j.dump(), d);
        printf(GRN "[保存] %s  code=%d\n" RST, addr.c_str(), c);
    }
    void TestClient::relocationPlFun()
    {
        std::string d;
        nlohmann::json j;
        j["data"]["x"] = 0; j["data"]["y"] = 0; j["data"]["z"] = 0;
        j["data"]["q_x"] = 0; j["data"]["q_y"] = 0; j["data"]["q_z"] = 0; j["data"]["q_w"] = 1;
        j["data"]["address"] = "/home/unitree/lab/map/" + currentMapName + ".pcd";
        int32_t c = Call(ROBOT_API_ID_START_RELOCATION_PL, j.dump(), d);
        printf(GRN "[重定位] code=%d\n" RST, c);
    }
    void TestClient::pauseNavFun()
    {
        std::string p = R"({"data": {}})", d;
        int32_t c = Call(ROBOT_API_ID_PAUSE_NAV, p, d);
        printf(YEL "[暫停] code=%d\n" RST, c);
    }
    void TestClient::resumeNavFun()
    {
        std::string p = R"({"data": {}})", d;
        int32_t c = Call(ROBOT_API_ID_RESUME_NAV, p, d);
        printf(GRN "[恢復] code=%d\n" RST, c);
    }

    // ════════════════════════════════════════════
    //  Navigation - Loop mode
    // ════════════════════════════════════════════
    void TestClient::taskThreadRun(bool loopMode)
    {
        taskThreadStop();
        prom = std::promise<void>();
        futThread = prom.get_future();
        if (loopMode)
            controlThread = std::thread(&TestClient::taskLoopFun, this, std::ref(prom), true);
        else
            controlThread = std::thread(&TestClient::taskLoopFunSingle, this, std::ref(prom));
        controlThread.detach();
    }

    void TestClient::taskThreadRunSingle() { taskThreadRun(false); }

    void TestClient::taskLoopFun(std::promise<void> &prom, bool loopMode)
    {
        std::string data;
        threadControl = true;
        printf(CYN "\n🚀 巡航開始 (%zu 點, %s)\n" RST,
               poseList.size(), loopMode ? "循環" : "單次");

        int i = 0;
        int n = (int)poseList.size();
        while (threadControl) {
            is_arrived = false;

            // TTS departure
            if (!poseList[i].tts_text.empty())
                playTTS("正在前往" + poseList[i].tts_text);

            int32_t sc = Call(ROBOT_API_ID_POSE_NAV_PL, poseList[i].toJsonStr(), data);
            printf(CYN "[%d/%d] → code=%d\n" RST, i, n, sc);

            if (sc != 0) { i++; if (i>=n) break; continue; }

            // Wait with timeout 120s
            int timeout = 12000; // × 10ms
            while (!is_arrived && threadControl && timeout > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                timeout--;
            }
            if (!is_arrived) playTTS("無法到達此點");

            i++;
            if (loopMode && i >= n) { i = 0; std::reverse(poseList.begin(), poseList.end()); printf(CYN "↔ 反轉路徑\n" RST); }
            else if (!loopMode && i >= n) break;
        }
        printf(GRN "🏁 巡航結束\n" RST);
        prom.set_value();
    }

    void TestClient::taskLoopFunSingle(std::promise<void> &prom) { taskLoopFun(prom, false); }

    void TestClient::taskThreadStop()
    {
        threadControl = false;
        if (futThread.valid()) {
            auto s = futThread.wait_for(std::chrono::milliseconds(0));
            if (s != std::future_status::ready) futThread.wait();
        }
    }

    // ════════════════════════════════════════════
    //  Key Interface
    // ════════════════════════════════════════════
    unsigned char TestClient::keyDetection()
    {
        termios old, now;
        tcgetattr(0, &old); now = old;
        now.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(0, TCSANOW, &now);
        unsigned char ch = getchar();
        tcsetattr(0, TCSANOW, &old);
        return ch;
    }

    unsigned char TestClient::keyExecute()
    {
        while (true) {
            unsigned char ch = keyDetection();

            switch (ch) {
            case 'q': startMappingPlFun(); break;
            case 'w': endMappingPlFun(); break;

            case 'a': {
                // List maps and select
                auto maps = listMaps();
                printf(CYN "\n========== 地圖列表 ==========\n" RST);
                for (size_t i = 0; i < maps.size(); i++)
                    printf("  [%zu] %s\n", i, maps[i].c_str());
                printf("==============================\n");
                printf("選擇地圖: "); fflush(stdout);

                termios old, now;
                tcgetattr(0, &old); now = old;
                now.c_lflag |= ICANON; now.c_lflag |= ECHO;
                tcsetattr(0, TCSANOW, &now);

                std::string input; std::getline(std::cin, input);
                tcsetattr(0, TCSANOW, &old);

                int idx = std::atoi(input.c_str());
                if (idx >= 0 && idx < (int)maps.size()) {
                    currentMapName = maps[idx];
                    loadTasks();
                    relocationPlFun();
                }
                break;
            }

            case 's': {
                // Record pose with name
                printf(YEL "\n輸入點位名稱: " RST); fflush(stdout);
                termios old, now;
                tcgetattr(0, &old); now = old;
                now.c_lflag |= ICANON; now.c_lflag |= ECHO;
                tcsetattr(0, TCSANOW, &now);

                std::string name; std::getline(std::cin, name);
                tcsetattr(0, TCSANOW, &old);

                poseDate p = curPose;
                p.tts_text = name;
                p.mode = 0;
                p.speed = 0.6f;
                poseList.push_back(p);
                printf(GRN "✅ 記錄點 [%zu]: " RST, poseList.size()-1);
                p.printInfo();
                saveTasks();
                break;
            }

            case 'l': {
                printf(BLD "\n📋 任務列表 (%zu 點)\n" RST, poseList.size());
                if (poseList.empty()) printf(YEL "  (空)\n\n" RST);
                else for (size_t i = 0; i < poseList.size(); i++) {
                    printf("  [%zu] ", i);
                    poseList[i].printInfo();
                }
                printf("\n"); break;
            }

            case 'c': taskThreadRunSingle(); break;
            case 'd': taskThreadRun(true); break;
            case 'f': poseList.clear(); printf(YEL "🗑 已清空\n" RST); saveTasks(); break;
            case 'r':
                if (!poseList.empty()) {
                    printf(YEL "🗑 移除末點 \"%s\"\n" RST, poseList.back().tts_text.c_str());
                    poseList.pop_back(); saveTasks();
                } break;
            case 'z': pauseNavFun(); break;
            case 'x': resumeNavFun(); break;

            default:
                printf(RED "\n🛑 停止\n" RST);
                taskThreadStop(); stopNodeFun();
                return ch;
            }
        }
    }
}

int main(int argc, const char **argv)
{
    if (argc < 2) { printf("Usage: %s <interface>\n", argv[0]); return -1; }
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    unitree::robot::slam::TestClient tc;
    tc.Init();
    tc.SetTimeout(10.0f);
    return tc.keyExecute();
}
