#include <unitree/robot/client/client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/ros2/String_.hpp>
#include <json.hpp>
#include <termio.h>
#include <string>
#include <future>
#include <thread>
#include <cstdlib>  // for std::system
#include <fstream>
#include <sstream>
#include <dirent.h>
#include <algorithm>
#include <atomic>
#include <cmath>    // for std::sqrt, std::atan2, M_PI

#include <sys/stat.h>   // for chmod

#define SlamInfoTopic "rt/slam_info"
#define SlamKeyInfoTopic "rt/slam_key_info"
#define AUDIO_DEVICE "plughw:0,0"  // 音頻設備，如變化請修改此處

// ========== 階躍星辰 TTS 配置 ==========
#define STEPFUN_API_KEY "YOUR_STEPFUN_API_KEY"
#define STEPFUN_TTS_URL "https://api.stepfun.com/v1/audio/speech"
#define STEPFUN_TTS_MODEL "step-tts-2"
#define STEPFUN_TTS_VOICE "elegantgentle-female"

#define ALIYUN_TTS_API_KEY  "YOUR_ALIYUN_TTS_API_KEY"   // 替换为你的Key
#define ALIYUN_TTS_VOICE    "Cherry"                  // 音色：Cherry/Serena/Ethan

#define LOCAL_TTS_URL "http://127.0.0.1:18789/tts" 


using namespace unitree::robot;
using namespace unitree::common;
unsigned char currentKey;

// 定義導航狀態枚舉
enum class NavStatus {
    NAVIGATING, // 導航進行中
    ARRIVED,    // 成功到達目標點（位置與方向皆符合）
    FAILED      // 導航失敗或超時
};

struct CruiseLogEntry {
    std::string timestamp;      // 2026-07-21 14:30:25.123
    std::string taskType;       // "single" / "loop"
    std::string mapName;
    std::string taskId;         // 任务唯一ID
    int pointIndex;
    float targetX, targetY, targetZ;
    float targetQW, targetQZ;
    std::string status;         // "arrived" / "failed" / "timeout" / "interrupted"
    float durationSec;          // 耗时秒数
    float finalDistError;       // 最终距离误差米
    float finalAngleError;      // 最终角度误差度
    std::string ttsText;
    std::string action;
    std::string errorMsg;
};

class poseDate
{
public:
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float q_x = 0.0f;
    float q_y = 0.0f;
    float q_z = 0.0f;
    float q_w = 1.0f;
    int mode = 0;
    float speed = 0.6f;
    std::string tts_text = "";
    std::string action = "";      // 到达后执行动作: "grasp"=机械臂抓取
    int drink = 0;                  // 饮料类型: 0=水 1=可乐 2=茶 3=王老吉

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

    nlohmann::json toFullJson() const
    {
        nlohmann::json j;
        j["x"] = x; j["y"] = y; j["z"] = z;
        j["q_x"] = q_x; j["q_y"] = q_y; j["q_z"] = q_z; j["q_w"] = q_w;
        j["mode"] = mode; j["speed"] = speed;
        j["tts_text"] = tts_text;
        j["action"] = action;
        j["drink"] = drink;
        return j;
    }

    void printInfo() const
    {
        std::cout << "x:" << x << " y:" << y << " z:" << z 
                  << " | q_w:" << q_w << " q_z:" << q_z
                  << " | TTS:[" << (tts_text.empty() ? "无" : tts_text) << "]" << " | 动作:[" << (action.empty() ? "无" : action) << "]" << std::endl;
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
        ChannelSubscriberPtr<std_msgs::msg::dds_::String_> subSlamInfo;
        ChannelSubscriberPtr<std_msgs::msg::dds_::String_> subSlamKeyInfo;

        void slamInfoHandler(const void *message);
        void slamKeyInfoHandler(const void *message);

        poseDate curPose;
        std::vector<poseDate> poseList;
        
        // 執行緒安全的原子狀態機
        std::atomic<NavStatus> m_navStatus{NavStatus::NAVIGATING};
        
        bool threadControl = false;
        std::future<void> futThread;
        std::promise<void> prom;
        std::thread controlThread;

        /* ========== 角度計算輔助 ========== */
        float calculateYaw(float qx, float qy, float qz, float qw);
        float angleDistance(float alpha, float beta);

        /* ========== 任務點管理 ========== */
        bool isNavigating() const;
        void listTaskFun();
        void deleteTaskFun();
        void insertTaskFun();
        void editTaskFun();
        void saveTasks();
        void loadTasks();

        /* ========== 地圖管理 ========== */
        std::vector<std::string> listPcdMaps();
        std::string currentMapName;
        std::string getTaskPath() const;
        void pcdToGridMap(const std::string& pcdPath, const std::string& mapName);

        /* ========== TTS 輔助 ========== */
        std::string escapeForJson(const std::string& text) const;
        void playTTS(const std::string& text, int index) const; // 確保這行在這裡
        /* ========== 导航日志 ========== */
        std::vector<CruiseLogEntry> cruiseLogs;
        std::string getCurrentTimestamp() const;
        void appendLog(const CruiseLogEntry& entry);
        void saveLogsToFile();
        void printLogSummary();
        void exportCsvLog();        // 导出CSV方便Excel分析
        
    
    

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
        void taskLoopFun(std::promise<void> &prom);
        void pauseNavFun();
        void resumeNavFun();
        void taskThreadRun();
        void taskThreadStop();

        void taskLoopFunSingle(std::promise<void> &prom);
        void taskThreadRunSingle();
        void showLogs();
    };

    TestClient::TestClient() : Client(TEST_SERVICE_NAME, false)
    {
        subSlamInfo = ChannelSubscriberPtr<std_msgs::msg::dds_::String_>(new ChannelSubscriber<std_msgs::msg::dds_::String_>(SlamInfoTopic));
        subSlamInfo->InitChannel(std::bind(&unitree::robot::slam::TestClient::slamInfoHandler, this, std::placeholders::_1), 1);
        subSlamKeyInfo = ChannelSubscriberPtr<std_msgs::msg::dds_::String_>(new ChannelSubscriber<std_msgs::msg::dds_::String_>(SlamKeyInfoTopic));
        subSlamKeyInfo->InitChannel(std::bind(&unitree::robot::slam::TestClient::slamKeyInfoHandler, this, std::placeholders::_1), 1);
        
        std::cout << "***********************  Unitree SLAM Demo ***********************\n";
        std::cout << "------------------------------------------------------------------\n";
        std::cout << "------------------ q: 开始建图                   -----------------\n";
        std::cout << "------------------ w: 结束建图并保存地图         -----------------\n";
        std::cout << "------------------ a: 加载地图并重定位           -----------------\n";
        std::cout << "------------------ s: 记录当前点位到任务列表     -----------------\n";
        std::cout << "------------------ d: 循环执行巡航任务           -----------------\n";
        std::cout << "------------------ c: 单次执行巡航任务           -----------------\n";
        std::cout << "------------------ f: 清空任务列表               -----------------\n";
        std::cout << "------------------ l: 查看任务列表               -----------------\n";
        std::cout << "------------------ r: 删除指定任务点             -----------------\n";
        std::cout << "------------------ i: 插入当前位姿到指定位置     -----------------\n";
        std::cout << "------------------ e: 编辑指定任务点             -----------------\n";
        std::cout << "------------------ z: 暂停导航                   -----------------\n";
        std::cout << "------------------ x: 恢复导航                   -----------------\n";
        std::cout << "------------------ v: 查看巡航日志                 -----------------\n";
        std::cout << "------------------ o: 导出CSV日志                  -----------------\n";
        std::cout << "------------------ 其他键: 确认后停止 SLAM       -----------------\n";
        std::cout << "------------------------------------------------------------------\n";
        std::cout << "--------------- 按 'Ctrl + C' 退出程序           -----------------\n";
        
        std::cout << "------------------------------------------------------------------\n" << std::endl;
        currentMapName = "";
        loadTasks();
    }

    TestClient::~TestClient()
    {
        saveTasks();
        stopNodeFun();
    }

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
}

/* ========== 角度計算輔助函式 ========== */
float unitree::robot::slam::TestClient::calculateYaw(float qx, float qy, float qz, float qw) {
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
}

float unitree::robot::slam::TestClient::angleDistance(float alpha, float beta) {
    float phi = std::abs(beta - alpha);
    while (phi > M_PI) {
        phi -= 2.0f * M_PI;
    }
    return std::abs(phi);
}

/* ========== 導航狀態檢查 ========== */
bool unitree::robot::slam::TestClient::isNavigating() const
{
    if (threadControl) return true;
    if (futThread.valid())
    {
        auto status = futThread.wait_for(std::chrono::milliseconds(0));
        return (status != std::future_status::ready);
    }
    return false;
}

std::string unitree::robot::slam::TestClient::getTaskPath() const
{
    if (currentMapName.empty()) {
        return "/home/unitree/0519/slam_task_points.json";
    }
    return "/home/unitree/lab/map/" + currentMapName + "_task.json";
}

void unitree::robot::slam::TestClient::saveTasks()
{
    nlohmann::json j;
    j["count"] = poseList.size();
    j["points"] = nlohmann::json::array();
    for (const auto& p : poseList) {
        j["points"].push_back(p.toFullJson());
    }
    std::string path = getTaskPath();
    std::ofstream ofs(path);
    if (ofs.is_open()) {
        ofs << j.dump(4);
        ofs.close();
        std::cout << "\033[32m[Saved] " << poseList.size() << " points -> " << path << "\033[0m" << std::endl;
    }
}

void unitree::robot::slam::TestClient::loadTasks()
{
    std::string path = getTaskPath();
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cout << "\033[33m[No task file for map: " << (currentMapName.empty() ? "default" : currentMapName) << "]\033[0m" << std::endl;
        return;
    }
    try {
        nlohmann::json j; ifs >> j;
        if (j.contains("points") && j["points"].is_array()) {
            poseList.clear();
            for (const auto& pj : j["points"]) {
                poseDate p;
                p.x = pj.value("x", 0.0f);
                p.y = pj.value("y", 0.0f);
                p.z = pj.value("z", 0.0f);
                p.q_x = pj.value("q_x", 0.0f);
                p.q_y = pj.value("q_y", 0.0f);
                p.q_z = pj.value("q_z", 0.0f);
                p.q_w = pj.value("q_w", 1.0f);
                p.mode = pj.value("mode", 0);
                p.speed = pj.value("speed", 0.4f);
                p.tts_text = pj.value("tts_text", "");
                p.action = pj.value("action", "");
                p.drink = pj.value("drink", 0);
                poseList.push_back(p);
            }
            std::cout << "\033[32m[Loaded] " << poseList.size() << " points from " << path << "\033[0m" << std::endl;
        }
    } catch (...) {
        std::cout << "\033[33m[Load failed] Start empty\033[0m" << std::endl;
    }
}

std::vector<std::string> unitree::robot::slam::TestClient::listPcdMaps()
{
    std::vector<std::string> maps;
    DIR* dir = opendir("/home/unitree/lab/map/");
    if (dir) {
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            std::string name = entry->d_name;
            if (name.size() > 4 && name.substr(name.size() - 4) == ".pcd") {
                maps.push_back(name);
            }
        }
        closedir(dir);
    }
    std::sort(maps.begin(), maps.end());
    return maps;
}

void unitree::robot::slam::TestClient::listTaskFun()
{
    std::cout << "\n========== 任务列表 (" << poseList.size() << " 个点) [地图: " << (currentMapName.empty() ? "无" : currentMapName) << "] ==========" << std::endl;
    if (poseList.empty()) {
        std::cout << "(空)" << std::endl;
    } else {
        for (size_t i = 0; i < poseList.size(); ++i) {
            std::cout << "[" << i << "] ";
            poseList[i].printInfo();
        }
    }
    std::cout << "========================================\n" << std::endl;
}

void unitree::robot::slam::TestClient::deleteTaskFun()
{
    if (poseList.empty()) { std::cout << "\033[31m任务列表为空，无法删除。\033[0m" << std::endl; return; }
    if (isNavigating()) { std::cout << "\033[31m导航运行中！请先停止再修改。\033[0m" << std::endl; return; }

    listTaskFun();
    std::cout << "输入要删除的序号 (0~" << poseList.size() - 1 << "): ";
    int idx;
    if (!(std::cin >> idx)) {
        std::cin.clear(); std::cin.ignore(10000, '\n');
        std::cout << "\033[31m输入无效！\033[0m" << std::endl; return;
    }
    std::cin.ignore(10000, '\n');
    if (idx < 0 || idx >= (int)poseList.size()) { std::cout << "\033[31m序号超出范围！\033[0m" << std::endl; return; }
    poseList.erase(poseList.begin() + idx);
    std::cout << "\033[32m已删除任务点 [" << idx << "]，剩余: " << poseList.size() << "\033[0m" << std::endl;
    saveTasks();
}

void unitree::robot::slam::TestClient::insertTaskFun()
{
    if (isNavigating()) { std::cout << "\033[31m导航运行中！请先停止再修改。\033[0m" << std::endl; return; }

    listTaskFun();
    std::cout << "输入要插入的位置 (0~" << poseList.size() << "): ";
    int idx;
    if (!(std::cin >> idx)) {
        std::cin.clear(); std::cin.ignore(10000, '\n');
        std::cout << "\033[31m输入无效！\033[0m" << std::endl; return;
    }
    std::cin.ignore(10000, '\n');
    if (idx < 0 || idx > (int)poseList.size()) { std::cout << "\033[31m序号超出范围！\033[0m" << std::endl; return; }

    std::cout << "请输入该任务点的 TTS 播报内容 (直接回车=不播报): ";
            std::cout << "请选择到达后动作 (直接回车=无, g=机械臂抓取, r=释放水瓶): ";
            std::string act_input; std::getline(std::cin, act_input);
    std::string tts_input;
    std::getline(std::cin, tts_input);

    poseDate newPose = curPose;
    newPose.tts_text = tts_input;
    if (act_input == "g") newPose.action = "grasp";
            if (act_input == "r") newPose.action = "release";
    poseList.insert(poseList.begin() + idx, newPose);
    std::cout << "\033[32m已在序号 [" << idx << "] 插入当前位姿\033[0m" << std::endl;
    newPose.printInfo();
    saveTasks();
}

void unitree::robot::slam::TestClient::editTaskFun()
{
    if (poseList.empty()) { std::cout << "\033[31m任务列表为空，无法编辑。\033[0m" << std::endl; return; }
    if (isNavigating()) { std::cout << "\033[31m导航运行中！请先停止再修改。\033[0m" << std::endl; return; }

    listTaskFun();
    std::cout << "输入要编辑的序号 (0~" << poseList.size() - 1 << "): ";
    int idx;
    if (!(std::cin >> idx)) {
        std::cin.clear(); std::cin.ignore(10000, '\n');
        std::cout << "\033[31m输入无效！\033[0m" << std::endl; return;
    }
    std::cin.ignore(10000, '\n');
    if (idx < 0 || idx >= (int)poseList.size()) { std::cout << "\033[31m序号超出范围！\033[0m" << std::endl; return; }

    std::cout << "当前值: "; poseList[idx].printInfo();
    std::cout << "是否替换为当前位姿？(y/n): ";
    char choice; std::cin >> choice; std::cin.ignore(10000, '\n');
    if (choice == 'y' || choice == 'Y') {
        poseList[idx] = curPose;
        std::cout << "\033[32m已更新任务点 [" << idx << "] 为当前位姿\033[0m" << std::endl;
    } else {
        std::cout << "输入 x y z q_x q_y q_z q_w mode speed: ";
        std::cin >> poseList[idx].x >> poseList[idx].y >> poseList[idx].z
                 >> poseList[idx].q_x >> poseList[idx].q_y >> poseList[idx].q_z
                 >> poseList[idx].q_w >> poseList[idx].mode >> poseList[idx].speed;
        std::cin.ignore(10000, '\n');
    }

    std::cout << "当前 TTS: [" << (poseList[idx].tts_text.empty() ? "无" : poseList[idx].tts_text) << "]\n是否修改？(y/n): ";
    char tts_choice; std::cin >> tts_choice; std::cin.ignore(10000, '\n');
    if (tts_choice == 'y' || tts_choice == 'Y') {
        std::cout << "输入新 TTS 内容: ";
        std::getline(std::cin, poseList[idx].tts_text);
    }
    saveTasks();
}

std::string unitree::robot::slam::TestClient::escapeForJson(const std::string& text) const
{
    std::string result = text;
    size_t pos = 0;
    while ((pos = result.find('\\', pos)) != std::string::npos) { result.replace(pos, 1, "\\\\"); pos += 2; }
    pos = 0;
    while ((pos = result.find('"', pos)) != std::string::npos) { result.replace(pos, 1, "\\\""); pos += 2; }
    pos = 0;
    while ((pos = result.find('\n', pos)) != std::string::npos) { result.replace(pos, 1, "\\n"); pos += 2; }
    return result;
}

void unitree::robot::slam::TestClient::playTTS(const std::string& text, int index) const
{
    if (text.empty()) return;

    // ========== 快取 key = DJB2 雜湊（與 Python tts_precache.py 一致） ==========
    unsigned long hash = 5381;
    for (char c : text) { hash = ((hash << 5) + hash) + (unsigned char)c; }
    std::string wavPath = "/tmp/tts_nav_" + std::to_string(hash) + ".wav";

    // 快取檢查
    {
        std::ifstream cacheCheck(wavPath, std::ios::binary | std::ios::ate);
        if (cacheCheck.is_open()) {
            long sz = static_cast<long>(cacheCheck.tellg());
            cacheCheck.close();
            if (sz >= 1000) {
                std::cout << "\033[1;36m[TTS] 播放快取: " << text << "\033[0m" << std::endl;
                std::string playCmd = "paplay " + wavPath + " >/dev/null 2>&1";
                std::system(playCmd.c_str());
                std::cout << "\033[32m[TTS] 播放完成\033[0m" << std::endl;
                return;
            }
        }
    }
    // ===========================================================

    std::cout << "\033[1;36m[TTS] 调用阿里云百炼TTS: " << text << "\033[0m" << std::endl;

    // ========== 内嵌 Python 桥接脚本（首次运行时落盘）==========
    static bool scriptReady = false;
    const std::string pyScript = "/tmp/aliyun_tts_bridge.py";

    if (!scriptReady) {
        std::ofstream pyf(pyScript);
        pyf << R"PY(#!/usr/bin/env python3
import websocket, json, base64, wave, sys, os, time

def synthesize(text, out_wav, api_key, voice):
    # 使用國際站 endpoint
    url = "wss://dashscope-intl.aliyuncs.com/api-ws/v1/realtime?model=qwen3-tts-flash-realtime"
    try:
        ws = websocket.create_connection(url, header=[f"Authorization: Bearer {api_key}"], timeout=15)
    except Exception as e:
        print(f"[TTS] WS连接失败: {e}", file=sys.stderr)
        return False

    chunks = []
    # 1. session.update
    ws.send(json.dumps({
        "event_id": f"evt_{os.urandom(4).hex()}",
        "type": "session.update",
        "session": {"voice": voice, "response_format": "pcm",
                    "sample_rate": 24000, "language_type": "Chinese"}
    }))
    # 2. 输入文本（兼容 dashscope-intl 協議）
    ws.send(json.dumps({
        "event_id": f"evt_{os.urandom(4).hex()}",
        "type": "input_text_buffer.append",
        "text": text
    }))
    ws.send(json.dumps({
        "event_id": f"evt_{os.urandom(4).hex()}",
        "type": "input_text_buffer.commit"
    }))

    done = False
    while not done:
        try:
            msg = ws.recv()
            payload = json.loads(msg)
            et = payload.get("type", "")
            if et in ("response.audio.delta", "output_audio.delta"):
                d = payload.get("delta") or payload.get("audio")
                if d: chunks.append(base64.b64decode(d))
            elif et in ("response.completed", "response.done", "session.finished"):
                done = True
            elif et == "error":
                print(f"[TTS] Server Error: {payload}", file=sys.stderr)
                done = True
        except Exception as e:
            print(f"[TTS] 接收异常: {e}", file=sys.stderr)
            break

    ws.close()
    pcm = b"".join(chunks)
    if not pcm:
        print("[TTS] 无音频数据", file=sys.stderr)
        return False

    with wave.open(out_wav, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(24000)
        w.writeframes(pcm)
    return True

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: tts_bridge.py <text> <out_wav> <api_key> [voice]")
        sys.exit(1)
    ok = synthesize(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4] if len(sys.argv)>4 else "Cherry")
    sys.exit(0 if ok else 1)
)PY";
        pyf.close();
        ::chmod(pyScript.c_str(), 0755);  // 加执行权限
        scriptReady = true;
    }

    // ========== 调用 Python 脚本 ==========
    // 文本中的双引号需要转义传给 shell
    std::string safeText = text;
    size_t pos = 0;
    while ((pos = safeText.find('"', pos)) != std::string::npos) {
        safeText.replace(pos, 1, "\\\"");
        pos += 2;
    }

    std::string cmd = "python3 " + pyScript + " \"" + safeText + "\" " + wavPath + " " + ALIYUN_TTS_API_KEY + " " + ALIYUN_TTS_VOICE + " 2>/dev/null";
    int ret = std::system(cmd.c_str());

    // ========== 验证并播放 ==========
    std::ifstream check(wavPath, std::ios::binary | std::ios::ate);
    if (!check.is_open()) {
        std::cout << "\033[31m[TTS] 合成失败，无输出文件\033[0m" << std::endl;
        return;
    }
    long fsize = static_cast<long>(check.tellg());
    check.close();

    if (fsize < 1000) {
        std::cout << "\033[31m[TTS] 合成失败，文件过小(" << fsize << " bytes)\033[0m" << std::endl;
        return;
    }

    // 使用 plughw 确保采样率转换，设备号根据你的声卡调整
    std::string playCmd = "paplay " + wavPath + " >/dev/null 2>&1";
    std::system(playCmd.c_str());

    std::cout << "\033[32m[TTS] 播放完成\033[0m" << std::endl;
}

void unitree::robot::slam::TestClient::taskThreadRun() {
    taskThreadStop();
    prom = std::promise<void>(); futThread = prom.get_future();
    controlThread = std::thread(&unitree::robot::slam::TestClient::taskLoopFun, this, std::ref(prom));
    controlThread.detach();
}

void unitree::robot::slam::TestClient::taskThreadRunSingle() {
    taskThreadStop();
    prom = std::promise<void>(); futThread = prom.get_future();
    controlThread = std::thread(&unitree::robot::slam::TestClient::taskLoopFunSingle, this, std::ref(prom));
    controlThread.detach();
}

// 获取当前时间字符串
std::string unitree::robot::slam::TestClient::getCurrentTimestamp() const
{
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    auto timer = std::chrono::system_clock::to_time_t(now);
    std::tm bt = *std::localtime(&timer);
    
    std::ostringstream oss;
    oss << std::put_time(&bt, "%Y-%m-%d %H:%M:%S");
    oss << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}

// 添加一条日志
void unitree::robot::slam::TestClient::appendLog(const CruiseLogEntry& entry)
{
    cruiseLogs.push_back(entry);
    // 实时追加写入文件（防崩溃丢失）
    saveLogsToFile();
}

// 保存到文件
void unitree::robot::slam::TestClient::saveLogsToFile()
{
    std::string logPath = "/home/unitree/lab/log/cruise_log.json";
    std::system("mkdir -p /home/unitree/lab/log/");
    
    nlohmann::json j;
    j["total_count"] = cruiseLogs.size();
    j["logs"] = nlohmann::json::array();
    
    for (const auto& e : cruiseLogs) {
        nlohmann::json lj;
        lj["timestamp"] = e.timestamp;
        lj["task_type"] = e.taskType;
        lj["map_name"] = e.mapName;
        lj["point_index"] = e.pointIndex;
        lj["target_pose"] = {{"x", e.targetX}, {"y", e.targetY}, {"z", e.targetZ}, 
                             {"q_w", e.targetQW}, {"q_z", e.targetQZ}};
        lj["status"] = e.status;
        lj["duration_sec"] = e.durationSec;
        lj["final_dist_error_m"] = e.finalDistError;
        lj["final_angle_error_deg"] = e.finalAngleError;
        lj["tts_text"] = e.ttsText;
        lj["action"] = e.action;
        lj["error_msg"] = e.errorMsg;
        j["logs"].push_back(lj);
    }
    
    std::ofstream ofs(logPath);
    if (ofs.is_open()) {
        ofs << j.dump(2);
        ofs.close();
    }
}

// 导出CSV
void unitree::robot::slam::TestClient::exportCsvLog()
{
    std::string csvPath = "/home/unitree/lab/log/cruise_log.csv";
    std::ofstream ofs(csvPath);
    if (!ofs.is_open()) return;
    
    // CSV表头
    ofs << "时间戳,任务类型,地图,点索引,目标X,目标Y,状态,耗时秒,距离误差米,角度误差度,TTS,动作,异常\n";
    
    for (const auto& e : cruiseLogs) {
        ofs << e.timestamp << ","
            << e.taskType << ","
            << e.mapName << ","
            << e.pointIndex << ","
            << e.targetX << ","
            << e.targetY << ","
            << e.status << ","
            << e.durationSec << ","
            << e.finalDistError << ","
            << e.finalAngleError << ","
            << "\"" << e.ttsText << "\","
            << e.action << ","
            << "\"" << e.errorMsg << "\"\n";
    }
    ofs.close();
    std::cout << "\033[32m[日志] CSV已导出: " << csvPath << "\033[0m" << std::endl;
}

// 查看日志摘要
void unitree::robot::slam::TestClient::showLogs()
{
    std::cout << "\n========== 巡航日志 (" << cruiseLogs.size() << " 条) ==========" << std::endl;
    if (cruiseLogs.empty()) {
        std::cout << "(空)" << std::endl;
        return;
    }
    
    int arrived = 0, failed = 0, timeout = 0, interrupted = 0;
    float totalTime = 0;
    for (const auto& e : cruiseLogs) {
        if (e.status == "arrived") arrived++;
        else if (e.status == "timeout") timeout++;
        else if (e.status == "interrupted") interrupted++;
        else failed++;
        totalTime += e.durationSec;
    }
    
    std::cout << "总任务数: " << cruiseLogs.size() << std::endl;
    std::cout << "  成功: " << arrived << " | 超时: " << timeout 
              << " | 中断: " << interrupted << " | 失败: " << failed << std::endl;
    std::cout << "总耗时: " << totalTime << " 秒 | 平均: " 
              << (cruiseLogs.empty() ? 0 : totalTime / cruiseLogs.size()) << " 秒/点" << std::endl;
    std::cout << "成功率: " << (cruiseLogs.empty() ? 0 : arrived * 100.0f / cruiseLogs.size()) << "%" << std::endl;
    
    // 显示当前任务的所有日志
    std::cout << "\n--- 当前任务详细日志 ---" << std::endl;
    std::cout << "序号 | 时间戳                    | 点 | 状态      | 耗时(秒) | 距离误差(米) | 角度误差(度) | TTS" << std::endl;
    std::cout << "--------------------------------------------------------------------------------------------" << std::endl;
    
    for (size_t i = 0; i < cruiseLogs.size(); i++) {
        const auto& e = cruiseLogs[i];
        std::string statusColor;
        if (e.status == "arrived") statusColor = "\033[32m";      // 绿色
        else if (e.status == "timeout") statusColor = "\033[33m"; // 黄色
        else statusColor = "\033[31m";                             // 红色
        
        std::cout << "[" << std::setw(2) << i << "] "
                  << e.timestamp << " | "
                  << "点" << std::setw(2) << e.pointIndex << " | "
                  << statusColor << std::setw(9) << e.status << "\033[0m | "
                  << std::fixed << std::setprecision(2) << std::setw(8) << e.durationSec << " | ";
        
        if (e.finalDistError >= 0) {
            std::cout << std::setw(12) << std::setprecision(4) << e.finalDistError << " | "
                      << std::setw(12) << std::setprecision(2) << e.finalAngleError << " | ";
        } else {
            std::cout << "      N/A    |       N/A    | ";
        }
        
        std::cout << (e.ttsText.empty() ? "-" : e.ttsText.substr(0, 20)) << std::endl;
    }
    
    std::cout << "========================================\n" << std::endl;
}

/* ========== 核心安全機制：單次巡航邏輯（完美破除同點死鎖） ========== */
void unitree::robot::slam::TestClient::taskLoopFunSingle(std::promise<void> &prom)
{
    std::string data;
    threadControl = true;

    // ========== 导航前机械臂归位 ==========
    std::cout << "\033[36m[Arm] 导航前机械臂归位 HOME (J1_OFFSET=90°)...\033[0m" << std::endl;
    int armRet = std::system("python3 /home/unitree/lab/all/vision+arm/arm_home.py");
    if (armRet != 0) {
        std::cout << "\033[33m[Arm] ⚠️ HOME 归位异常(返回码:" << armRet << ")，继续巡航\033[0m" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "\033[32m[Arm] 机械臂已归位 HOME\033[0m" << std::endl;

    std::cout << "\033[34m[巡航启动] 单次巡航开始，总任务点数量: " << poseList.size() << "\033[0m" << std::endl;

    for (int i = 0; i < (int)poseList.size(); i++)
    {
        m_navStatus = NavStatus::NAVIGATING;

        // 發送前前置計算：判定是否為純原地轉向點（距離小於 5 公分）
        float init_dx = curPose.x - poseList[i].x;
        float init_dy = curPose.y - poseList[i].y;
        float init_distance = std::sqrt(init_dx * init_dx + init_dy * init_dy);

        bool isPureRotation = (init_distance < 0.05f);

        // ========== TTS: 出发播报 ==========
        if (!poseList[i].tts_text.empty()) {
            playTTS("正在前往" , i * 10 + 1);
        }
        // ===================================

        if (isPureRotation) {
            std::cout << "\033[35m------------------------------------------\n"
                      << "[纯转向任务] 目标点 [" << i << "] 坐标相同但方向不同！进入高精度旋转监控...\n"
                      << "------------------------------------------\033[0m" << std::endl;
            Call(ROBOT_API_ID_POSE_NAV_PL, poseList[i].toJsonStr(), data);
        } else {
            int32_t statusCode = Call(ROBOT_API_ID_POSE_NAV_PL, poseList[i].toJsonStr(), data);
            std::cout << "------------------------------------------" << std::endl;
            std::cout << "发送导航目标点 [" << i << "] -> x: " << poseList[i].x << ", y: " << poseList[i].y << std::endl;
            if (statusCode != 0) {
                std::cout << "\033[31m[错误] 发送目标点失败，跳过点 [" << i << "]\033[0m" << std::endl;
                continue;
            }
        }

        auto navStartTime = std::chrono::steady_clock::now();  // ← 记录导航开始时间
        int printCounter = 0;
        
        // 多級减速控制: 0=全速, 1=0.15(1.2m), 2=0.08(0.5m), 3=0.03(0.3m)
        static const float SPEED_STAGE[] = {0.6f, 0.15f, 0.08f, 0.03f};
        static const float DIST_STAGE[] = {1.20f, 0.50f, 0.30f};
        int speedStage = 0;

        // 關鍵硬緩衝：發送指令後強行等待 100 毫秒，保證底層緩衝區及當前位姿數據徹底刷新
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        while (m_navStatus == NavStatus::NAVIGATING)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (!threadControl) break;

            // 即時位置與角度差計算
            float dx = curPose.x - poseList[i].x;
            float dy = curPose.y - poseList[i].y;
            float distance = std::sqrt(dx * dx + dy * dy);

            // ------------------ 【动态控制】多级渐进减速 ------------------
            if (!isPureRotation) {
                int newStage = 0;
                for (int s = 0; s < 3; s++) {
                    if (distance <= DIST_STAGE[s]) newStage = s + 1;
                }
                if (newStage > speedStage) {
                    speedStage = newStage;
                    poseDate slowPose = poseList[i];
                    slowPose.speed = SPEED_STAGE[newStage];
                    std::string slowData;
                    Call(ROBOT_API_ID_POSE_NAV_PL, slowPose.toJsonStr(), slowData);
                    std::cout << "\033[33m[动态控制] 距离 " << distance << " 米，速度降至 " << SPEED_STAGE[newStage] << " m/s (stage=" << newStage << ")\033[0m" << std::endl;
                    // ========== TTS: 减速播报 ==========
                    if (newStage == 1) { playTTS("接近目標", i * 10 + 3); }
                    else if (newStage == 2) { playTTS("即將到達", i * 10 + 4); }
                    else if (newStage == 3) { playTTS("馬上到了", i * 10 + 5); }
                    // ===================================
                }
            }
            // -------------------------------------------------------------------------

            float curYaw = calculateYaw(curPose.q_x, curPose.q_y, curPose.q_z, curPose.q_w);
            float targetYaw = calculateYaw(poseList[i].q_x, poseList[i].q_y, poseList[i].q_z, poseList[i].q_w);
            float angleDiffDeg = angleDistance(curYaw, targetYaw) * 180.0f / M_PI;

            if (++printCounter % 15 == 0) {
                std::cout << "[巡航中] 点 [" << i << "] -> 距离剩: " << distance
                          << " 米 | 角度偏差: " << angleDiffDeg << " 度"
                          << (isPureRotation ? " (纯转向模式)" : "") << std::endl;
            }

            // 【動態動態閾值】如果是純原地轉頭，強迫要求角度進入 5 度以內、距離 5 公分以內，徹底掐死秒跳Bug
            // 輪腿版放寬角度閾值
            float targetAngleThreshold = isPureRotation ? 5.0f : 20.0f;
            float targetDistThreshold = isPureRotation ? 0.05f : 0.12f;

            if (distance < targetDistThreshold && angleDiffDeg < targetAngleThreshold) {
                std::cout << "\033[32m[自主判定] 成功！点 [" << i 
                          << "] 位置与精确定向均已达标！\033[0m" << std::endl;
                m_navStatus = NavStatus::ARRIVED;
                break;
            }

            // 智能超時防卡死（輪腿版超時加長至 120s）
            auto currentTime = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(currentTime - navStartTime).count();
            uint32_t timeoutLimit = isPureRotation ? 30 : 40;
            if (elapsed > timeoutLimit) {
                std::cout << "\033[33m[自主判定] 导航超时，强制切换至下一点。\033[0m" << std::endl;
                playTTS("無法到達此點", i * 10 + 6);
                m_navStatus = NavStatus::FAILED;
                break;
            }
        }

        if (!threadControl) break;

        // ========== 导航结束，记录巡航日志 ==========
        auto navEndTime = std::chrono::steady_clock::now();
        float duration = std::chrono::duration<float>(navEndTime - navStartTime).count();

        CruiseLogEntry log;
        log.timestamp = getCurrentTimestamp();
        log.taskType = "single";
        log.mapName = currentMapName.empty() ? "default" : currentMapName;
        log.pointIndex = i;
        log.targetX = poseList[i].x;
        log.targetY = poseList[i].y;
        log.targetZ = poseList[i].z;
        log.targetQW = poseList[i].q_w;
        log.targetQZ = poseList[i].q_z;
        log.durationSec = duration;
        log.ttsText = poseList[i].tts_text;
        log.action = poseList[i].action;

        if (m_navStatus == NavStatus::ARRIVED) {
            log.status = "arrived";
            float dx = curPose.x - poseList[i].x;
            float dy = curPose.y - poseList[i].y;
            log.finalDistError = std::sqrt(dx * dx + dy * dy);
            float curYaw = calculateYaw(curPose.q_x, curPose.q_y, curPose.q_z, curPose.q_w);
            float targetYaw = calculateYaw(poseList[i].q_x, poseList[i].q_y, poseList[i].q_z, poseList[i].q_w);
            log.finalAngleError = angleDistance(curYaw, targetYaw) * 180.0f / M_PI;
            log.errorMsg = "";
        } else if (m_navStatus == NavStatus::FAILED) {
            log.status = "timeout";
            log.finalDistError = -1.0f;
            log.finalAngleError = -1.0f;
            log.errorMsg = "导航超时";
        } else {
            log.status = "interrupted";
            log.finalDistError = -1.0f;
            log.finalAngleError = -1.0f;
            log.errorMsg = "用户中断或异常终止";
        }

        appendLog(log);
        // ============================================

        // ========== 到達後角度校正（原地旋轉至目標朝向） ==========
        if (m_navStatus == NavStatus::ARRIVED) {
            float curYaw = calculateYaw(curPose.q_x, curPose.q_y, curPose.q_z, curPose.q_w);
            float targetYaw = calculateYaw(poseList[i].q_x, poseList[i].q_y, poseList[i].q_z, poseList[i].q_w);
            float angleAfterArrival = angleDistance(curYaw, targetYaw) * 180.0f / M_PI;

            if (angleAfterArrival > 5.0f) {
                std::cout << "\033[36m[校正] 到达后角度偏差 " << angleAfterArrival
                          << "°，启动原地旋转校正...\033[0m" << std::endl;

                std::string rotData;
                Call(ROBOT_API_ID_POSE_NAV_PL, poseList[i].toJsonStr(), rotData);

                int rotPrintCnt = 0;
                auto rotStart = std::chrono::steady_clock::now();
                while (threadControl) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));

                    float curYawNow = calculateYaw(curPose.q_x, curPose.q_y, curPose.q_z, curPose.q_w);
                    float targetYawNow = calculateYaw(poseList[i].q_x, poseList[i].q_y, poseList[i].q_z, poseList[i].q_w);
                    float rotDiff = angleDistance(curYawNow, targetYawNow) * 180.0f / M_PI;

                    if (++rotPrintCnt % 15 == 0) {
                        std::cout << "[校正] 旋转中... 当前偏差 " << rotDiff << "°" << std::endl;
                    }

                    if (rotDiff < 15.0f) {
                        std::cout << "\033[32m[校正] ✅ 角度校正成功！偏差 " << rotDiff << "°\033[0m" << std::endl;
                        break;
                    }

                    auto rotElapsed = std::chrono::duration_cast<std::chrono::seconds>(
                        std::chrono::steady_clock::now() - rotStart).count();
                    if (rotElapsed > 30) {
                        std::cout << "\033[33m[校正] ⚠️ 角度校正超时（当前 " << rotDiff << "°）\033[0m" << std::endl;
                        break;
                    }
                }

                float finalYaw = calculateYaw(curPose.q_x, curPose.q_y, curPose.q_z, curPose.q_w);
                float newAngleErr = angleDistance(finalYaw, targetYaw) * 180.0f / M_PI;
                log.finalAngleError = newAngleErr;

                float newDx = curPose.x - poseList[i].x;
                float newDy = curPose.y - poseList[i].y;
                log.finalDistError = std::sqrt(newDx * newDx + newDy * newDy);

                std::cout << "\033[36m[校正] 最终精度: 距离 " << log.finalDistError
                          << " 米, 角度 " << newAngleErr << "°\033[0m" << std::endl;
            }
        }
        // ============================================================

        if (m_navStatus == NavStatus::ARRIVED) {
            // ========== TTS: 到达播报 ==========
            if (!poseList[i].tts_text.empty()) {
                playTTS("已到達" + poseList[i].tts_text, i * 10 + 2);
            }
            // ===================================

            // 执行航点动作
            if (poseList[i].action == "grasp") {
                std::cout << "\033[1;35m[动作] 到达抓取点，启动机械臂抓取...\033[0m" << std::endl;
                playTTS("開始抓取", 100);
                std::stringstream graspCmd; graspCmd << "cd /home/unitree/lab/all/vision+arm && python3 3dcatchv5.py --drink=" << poseList[i].drink << " --delay=1";
                int ret = std::system(graspCmd.str().c_str());
                std::cout << "\033[1;35m[动作] 抓取完成 (返回码:" << ret << ")\033[0m" << std::endl;
                // ========== TTS: 抓取结果 ==========
                if (ret == 0) { playTTS("抓取完成", 101); }
                else { playTTS("抓取失敗", 102); }
                // ===================================
            }
            if (poseList[i].action == "release") {
                std::cout << "\033[1;35m[动作] 到达释放点，启动释放...\033[0m" << std::endl;
                playTTS("請伸出你的手,讓我看到你", 110);
                playTTS("說放手我就會把水給你", 111);
                int ret = std::system("cd /home/unitree/lab/all/vision+arm && python3 handover/handover_listener.py");
                playTTS("這是你的水", 112);
                std::cout << "\033[1;35m[动作] 释放完成 (返回码:" << ret << ")\033[0m" << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::seconds(1));
        } else if (m_navStatus == NavStatus::FAILED) {
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }

    std::cout << "\033[32m单次巡航任务结束。\033[0m" << std::endl;
    prom.set_value();
}

/* ========== 核心安全機制：循環巡航邏輯（完美破除同點死鎖） ========== */
void unitree::robot::slam::TestClient::taskLoopFun(std::promise<void> &prom)
{
    std::string data;
    threadControl = true;

    // ========== 导航前机械臂归位 ==========
    std::cout << "\033[36m[Arm] 导航前机械臂归位 HOME (J1_OFFSET=90°)...\033[0m" << std::endl;
    int armRet = std::system("python3 /home/unitree/lab/all/vision+arm/arm_home.py");
    if (armRet != 0) {
        std::cout << "\033[33m[Arm] ⚠️ HOME 归位异常(返回码:" << armRet << ")，继续巡航\033[0m" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "\033[32m[Arm] 机械臂已归位 HOME\033[0m" << std::endl;

    std::cout << "\033[34m[巡航启动] 循环巡航开始，总任务点数量: " << poseList.size() << "\033[0m" << std::endl;

    for (int i = 0; i < (int)poseList.size(); i++)
    {
        m_navStatus = NavStatus::NAVIGATING;

        float init_dx = curPose.x - poseList[i].x;
        float init_dy = curPose.y - poseList[i].y;
        float init_distance = std::sqrt(init_dx * init_dx + init_dy * init_dy);

        bool isPureRotation = (init_distance < 0.05f);

        // ========== TTS: 出发播报 ==========
        if (!poseList[i].tts_text.empty()) {
            playTTS("正在前往" + poseList[i].tts_text, i * 10 + 1);
        }
        // ===================================

        if (isPureRotation) {
            std::cout << "\033[35m------------------------------------------\n"
                      << "[纯转向任务] 目标点 [" << i << "] 坐标相同但方向不同！进入高精度旋转监控...\n"
                      << "------------------------------------------\033[0m" << std::endl;
            Call(ROBOT_API_ID_POSE_NAV_PL, poseList[i].toJsonStr(), data);
        } else {
            int32_t statusCode = Call(ROBOT_API_ID_POSE_NAV_PL, poseList[i].toJsonStr(), data);
            std::cout << "------------------------------------------" << std::endl;
            std::cout << "发送导航目标点 [" << i << "] -> x: " << poseList[i].x << ", y: " << poseList[i].y << std::endl;
            if (statusCode != 0) {
                std::cout << "\033[31m[错误] 发送目标点失败，跳过点 [" << i << "]\033[0m" << std::endl;
                continue;
            }
        }

        auto navStartTime = std::chrono::steady_clock::now();  // ← 记录导航开始时间
        int printCounterLoop = 0;

        // 多級减速控制: 0=全速, 1=0.15(1.2m), 2=0.08(0.5m), 3=0.03(0.3m)
        static const float SPEED_STAGE[] = {0.6f, 0.15f, 0.08f, 0.03f};
        static const float DIST_STAGE[] = {1.20f, 0.50f, 0.30f};
        int speedStage = 0;

        // 關鍵硬緩衝
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        while (m_navStatus == NavStatus::NAVIGATING)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (!threadControl) break;

            float dx = curPose.x - poseList[i].x;
            float dy = curPose.y - poseList[i].y;
            float distance = std::sqrt(dx * dx + dy * dy);

            // ------------------ 【动态控制】多级渐进减速 ------------------
            if (!isPureRotation) {
                int newStage = 0;
                for (int s = 0; s < 3; s++) {
                    if (distance <= DIST_STAGE[s]) newStage = s + 1;
                }
                if (newStage > speedStage) {
                    speedStage = newStage;
                    poseDate slowPose = poseList[i];
                    slowPose.speed = SPEED_STAGE[newStage];
                    std::string slowData;
                    Call(ROBOT_API_ID_POSE_NAV_PL, slowPose.toJsonStr(), slowData);
                std::cout << "\033[33m[动态控制] 距离 " << distance << " 米，速度降至 " << SPEED_STAGE[newStage] << " m/s (stage=" << newStage << ")\033[0m" << std::endl;
                    // ========== TTS: 减速播报 ==========
                    if (newStage == 1) { playTTS("接近目標", i * 10 + 3); }
                    else if (newStage == 2) { playTTS("即將到達", i * 10 + 4); }
                    else if (newStage == 3) { playTTS("馬上到了", i * 10 + 5); }
                    // ===================================
                }
            }
            // -------------------------------------------------------------------------

            float curYaw = calculateYaw(curPose.q_x, curPose.q_y, curPose.q_z, curPose.q_w);
            float targetYaw = calculateYaw(poseList[i].q_x, poseList[i].q_y, poseList[i].q_z, poseList[i].q_w);
            float angleDiffDeg = angleDistance(curYaw, targetYaw) * 180.0f / M_PI;

            if (++printCounterLoop % 15 == 0) {
                std::cout << "[巡航中] 点 [" << i << "] -> 距离剩: " << distance 
                          << " 米 | 角度偏差: " << angleDiffDeg << " 度" 
                          << (isPureRotation ? " (纯转向模式)" : "") << std::endl;
            }

            // 輪腿版放寬角度閾值
            float targetAngleThreshold = isPureRotation ? 5.0f : 20.0f;
            float targetDistThreshold = isPureRotation ? 0.05f : 0.12f;

            if (distance < targetDistThreshold && angleDiffDeg < targetAngleThreshold) {
                std::cout << "\033[32m[自主判定] 成功！点 [" << i << "] 位置与精确定向均已达标！\033[0m" << std::endl;
                m_navStatus = NavStatus::ARRIVED;
                break;
            }

            // 智能超時防卡死（輪腿版超時加長至 120s）
            auto currentTime = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(currentTime - navStartTime).count();
            uint32_t timeoutLimit = isPureRotation ? 30 : 120;
            if (elapsed > timeoutLimit) {
                std::cout << "\033[33m[自主判定] 导航超时，强制切换至下一点。\033[0m" << std::endl;
                playTTS("無法到達此點", i * 10 + 6);
                m_navStatus = NavStatus::FAILED;
                break;
            }
        }

        if (!threadControl) break;

        // ========== 导航结束，记录巡航日志 ==========
        auto navEndTime = std::chrono::steady_clock::now();
        float duration = std::chrono::duration<float>(navEndTime - navStartTime).count();

        CruiseLogEntry log;
        log.timestamp = getCurrentTimestamp();
        log.taskType = "loop";
        log.mapName = currentMapName.empty() ? "default" : currentMapName;
        log.pointIndex = i;
        log.targetX = poseList[i].x;
        log.targetY = poseList[i].y;
        log.targetZ = poseList[i].z;
        log.targetQW = poseList[i].q_w;
        log.targetQZ = poseList[i].q_z;
        log.durationSec = duration;
        log.ttsText = poseList[i].tts_text;
        log.action = poseList[i].action;

        if (m_navStatus == NavStatus::ARRIVED) {
            log.status = "arrived";
            float dx = curPose.x - poseList[i].x;
            float dy = curPose.y - poseList[i].y;
            log.finalDistError = std::sqrt(dx * dx + dy * dy);
            float curYaw = calculateYaw(curPose.q_x, curPose.q_y, curPose.q_z, curPose.q_w);
            float targetYaw = calculateYaw(poseList[i].q_x, poseList[i].q_y, poseList[i].q_z, poseList[i].q_w);
            log.finalAngleError = angleDistance(curYaw, targetYaw) * 180.0f / M_PI;
            log.errorMsg = "";
        } else if (m_navStatus == NavStatus::FAILED) {
            log.status = "timeout";
            log.finalDistError = -1.0f;
            log.finalAngleError = -1.0f;
            log.errorMsg = "导航超时";
        } else {
            log.status = "interrupted";
            log.finalDistError = -1.0f;
            log.finalAngleError = -1.0f;
            log.errorMsg = "用户中断或异常终止";
        }

        appendLog(log);
        // ============================================

        // ========== 到達後角度校正（原地旋轉至目標朝向） ==========
        if (m_navStatus == NavStatus::ARRIVED) {
            float curYaw = calculateYaw(curPose.q_x, curPose.q_y, curPose.q_z, curPose.q_w);
            float targetYaw = calculateYaw(poseList[i].q_x, poseList[i].q_y, poseList[i].q_z, poseList[i].q_w);
            float angleAfterArrival = angleDistance(curYaw, targetYaw) * 180.0f / M_PI;

            if (angleAfterArrival > 5.0f) {
                std::cout << "\033[36m[校正] 到达后角度偏差 " << angleAfterArrival
                          << "°，启动原地旋转校正...\033[0m" << std::endl;

                // 發送純旋轉指令（相同座標 + 正確朝向）
                std::string rotData;
                Call(ROBOT_API_ID_POSE_NAV_PL, poseList[i].toJsonStr(), rotData);

                // 旋轉監控循環：5° 閾值，30s 超時
                auto rotStart = std::chrono::steady_clock::now();
                int rotPrintCnt = 0;
                while (threadControl) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));

                    float curYawNow = calculateYaw(curPose.q_x, curPose.q_y, curPose.q_z, curPose.q_w);
                    float targetYawNow = calculateYaw(poseList[i].q_x, poseList[i].q_y, poseList[i].q_z, poseList[i].q_w);
                    float rotDiff = angleDistance(curYawNow, targetYawNow) * 180.0f / M_PI;

                    if (++rotPrintCnt % 15 == 0) {
                        std::cout << "[校正] 旋转中... 当前偏差 " << rotDiff << "°" << std::endl;
                    }

                    if (rotDiff < 5.0f) {
                        std::cout << "\033[32m[校正] ✅ 角度校正成功！偏差 " << rotDiff << "°\033[0m" << std::endl;
                        break;
                    }

                    auto rotElapsed = std::chrono::duration_cast<std::chrono::seconds>(
                        std::chrono::steady_clock::now() - rotStart).count();
                    if (rotElapsed > 30) {
                        std::cout << "\033[33m[校正] ⚠️ 角度校正超时（当前 " << rotDiff << "°）\033[0m" << std::endl;
                        break;
                    }
                }

                // 更新巡航日誌中的最終角度誤差
                float finalYaw = calculateYaw(curPose.q_x, curPose.q_y, curPose.q_z, curPose.q_w);
                float newAngleErr = angleDistance(finalYaw, targetYaw) * 180.0f / M_PI;
                log.finalAngleError = newAngleErr;

                float newDx = curPose.x - poseList[i].x;
                float newDy = curPose.y - poseList[i].y;
                log.finalDistError = std::sqrt(newDx * newDx + newDy * newDy);

                std::cout << "\033[36m[校正] 最终精度: 距离 " << log.finalDistError
                          << " 米, 角度 " << newAngleErr << "°\033[0m" << std::endl;
            }
        }
        // ============================================================

        if (m_navStatus == NavStatus::ARRIVED) {
            // ========== TTS: 到达播报 ==========
            if (!poseList[i].tts_text.empty()) {
                playTTS("已到達" + poseList[i].tts_text, i * 10 + 2);
            }
            // ===================================

            // 执行航点动作
            if (poseList[i].action == "grasp") {
                std::cout << "\033[1;35m[动作] 到达抓取点，启动机械臂抓取...\033[0m" << std::endl;
                playTTS("開始抓取", 100);
                std::stringstream graspCmd; graspCmd << "cd /home/unitree/lab/all/vision+arm && python3 3dcatchv5.py --drink=" << poseList[i].drink << " --delay=1";
                int ret = std::system(graspCmd.str().c_str());
                std::cout << "\033[1;35m[动作] 抓取完成 (返回码:" << ret << ")\033[0m" << std::endl;
                // ========== TTS: 抓取结果 ==========
                if (ret == 0) { playTTS("抓取完成", 101); }
                else { playTTS("抓取失敗", 102); }
                // ===================================
            }
            if (poseList[i].action == "release") {
                std::cout << "\033[1;35m[动作] 到达释放点，启动释放...\033[0m" << std::endl;
                playTTS("請伸出你的手,讓我看到你", 110);
                playTTS("說放手我就會把水給你", 111);
                int ret = std::system("cd /home/unitree/lab/all/vision+arm && python3 handover/handover_listener.py");
                playTTS("這是你的水", 112);
                std::cout << "\033[1;35m[动作] 释放完成 (返回码:" << ret << ")\033[0m" << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::seconds(1));
        } else if (m_navStatus == NavStatus::FAILED) {
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }

        // 路径末端反转判定（增加了 poseList.size() > 1 的保护，防止单点陷入死循环反转）
        if (poseList.size() > 1 && i == (int)poseList.size() - 1) {
            i = -1;
            std::reverse(poseList.begin(), poseList.end());
            std::cout << "\033[36m[循环巡航] 已到达路径末端，反转路径继续前进！\033[0m" << std::endl;
        }
    }
    prom.set_value();
}


void unitree::robot::slam::TestClient::taskThreadStop()
{
    threadControl = false;
    if (futThread.valid()) {
        auto status = futThread.wait_for(std::chrono::milliseconds(0));
        if (status != std::future_status::ready) futThread.wait();
    }
}

void unitree::robot::slam::TestClient::slamInfoHandler(const void *message)
{
    std_msgs::msg::dds_::String_ currentMsg = *(std_msgs::msg::dds_::String_ *)message;
    nlohmann::json jsonData = nlohmann::json::parse(currentMsg.data());
    if (jsonData["errorCode"] != 0) return;

    if (jsonData["type"] == "pos_info") {
        curPose.x = jsonData["data"]["currentPose"]["x"];
        curPose.y = jsonData["data"]["currentPose"]["y"];
        curPose.z = jsonData["data"]["currentPose"]["z"];
        curPose.q_x = jsonData["data"]["currentPose"]["q_x"];
        curPose.q_y = jsonData["data"]["currentPose"]["q_y"];
        curPose.q_z = jsonData["data"]["currentPose"]["q_z"];
        curPose.q_w = jsonData["data"]["currentPose"]["q_w"];
    }
}

void unitree::robot::slam::TestClient::slamKeyInfoHandler(const void *message) {}

void unitree::robot::slam::TestClient::stopNodeFun()
{
    std::string parameter = R"({"data": {}})", data;
    Call(ROBOT_API_ID_STOP_NODE, parameter, data);
}

void unitree::robot::slam::TestClient::startMappingPlFun()
{
    std::string parameter = R"({"data": {"slam_type": "indoor"}})", data;
    int32_t statusCode = Call(ROBOT_API_ID_START_MAPPING_PL, parameter, data);
    
    std::cout << "\033[1;35m[Start Mapping] statusCode: " << statusCode << "\033[0m" << std::endl;
    std::cout << "\033[1;35m[Start Mapping] data: " << data << "\033[0m" << std::endl;
}

void unitree::robot::slam::TestClient::endMappingPlFun()
{
    std::cout << "输入地图名称（不需要加 .pcd）: ";
    std::string mapName; std::cin >> mapName; std::cin.ignore(10000, '\n');
    if (mapName.size() < 4 || mapName.substr(mapName.size() - 4) != ".pcd") { mapName += ".pcd"; }

    std::system("mkdir -p /home/unitree/lab/map/");
    std::string fullPath = "/home/unitree/lab/map/" + mapName;

    nlohmann::json j; j["data"]["address"] = fullPath;
    std::string data; Call(ROBOT_API_ID_END_MAPPING_PL, j.dump(), data);

    currentMapName = mapName;
    if (currentMapName.size() > 4 && currentMapName.substr(currentMapName.size() - 4) == ".pcd") {
        currentMapName = currentMapName.substr(0, currentMapName.size() - 4);
    }

    nlohmann::json initJ;
    initJ["x"] = curPose.x; initJ["y"] = curPose.y; initJ["z"] = curPose.z;
    initJ["q_x"] = curPose.q_x; initJ["q_y"] = curPose.q_y; initJ["q_z"] = curPose.q_z; initJ["q_w"] = curPose.q_w;
    std::ofstream initOfs("/home/unitree/lab/map/" + currentMapName + "_init.json");
    if (initOfs.is_open()) { initOfs << initJ.dump(4); initOfs.close(); }
    saveTasks();

    // ---> 在這裡加入這行：自動將存下來的 PCD 轉為 2D PGM/YAML 地圖 <---
    pcdToGridMap(fullPath, currentMapName);
}

void unitree::robot::slam::TestClient::relocationPlFun()
{
    auto maps = listPcdMaps(); // 获取系统内实际带有 .pcd 的档案清单
    if (maps.empty()) { 
        std::cout << "\033[31m/home/unitree/lab/map/ 目录下没有找到地图文件\033[0m" << std::endl; 
        return;
    }

    // 1. 显示地图列表（去除 .pcd 后缀）
    std::cout << "\n========== 地图列表 ==========" << std::endl;
    for (size_t i = 0; i < maps.size(); ++i) { 
        std::string displayName = maps[i];
        if (displayName.size() > 4 && displayName.substr(displayName.size() - 4) == ".pcd") {
            displayName = displayName.substr(0, displayName.size() - 4);
        }
        std::cout << "[" << i << "] " << displayName << std::endl;
    }
    std::cout << "==============================" << std::endl;
    std::cout << "选择地图序号或直接输入地图名称: ";

    // 2. 读取使用者输入
    std::string input;
    if (!(std::cin >> input)) { std::cin.clear(); std::cin.ignore(10000, '\n'); return; }
    std::cin.ignore(10000, '\n');

    std::string selectedMap = "";
    // 判断输入是否为纯数字（序号）
    bool isNumber = !input.empty() && std::all_of(input.begin(), input.end(), ::isdigit);

    // 3. 根据输入进行匹配
    if (isNumber) {
        int idx = std::stoi(input);
        if (idx >= 0 && idx < (int)maps.size()) {
            selectedMap = maps[idx]; 
        } else {
            std::cout << "\033[31m序号超出范围！\033[0m" << std::endl;
            return;
        }
    } else {
        // 如果输入的是地图名称（不带后缀），则在内部加上 .pcd 进行实体档案比对
        std::string targetFile = input + ".pcd";
        auto it = std::find(maps.begin(), maps.end(), targetFile);
        if (it != maps.end()) {
            selectedMap = *it; 
        } else {
            std::cout << "\033[31m未找到名为 " << input << " 的地图文件！\033[0m" << std::endl;
            return;
        }
    }

    // 4. 后续重定位和加载任务点逻辑
    currentMapName = selectedMap;
    if (currentMapName.size() > 4 && currentMapName.substr(currentMapName.size() - 4) == ".pcd") {
        currentMapName = currentMapName.substr(0, currentMapName.size() - 4);
    }
    poseList.clear(); 
    loadTasks();

    std::string fullPath = "/home/unitree/lab/map/" + selectedMap;
    
        // ========== 使用当前位姿作为初始位姿（任意点重定位）==========
    float init_x = curPose.x;
    float init_y = curPose.y;
    float init_z = curPose.z;
    float init_qx = curPose.q_x;
    float init_qy = curPose.q_y;
    float init_qz = curPose.q_z;
    float init_qw = curPose.q_w;
    bool hasInitPose = true;

    std::cout << "\033[36m[Relocation] 使用当前位姿作为初始位姿:\033[0m" << std::endl;
    std::cout << "  x=" << init_x << " y=" << init_y << " z=" << init_z << std::endl;
    std::cout << "  qx=" << init_qx << " qy=" << init_qy << " qz=" << init_qz << " qw=" << init_qw << std::endl;
    std::cout << "\033[32m[Relocation] 正在尝试重定位...\033[0m" << std::endl;

    // 严格按照宇树底层要求的格式拼接 Json
    auto tryRelocation = [&](float x, float y, float z, float qx, float qy, float qz, float qw) -> int32_t {
        nlohmann::json j;
        j["data"]["x"] = (double)x; 
        j["data"]["y"] = (double)y; 
        j["data"]["z"] = (double)z;
        j["data"]["q_x"] = (double)qx; 
        j["data"]["q_y"] = (double)qy; 
        j["data"]["q_z"] = (double)qz;
        j["data"]["q_w"] = (double)qw;
        j["data"]["address"] = fullPath;
        std::string parameter = j.dump(), data;
        return Call(ROBOT_API_ID_START_RELOCATION_PL, parameter, data);
    };

    // 执行首次重定位尝试
    int32_t statusCode = tryRelocation(init_x, init_y, init_z, init_qx, init_qy, init_qz, init_qw);
    
    // 如果失败则进入交互重试循环
    while (statusCode != 0) {
        std::cout << "\n\033[33m========== 重定位失败 (errorCode:" << statusCode << ") ==========\033[0m\n选择重试方式：\n [1] 初始位姿重试\n [2] 当前位姿重试\n [3] 手动输入\n [0] 取消\n输入选择: ";
        char choice; 
        if (!(std::cin >> choice)) { std::cin.clear(); std::cin.ignore(10000, '\n'); break; }
        std::cin.ignore(10000, '\n');

        if (choice == '0') break;
        else if (choice == '1' && hasInitPose) statusCode = tryRelocation(init_x, init_y, init_z, init_qx, init_qy, init_qz, init_qw);
        else if (choice == '2') statusCode = tryRelocation(curPose.x, curPose.y, curPose.z, curPose.q_x, curPose.q_y, curPose.q_z, curPose.q_w);
        else if (choice == '3') {
            std::cout << "输入 x y z q_x q_y q_z q_w: ";
            float mx, my, mz, mqx, mqy, mqz, mqw;
            if (std::cin >> mx >> my >> mz >> mqx >> mqy >> mqz >> mqw) {
                std::cin.ignore(10000, '\n');
                statusCode = tryRelocation(mx, my, mz, mqx, mqy, mqz, mqw);
            }
        }
    }

    // =========================================================================
    // 5. 自动发布对应名称的 2D 栅格地图及生命周期激活 (Transition)
    // =========================================================================
    if (statusCode == 0) {
        std::cout << "\n\033[32m[Relocation] 重定位成功！准备发布对应的 2D 地图服务...\033[0m" << std::endl;
        
        std::string mapYamlPath = "/home/unitree/lab/map/" + currentMapName + ".yaml";
        
        // 检查 2D yaml 配置文件是否存在（依赖于 endMappingPlFun 中自动生成的栅格地图）
        std::ifstream checkYaml(mapYamlPath);
        if (!checkYaml.is_open()) {
            std::cout << "\033[31m[警告] 未找到对应 2D 栅格地图文件: " << mapYamlPath << "，请确认之前建图时已成功生成。自动发布取消。\033[0m" << std::endl;
        } else {
            checkYaml.close();

            // 预清理：杀死残存的旧地图节点，防止节点名和端口冲突
            std::cout << "\033[36m[System] 清理残留的 map_server 进程...\033[0m" << std::endl;
            std::system("pkill -f map_server > /dev/null 2>&1"); 
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 给系统内核预留释放资源的时间

            // 异步后台拉起 ROS2 nav2_map_server
            std::string ros2_cmd = "ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=" + mapYamlPath + " > /dev/null 2>&1 &";
            std::cout << "\033[32m[System] 正在后台启动 map_server 节点...\033[0m\n路径: " << mapYamlPath << std::endl;
            std::system(ros2_cmd.c_str());
            
            // 异步后台拉起声明周期管理，触发状态机 Transition 转换为 Active 激活状态
            std::string transition_cmd = "(sleep 1.5 && ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p node_names:='[\"map_server\"]' -p autostart:=true) > /dev/null 2>&1 &";
            std::cout << "\033[32m[System] 正在触发地图服务器状态机生命周期激活 (Transition Active)...\033[0m" << std::endl;
            std::system(transition_cmd.c_str());
        }
    }
}

void unitree::robot::slam::TestClient::pauseNavFun() { std::string p = R"({"data": {}})", d; Call(ROBOT_API_ID_PAUSE_NAV, p, d); }
void unitree::robot::slam::TestClient::resumeNavFun() { std::string p = R"({"data": {}})", d; Call(ROBOT_API_ID_RESUME_NAV, p, d); }

/* ========== 升級版：支援 ASCII 與 Binary 的原生 PCD 轉 2D 柵格地圖 (高度再上調 10cm) ========== */
void unitree::robot::slam::TestClient::pcdToGridMap(const std::string& pcdPath, const std::string& mapName)
{
    // 改用 binary 模式打開檔案，相容兩種格式
    std::ifstream ifs(pcdPath, std::ios::in | std::ios::binary);
    if (!ifs.is_open()) {
        std::cout << "\033[31m[地圖轉換] 無法打開 PCD 檔案: " << pcdPath << "\033[0m" << std::endl;
        return;
    }

    std::cout << "\033[36m[地圖轉換] 開始讀取 PCD 並解算 2D 地圖... (請稍候)\033[0m" << std::endl;

    std::string line;
    int data_type = 0; // 0: ascii, 1: binary
    long unsigned int points_num = 0;
    int fields_count = 3; // 預設 x, y, z 3個欄位
    int point_size = 16;  // 每個點佔用的位元組數
    long header_end_pos = 0;
    
    // 1. 讀取並解析 PCD 標頭
    while (std::getline(ifs, line)) {
        if (line.rfind("FIELDS", 0) == 0) {
            std::stringstream ss(line.substr(7));
            std::string field;
            fields_count = 0;
            while (ss >> field) fields_count++;
        } else if (line.rfind("POINTS", 0) == 0) {
            points_num = std::stoul(line.substr(7));
        } else if (line.rfind("DATA", 0) == 0) {
            if (line.substr(5, 6) == "binary") {
                data_type = 1;
            }
            header_end_pos = ifs.tellg();
            break;
        }
    }

    if (points_num == 0) {
        std::cout << "\033[31m[地圖轉換] PCD 標頭解析失敗或點雲數量為 0\033[0m" << std::endl;
        ifs.close();
        return;
    }

    // 點雲邊界與數據容器
    float min_x = 99999.0f, max_x = -99999.0f, min_y = 99999.0f, max_y = -99999.0f;
    struct Point2D { float x, y; };
    std::vector<Point2D> valid_points;
    valid_points.reserve(points_num);

    // 【極致高度切面過濾】
    // 設為 0.0f 代表完全過濾掉雷達水平面以下（45cm 以下）的所有點雲
    // 徹底將狗身、狗大腿、狗膝關節在運動時的所有點雲在 3D 空間上直接切除！
    float min_z_filter = 0.32f; 
    float max_z_filter = 1.0f;  // 過濾掉天花板

    // 2. 根據格式讀取點雲數據
    if (data_type == 1) {
        // ========== BINARY 格式解析 ==========
        ifs.seekg(header_end_pos);
        point_size = fields_count * 4; 
        std::vector<char> buffer(point_size);

        for (size_t i = 0; i < points_num; ++i) {
            if (!ifs.read(buffer.data(), point_size)) break;
            
            float x = *reinterpret_cast<float*>(&buffer[0]);
            float y = *reinterpret_cast<float*>(&buffer[4]);
            float z = *reinterpret_cast<float*>(&buffer[8]);

            if (z >= min_z_filter && z <= max_z_filter) {
                // 【水平本體防護罩】排除掉雷達水平半徑 42 公分以內狗本體與外擺關節產生的噪點
                float dist_sq = x * x + y * y;
                if (dist_sq < 0.2025f) { // 0.42m * 0.42m = 0.1764
                    continue;
                }

                valid_points.push_back({x, y});
                if (x < min_x) min_x = x; if (x > max_x) max_x = x;
                if (y < min_y) min_y = y; if (y > max_y) max_y = y;
            }
        }
    } else {
        // ========== ASCII 格式解析 ==========
        float x, y, z;
        while (ifs >> x >> y >> z) {
            ifs.ignore(1024, '\n');
            if (z >= min_z_filter && z <= max_z_filter) {
                // 【水平本體防護罩】排除掉雷達水平半徑 42 公分以內狗本體與外擺關節產生的噪點
                float dist_sq = x * x + y * y;
                if (dist_sq < 0.1764f) {
                    continue;
                }

                valid_points.push_back({x, y});
                if (x < min_x) min_x = x; if (x > max_x) max_x = x;
                if (y < min_y) min_y = y; if (y > max_y) max_y = y;
            }
        }
    }
    ifs.close();

    if (valid_points.empty()) {
        std::cout << "\033[31m[地圖轉換] 點雲過濾後無有效障礙物點！\033[0m" << std::endl;
        return;
    }

    // 3. 地圖參數與生成邏輯
    float resolution = 0.05f; // 5cm 解析度
    int padding = 20;         // 邊緣留白像素
    
    int width = std::ceil((max_x - min_x) / resolution) + padding * 2;
    int height = std::ceil((max_y - min_y) / resolution) + padding * 2;
    
    float origin_x = min_x - padding * resolution;
    float origin_y = min_y - padding * resolution;

    std::vector<uint8_t> map_data(width * height, 255);

    for (const auto& pt : valid_points) {
        int gx = (pt.x - origin_x) / resolution;
        int gy = (pt.y - origin_y) / resolution;
        
        if (gx >= 0 && gx < width && gy >= 0 && gy < height) {
            map_data[gy * width + gx] = 0;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    int nx = gx + dx; int ny = gy + dy;
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        map_data[ny * width + nx] = 0;
                    }
                }
            }
        }
    }

    // 輸出 PGM 圖片
    std::string pgm_path = "/home/unitree/lab/map/" + mapName + ".pgm";
    std::ofstream pgm(pgm_path, std::ios::binary);
    if (pgm.is_open()) {
        pgm << "P5\n" << width << " " << height << "\n255\n";
        for (int r = height - 1; r >= 0; --r) {
            pgm.write(reinterpret_cast<char*>(&map_data[r * width]), width);
        }
        pgm.close();
        std::cout << "\033[32m[地圖轉換] PGM 地圖已成功生成: " << pgm_path << "\033[0m" << std::endl;
    }

    // 輸出 YAML 配置文件
    std::string yaml_path = "/home/unitree/lab/map/" + mapName + ".yaml";
    std::ofstream yaml(yaml_path);
    if (yaml.is_open()) {
        yaml << "image: " << mapName << ".pgm\n";
        yaml << "resolution: " << resolution << "\n";
        yaml << "origin: [" << origin_x << ", " << origin_y << ", 0.0]\n";
        yaml << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
        yaml.close();
        std::cout << "\033[32m[地圖轉換] YAML 配置已成功生成: " << yaml_path << "\033[0m" << std::endl;
    }
}

unsigned char unitree::robot::slam::TestClient::keyDetection()
{
    termios tms_old, tms_new;
    tcgetattr(0, &tms_old); tms_new = tms_old; tms_new.c_lflag &= ~(ICANON | ECHO); tcsetattr(0, TCSANOW, &tms_new);
    unsigned char ch = getchar(); tcsetattr(0, TCSANOW, &tms_old);
    std::cout << "\033[1;32m" << "按下按键 " << ch << "\033[0m" << std::endl;
    return ch;
}

unsigned char unitree::robot::slam::TestClient::keyExecute()
{
    unsigned char currentKey;
    while (true)
    {
        currentKey = keyDetection();
        if (currentKey == '\n' || currentKey == '\r' || currentKey == ' ') continue;

        switch (currentKey)
        {
        case 'q': startMappingPlFun(); break;
        case 'w': endMappingPlFun(); break;
        case 'a': relocationPlFun(); break;
        case 's':
        {
            if (curPose.x == 0.0f && curPose.y == 0.0f && curPose.z == 0.0f) {
                std::cout << "\033[31m[警告] 当前位姿全为0！请确认重定位已成功。\033[0m" << std::endl; break;
            }
            std::cin.ignore(10000, '\n'); 

            std::string tts_input;
            std::cout << "请输入该任务点的 TTS 播报内容 (直接回车=不播报): ";
            std::getline(std::cin, tts_input);

            std::string act_input;
            std::cout << "请选择到达后动作 (直接回车=无, g=机械臂抓取, r=释放水瓶): ";
            std::getline(std::cin, act_input);

            poseDate newPose = curPose; 
            newPose.tts_text = tts_input;
            
            if (act_input == "g") newPose.action = "grasp";
            if (act_input == "r") newPose.action = "release";
            
            poseList.push_back(newPose); 
            newPose.printInfo(); 
            saveTasks(); 
            break;
        }
        case 'd': if (poseList.empty()) break; taskThreadRun(); break;
        case 'c': if (poseList.empty()) break; taskThreadRunSingle(); break;
        case 'f': poseList.clear(); saveTasks(); break;
        case 'l': listTaskFun(); break;
        case 'r': deleteTaskFun(); break;
        case 'i': insertTaskFun(); break;
        case 'e': editTaskFun(); break;
        case 'z': pauseNavFun(); break;
        case 'x': resumeNavFun(); break;
        case 'v': showLogs(); break;           // 查看日志摘要
        case 'o': exportCsvLog(); break;        // 导出CSV
        default:
            std::cout << "\033[33m[确认] 按 'o' 停止 SLAM，其他键取消: \033[0m";
            char confirm = getchar(); std::cin.ignore(10000, '\n'); 
            if (confirm == 'o' || confirm == 'O') {
                taskThreadStop(); stopNodeFun();
            }
            break;
        }
    }
}

int main(int argc, const char **argv)
{
    if (argc < 2) { std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl; exit(-1); }
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    unitree::robot::slam::TestClient tc;
    tc.Init(); tc.SetTimeout(10.0f);
    tc.keyExecute();
    return 0;
}
