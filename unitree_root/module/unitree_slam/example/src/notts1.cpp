//遇到障碍物避障播报

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
#include <dirent.h>
#include <algorithm>
#include <atomic>
#include <cmath>    // for std::sqrt, std::atan2, M_PI

#define SlamInfoTopic "rt/slam_info"
#define SlamKeyInfoTopic "rt/slam_key_info"
// 默认音频设备，如果测试无声音可改为 "default" 或 "plughw:1,0"
#define AUDIO_DEVICE "plughw:0,0"

using namespace unitree::robot;
using namespace unitree::common;
unsigned char currentKey;

// 定义导航状态枚举
enum class NavStatus {
    NAVIGATING, // 导航进行中
    ARRIVED,    // 成功到达目标点（位置与方向皆符合）
    FAILED      // 导航失败或超时
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
    float speed = 0.4f;
    std::string audio_path = "";  // 本地 mp3 音频路径
    std::string action = "";      // 到达后执行动作: "grasp"=机械臂抓取, "release"=释放

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
        j["audio_path"] = audio_path;
        j["action"] = action;
        return j;
    }

    void printInfo() const
    {
        std::cout << "x:" << x << " y:" << y << " z:" << z
                  << " | q_w:" << q_w << " q_z:" << q_z
                  << " | 音频(MP3):[" << (audio_path.empty() ? "无" : audio_path) << "]"
                  << " | 动作:[" << (action.empty() ? "无" : action) << "]" << std::endl;
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

        // 线程安全的原子状态机
        std::atomic<NavStatus> m_navStatus{NavStatus::NAVIGATING};

        bool threadControl = false;
        std::future<void> futThread;
        std::promise<void> prom;
        std::thread controlThread;

        /* ========== 角度计算辅助 ========== */
        float calculateYaw(float qx, float qy, float qz, float qw);
        float angleDistance(float alpha, float beta);

        /* ========== 任务点管理 ========== */
        bool isNavigating() const;
        void listTaskFun();
        void deleteTaskFun();
        void insertTaskFun();
        void editTaskFun();
        void saveTasks();
        void loadTasks();

        /* ========== 地图管理 ========== */
        std::vector<std::string> listPcdMaps();
        std::string currentMapName;
        std::string getTaskPath() const;
        void pcdToGridMap(const std::string& pcdPath, const std::string& mapName);

        /* ========== 本地 MP3 音频播报 ========== */
        void playAudio(const std::string& audioPath) const;

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

/* ========== 角度计算辅助函数 ========== */
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

/* ========== 导航状态检查 ========== */
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

                p.audio_path = pj.value("audio_path", "");
                p.action = pj.value("action", "");
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

    std::string audio_input;
    std::cout << "请输入该任务点的 .mp3 音频路径 (例: /home/unitree/mp3/point1.mp3, 回车=无): ";
    std::getline(std::cin, audio_input);

    std::string act_input;
    std::cout << "请选择到达后动作 (直接回车=无, g=机械臂抓取, r=释放水瓶): ";
    std::getline(std::cin, act_input);

    poseDate newPose = curPose;
    newPose.audio_path = audio_input;
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

    std::cout << "当前 MP3 音频: [" << (poseList[idx].audio_path.empty() ? "无" : poseList[idx].audio_path) << "]\n是否修改？(y/n): ";
    char audio_choice; std::cin >> audio_choice; std::cin.ignore(10000, '\n');
    if (audio_choice == 'y' || audio_choice == 'Y') {
        std::cout << "输入新 .mp3 音频路径: ";
        std::getline(std::cin, poseList[idx].audio_path);
    }
    saveTasks();
}

/* ========== .mp3 本地音频播报逻辑 ========== */
void unitree::robot::slam::TestClient::playAudio(const std::string& audioPath) const
{
    if (audioPath.empty()) return;

    // 检查 .mp3 路径文件是否存在
    std::ifstream checkFile(audioPath);
    if (!checkFile.is_open()) {
        std::cout << "\033[31m[Audio Error] 找不到音频文件: " << audioPath << "\033[0m" << std::endl;
        return;
    }
    checkFile.close();

    std::cout << "\033[1;36m[Audio] 正在播放点位 MP3 音频: " << audioPath << "\033[0m" << std::endl;

    // 优先使用 mpg123，如未安装则尝试 ffplay
    std::string playCmd = "mpg123 -a " AUDIO_DEVICE " -q " + audioPath +
                          " 2>/dev/null || ffplay -nodisp -autoexit -loglevel quiet " + audioPath + " 2>/dev/null";

    std::system(playCmd.c_str());
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

/* ========== 单次巡航逻辑 ========== */
void unitree::robot::slam::TestClient::taskLoopFunSingle(std::promise<void> &prom)
{
    std::string data;
    threadControl = true;
    std::cout << "\033[34m[巡航启动] 单次巡航开始，总任务点数量: " << poseList.size() << "\033[0m" << std::endl;

    for (int i = 0; i < (int)poseList.size(); i++)
    {
        m_navStatus = NavStatus::NAVIGATING;

        float init_dx = curPose.x - poseList[i].x;
        float init_dy = curPose.y - poseList[i].y;
        float init_distance = std::sqrt(init_dx * init_dx + init_dy * init_dy);

        bool isPureRotation = (init_distance < 0.05f);

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

        auto startTime = std::chrono::steady_clock::now();
        int printCounter = 0;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        while (m_navStatus == NavStatus::NAVIGATING)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (!threadControl) break;

            float dx = curPose.x - poseList[i].x;
            float dy = curPose.y - poseList[i].y;
            float distance = std::sqrt(dx * dx + dy * dy);

            float curYaw = calculateYaw(curPose.q_x, curPose.q_y, curPose.q_z, curPose.q_w);
            float targetYaw = calculateYaw(poseList[i].q_x, poseList[i].q_y, poseList[i].q_z, poseList[i].q_w);
            float angleDiffDeg = angleDistance(curYaw, targetYaw) * 180.0f / M_PI;

            if (++printCounter % 15 == 0) {
                std::cout << "[巡航中] 点 [" << i << "] -> 距离剩: " << distance
                          << " 米 | 角度偏差: " << angleDiffDeg << " 度"
                          << (isPureRotation ? " (纯转向模式)" : "") << std::endl;
            }

            float targetAngleThreshold = isPureRotation ? 5.0f : 12.0f;
            float targetDistThreshold = isPureRotation ? 0.05f : 0.12f;

            if (distance < targetDistThreshold && angleDiffDeg < targetAngleThreshold) {
                std::cout << "\033[32m[自主判定] 成功！点 [" << i << "] 位置与精确定向均已达标！\033[0m" << std::endl;
                m_navStatus = NavStatus::ARRIVED;
                break;
            }

            auto currentTime = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();
            uint32_t timeoutLimit = isPureRotation ? 30 : 60;
            if (elapsed > timeoutLimit) {
                std::cout << "\033[33m[自主判定] 导航超时，强制切换至下一点。\033[0m" << std::endl;
                m_navStatus = NavStatus::FAILED;
                break;
            }
        }

        if (!threadControl) break;

        if (m_navStatus == NavStatus::ARRIVED) {
            // 到达目标点，播放对应的 .mp3 声音
            if (!poseList[i].audio_path.empty()) {
                playAudio(poseList[i].audio_path);
            }

            // 到达后执行的扩展动作
            if (poseList[i].action == "grasp") {
                std::cout << "\033[1;35m[动作] 到达抓取点，启动机械臂抓取...\033[0m" << std::endl;
                int ret = std::system("python3 /home/unitree/vision+arm/auto_grasp.py");
                std::cout << "\033[1;35m[动作] 抓取完成 (返回码:" << ret << ")\033[0m" << std::endl;
            }
            if (poseList[i].action == "release") {
                std::cout << "\033[1;35m[动作] 到达释放点，启动释放...\033[0m" << std::endl;
                int ret = std::system("python3 /home/unitree/vision+arm/release.py");
                std::cout << "\033[1;35m[动作] 释放完成 (返回码:" << ret << ")\033[0m" << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::seconds(1));
        } else if (m_navStatus == NavStatus::FAILED) {
              std::cout << "\033[1;31m[障碍提示] 目标点无效(Invalid goal)或遇障卡住，播放 6.mp3！\033[0m" << std::endl;
              playAudio("/home/unitree/lab/music/6.mp3");
              std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }

    std::cout << "\033[32m单次巡航任务结束。\033[0m" << std::endl;
    prom.set_value();
}

/* ========== 循环巡航逻辑 ========== */
void unitree::robot::slam::TestClient::taskLoopFun(std::promise<void> &prom)
{
    std::string data;
    threadControl = true;
    std::cout << "\033[34m[巡航启动] 循环巡航开始，总任务点数量: " << poseList.size() << "\033[0m" << std::endl;

    for (int i = 0; i < (int)poseList.size(); i++)
    {
        m_navStatus = NavStatus::NAVIGATING;

        float init_dx = curPose.x - poseList[i].x;
        float init_dy = curPose.y - poseList[i].y;
        float init_distance = std::sqrt(init_dx * init_dx + init_dy * init_dy);

        bool isPureRotation = (init_distance < 0.05f);

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

        auto startTime = std::chrono::steady_clock::now();
        int printCounterLoop = 0;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        while (m_navStatus == NavStatus::NAVIGATING)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (!threadControl) break;

            float dx = curPose.x - poseList[i].x;
            float dy = curPose.y - poseList[i].y;
            float distance = std::sqrt(dx * dx + dy * dy);

            float curYaw = calculateYaw(curPose.q_x, curPose.q_y, curPose.q_z, curPose.q_w);
            float targetYaw = calculateYaw(poseList[i].q_x, poseList[i].q_y, poseList[i].q_z, poseList[i].q_w);
            float angleDiffDeg = angleDistance(curYaw, targetYaw) * 180.0f / M_PI;

            if (++printCounterLoop % 15 == 0) {
                std::cout << "[巡航中] 点 [" << i << "] -> 距离剩: " << distance
                          << " 米 | 角度偏差: " << angleDiffDeg << " 度"
                          << (isPureRotation ? " (纯转向模式)" : "") << std::endl;
            }

            float targetAngleThreshold = isPureRotation ? 5.0f : 8.0f;
            float targetDistThreshold = isPureRotation ? 0.05f : 0.12f;

            if (distance < targetDistThreshold && angleDiffDeg < targetAngleThreshold) {
                std::cout << "\033[32m[自主判定] 成功！点 [" << i << "] 位置与精确定向均已达标！\033[0m" << std::endl;
                m_navStatus = NavStatus::ARRIVED;
                break;
            }

            auto currentTime = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();
            uint32_t timeoutLimit = isPureRotation ? 30 : 60;
            if (elapsed > timeoutLimit) {
                std::cout << "\033[33m[自主判定] 导航超时，强制切换至下一点。\033[0m" << std::endl;
                m_navStatus = NavStatus::FAILED;
                break;
            }
        }

        if (!threadControl) break;

        if (m_navStatus == NavStatus::ARRIVED) {
            // 到达目标点，播放对应的 .mp3 声音
            if (!poseList[i].audio_path.empty()) {
                playAudio(poseList[i].audio_path);
            }

            // 到达后执行的扩展动作
            if (poseList[i].action == "grasp") {
                std::cout << "\033[1;35m[动作] 到达抓取点，启动机械臂抓取...\033[0m" << std::endl;
                int ret = std::system("python3 /home/unitree/vision+arm/auto_grasp.py");
                std::cout << "\033[1;35m[动作] 抓取完成 (返回码:" << ret << ")\033[0m" << std::endl;
            }
            if (poseList[i].action == "release") {
                std::cout << "\033[1;35m[动作] 到达释放点，启动释放...\033[0m" << std::endl;
                int ret = std::system("python3 /home/unitree/vision+arm/release.py");
                std::cout << "\033[1;35m[动作] 释放完成 (返回码:" << ret << ")\033[0m" << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::seconds(1));
        } else if (m_navStatus == NavStatus::FAILED) {
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }

        if (i == (int)poseList.size() - 1) {
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
    nlohmann::json jsonData;
    
    try {
        jsonData = nlohmann::json::parse(currentMsg.data());
    } catch (...) {
        return;
    }

    // -------------------------------------------------------------------
    // 🚨 核心逻辑：路径上有障碍物（触发绕行前）立即播报
    // -------------------------------------------------------------------
    bool obstacleOnPath = false;

    // 1. 解析 Unitree SLAM 消息中的障碍物/重规划事件标志
    if (jsonData.contains("event")) {
        std::string eventType = jsonData["event"].get<std::string>();
        // 捕获“检测到障碍物/开始重新规划路径”的瞬间事件
        if (eventType == "obstacle_detected" || eventType == "replanning_start" || eventType == "obstacle_avoidance_start") {
            obstacleOnPath = true;
        }
    } 
    // 2. 检查 JSON 中的 obstacle / status 状态位
    else if (jsonData.contains("obstacle") && jsonData["obstacle"].is_boolean() && jsonData["obstacle"].get<bool>()) {
        obstacleOnPath = true;
    }
    else if (jsonData.contains("data") && jsonData["data"].contains("obstacle") && jsonData["data"]["obstacle"].get<bool>()) {
        obstacleOnPath = true;
    }

    // 3. 满足遇障条件，立即触发语音播报
    if (obstacleOnPath) {
        static auto lastAlertTime = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();

        // 设置 5 秒冷却时间，防止绕行过程中重复连续播报
        if (std::chrono::duration_cast<std::chrono::seconds>(now - lastAlertTime).count() >= 5) {
            lastAlertTime = now;
            std::cout << "\033[1;33m[障碍提示] 路径前方发现障碍物，准备绕行，触发播报！\033[0m" << std::endl;

            // 异步线程播放，防止卡死 DDS 接收线程
            std::thread([this]() {
                // 如果你想播放 7.mp3（或 6.mp3），替换此处路径即可
                this->playAudio("/home/unitree/lab/music/7.mp3");
            }).detach();
        }
    }

    // -------------------------------------------------------------------
    // 解析位姿等其余逻辑
    // -------------------------------------------------------------------
    if (jsonData.contains("errorCode") && jsonData["errorCode"] != 0) return;

    if (jsonData.contains("type") && jsonData["type"] == "pos_info") {
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

    pcdToGridMap(fullPath, currentMapName);
}

void unitree::robot::slam::TestClient::relocationPlFun()
{
    auto maps = listPcdMaps();
    if (maps.empty()) {
        std::cout << "\033[31m/home/unitree/lab/map/ 目录下没有找到地图文件\033[0m" << std::endl;
        return;
    }

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

    std::string input;
    if (!(std::cin >> input)) { std::cin.clear(); std::cin.ignore(10000, '\n'); return; }
    std::cin.ignore(10000, '\n');

    std::string selectedMap = "";
    bool isNumber = !input.empty() && std::all_of(input.begin(), input.end(), ::isdigit);

    if (isNumber) {
        int idx = std::stoi(input);
        if (idx >= 0 && idx < (int)maps.size()) {
            selectedMap = maps[idx];
        } else {
            std::cout << "\033[31m序号超出范围！\033[0m" << std::endl;
            return;
        }
    } else {
        std::string targetFile = input + ".pcd";
        auto it = std::find(maps.begin(), maps.end(), targetFile);
        if (it != maps.end()) {
            selectedMap = *it;
        } else {
            std::cout << "\033[31m未找到名为 " << input << " 的地图文件！\033[0m" << std::endl;
            return;
        }
    }

    currentMapName = selectedMap;
    if (currentMapName.size() > 4 && currentMapName.substr(currentMapName.size() - 4) == ".pcd") {
        currentMapName = currentMapName.substr(0, currentMapName.size() - 4);
    }
    poseList.clear();
    loadTasks();

    std::string fullPath = "/home/unitree/lab/map/" + selectedMap;

    float init_x = curPose.x;
    float init_y = curPose.y;
    float init_z = curPose.z;
    float init_qx = curPose.q_x;
    float init_qy = curPose.q_y;
    float init_qz = curPose.q_z;
    float init_qw = curPose.q_w;
    bool hasInitPose = true;

    if (init_x == 0.0f && init_y == 0.0f && init_z == 0.0f &&
        init_qx == 0.0f && init_qy == 0.0f && init_qz == 0.0f && init_qw == 1.0f) {

        std::ifstream initIfs("/home/unitree/lab/map/" + currentMapName + "_init.json");
        if (initIfs.is_open()) {
            try {
                nlohmann::json initJ;
                initIfs >> initJ;
                init_x = initJ.value("x", 0.0f);
                init_y = initJ.value("y", 0.0f);
                init_z = initJ.value("z", 0.0f);
                init_qx = initJ.value("q_x", 0.0f);
                init_qy = initJ.value("q_y", 0.0f);
                init_qz = initJ.value("q_z", 0.0f);
                init_qw = initJ.value("q_w", 1.0f);
                hasInitPose = true;
                std::cout << "\033[33m[Relocation] 当前位姿为0，已从历史文件恢复初始位姿\033[0m" << std::endl;
            } catch (...) {}
            initIfs.close();
        }
    } else {
        std::cout << "\033[36m[Relocation] 使用当前位姿作为初始位姿:\033[0m" << std::endl;
        std::cout << "  x=" << init_x << " y=" << init_y << " z=" << init_z << std::endl;
    }

    std::cout << "\033[32m[Relocation] 正在尝试重定位...\033[0m" << std::endl;

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

    int32_t statusCode = tryRelocation(init_x, init_y, init_z, init_qx, init_qy, init_qz, init_qw);

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

    if (statusCode == 0) {
        std::cout << "\n\033[32m[Relocation] 重定位成功！准备发布对应的 2D 地图服务...\033[0m" << std::endl;

        std::string mapYamlPath = "/home/unitree/lab/map/" + currentMapName + ".yaml";

        std::ifstream checkYaml(mapYamlPath);
        if (!checkYaml.is_open()) {
            std::cout << "\033[31m[警告] 未找到对应 2D 栅格地图文件: " << mapYamlPath << "，请确认之前建图时已成功生成。自动发布取消。\033[0m" << std::endl;
        } else {
            checkYaml.close();

            std::cout << "\033[36m[System] 清理残留的 map_server 进程...\033[0m" << std::endl;
            std::system("pkill -f map_server > /dev/null 2>&1");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            std::string ros2_cmd = "ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=" + mapYamlPath + " > /dev/null 2>&1 &";
            std::cout << "\033[32m[System] 正在后台启动 map_server 节点...\033[0m\n路径: " << mapYamlPath << std::endl;
            std::system(ros2_cmd.c_str());

            std::string transition_cmd = "(sleep 1.5 && ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p node_names:='[\"map_server\"]' -p autostart:=true) > /dev/null 2>&1 &";
            std::cout << "\033[32m[System] 正在触发地图服务器状态机生命周期激活 (Transition Active)...\033[0m" << std::endl;
            std::system(transition_cmd.c_str());
        }
    }
}

void unitree::robot::slam::TestClient::pauseNavFun() { std::string p = R"({"data": {}})", d; Call(ROBOT_API_ID_PAUSE_NAV, p, d); }
void unitree::robot::slam::TestClient::resumeNavFun() { std::string p = R"({"data": {}})", d; Call(ROBOT_API_ID_RESUME_NAV, p, d); }

/* ========== PCD 转 2D 栅格地图转换引擎 ========== */
void unitree::robot::slam::TestClient::pcdToGridMap(const std::string& pcdPath, const std::string& mapName)
{
    std::ifstream ifs(pcdPath, std::ios::in | std::ios::binary);
    if (!ifs.is_open()) {
        std::cout << "\033[31m[地图转换] 无法打开 PCD 文件: " << pcdPath << "\033[0m" << std::endl;
        return;
    }

    std::cout << "\033[36m[地图转换] 开始读取 PCD 并解算 2D 地图... (请稍候)\033[0m" << std::endl;

    std::string line;
    int data_type = 0;
    long unsigned int points_num = 0;
    int fields_count = 3;
    int point_size = 16;
    long header_end_pos = 0;

    while (std::getline(ifs, line)) {
        if (line.rfind("FIELDS", 0) == 0) {
            std::stringstream ss(line.substr(7));
            std::string field;
            fields_count = 0;
            while (ss >> field) fields_count++;
        } else if (line.rfind("SIZE", 0) == 0) {
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
        std::cout << "\033[31m[地图转换] PCD 标头解析失败或点云数量为 0\033[0m" << std::endl;
        ifs.close();
        return;
    }

    float min_x = 99999.0f, max_x = -99999.0f, min_y = 99999.0f, max_y = -99999.0f;
    struct Point2D { float x, y; };
    std::vector<Point2D> valid_points;
    valid_points.reserve(points_num);

    float min_z_filter = 0.18f;
    float max_z_filter = 1.0f;

    if (data_type == 1) {
        ifs.seekg(header_end_pos);

        point_size = fields_count * 4;
        std::vector<char> buffer(point_size);

        for (size_t i = 0; i < points_num; ++i) {
            if (!ifs.read(buffer.data(), point_size)) break;

            float x = *reinterpret_cast<float*>(&buffer[0]);
            float y = *reinterpret_cast<float*>(&buffer[4]);
            float z = *reinterpret_cast<float*>(&buffer[8]);

            if (z >= min_z_filter && z <= max_z_filter) {
                valid_points.push_back({x, y});
                if (x < min_x) min_x = x; if (x > max_x) max_x = x;
                if (y < min_y) min_y = y; if (y > max_y) max_y = y;
            }
        }
    } else {
        float x, y, z;
        while (ifs >> x >> y >> z) {
            ifs.ignore(1024, '\n');
            if (z >= min_z_filter && z <= max_z_filter) {
                valid_points.push_back({x, y});
                if (x < min_x) min_x = x; if (x > max_x) max_x = x;
                if (y < min_y) min_y = y; if (y > max_y) max_y = y;
            }
        }
    }
    ifs.close();

    if (valid_points.empty()) {
        std::cout << "\033[31m[地图转换] 点云过滤后无有效障碍物点！\033[0m" << std::endl;
        return;
    }

    float resolution = 0.05f;
    int padding = 20;

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

    std::string pgm_path = "/home/unitree/lab/map/" + mapName + ".pgm";
    std::ofstream pgm(pgm_path, std::ios::binary);
    if (pgm.is_open()) {
        pgm << "P5\n" << width << " " << height << "\n255\n";
        for (int r = height - 1; r >= 0; --r) {
            pgm.write(reinterpret_cast<char*>(&map_data[r * width]), width);
        }
        pgm.close();
        std::cout << "\033[32m[地图转换] PGM 地图已成功生成: " << pgm_path << "\033[0m" << std::endl;
    }

    std::string yaml_path = "/home/unitree/lab/map/" + mapName + ".yaml";
    std::ofstream yaml(yaml_path);
    if (yaml.is_open()) {
        yaml << "image: " << mapName << ".pgm\n";
        yaml << "resolution: " << resolution << "\n";
        yaml << "origin: [" << origin_x << ", " << origin_y << ", 0.0]\n";
        yaml << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
        yaml.close();
        std::cout << "\033[32m[地图转换] YAML 配置已成功生成: " << yaml_path << "\033[0m" << std::endl;
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

            std::string audio_input;
            std::cout << "请输入该任务点的 .mp3 音频路径 (例: /home/unitree/mp3/point1.mp3, 直接回车=不播放): ";
            std::getline(std::cin, audio_input);

            std::string act_input;
            std::cout << "请选择到达后动作 (直接回车=无, g=机械臂抓取, r=释放水瓶): ";
            std::getline(std::cin, act_input);

            poseDate newPose = curPose;
            newPose.audio_path = audio_input;

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

