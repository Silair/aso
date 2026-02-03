//C system headers

//C++ standard library headers
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>
#include <vector>
#include <fstream>
#include <filesystem>

//other libraries' headers

//project's headers
#include "kvaser.h"
#include "motors.h"
#include "Timer.h"
#include "selfDevManuipulator.h"
#include "PIDController.h"
#include "Sinusoid.h"
#include "DataRecorder.h"
#include "aso.h"

using namespace std;

namespace {
std::string findProjectRoot()
{
    namespace fs = std::filesystem;
    fs::path cur = fs::current_path();
    for (int i = 0; i <= 6; ++i) {
        if (fs::exists(cur / "Data")) {
            return cur.string();
        }
        if (!cur.has_parent_path()) break;
        cur = cur.parent_path();
    }
    return "";
}

bool loadOfflineData(const std::string& path, std::vector<double>& out)
{
    std::ifstream in(path);
    if (!in.is_open()) {
        return false;
    }
    double v = 0.0;
    while (in >> v) {
        out.push_back(v);
    }
    return !out.empty();
}

double lerp(double a, double b, double t)
{
    return a + (b - a) * t;
}
}

void MotionTest()
{
    cout << "[INFO] MotionTest start" << endl;
    double gait_period = 0.75;//步态周期s
    int num_gait = 20;//步态次数
    double duration = gait_period * num_gait;//总运动时间
    double step_size = 0.005;//控制周期s
    int cnt = 0;
    int cnt_max = 3022;//最大循环次数

    MotorParameters;//设置电机参数
    selfDevManuipulator<2> hip;//创建髋关节对象
    hip.setMotor(motor);
    hip.CANBus().canInit(0, canBITRATE_1M);//总线初始化
    cout << "[INFO] CAN initialized" << endl;

    long id_get_L = 0x0001;
    // long id_get_R = 0x0002; // 注释：只使能单电机
    uint8_t CtlDat = 0x01;
    uint8_t CtlDat_dis = 0x00;
    hip.CANBus().MotorOnOff(id_get_L, CtlDat);
    cout << "[INFO] Motor enable sent (ID=0x0001)" << endl;
    // hip.CANBus().MotorOnOff(id_get_R, CtlDat); // 注释：只使能单电机

    PIDController<2> hip_controller(step_size);//创建PID控制对象，控制周期5ms
    std::string project_root = findProjectRoot();
    if (project_root.empty()) {
        cout << "[ERROR] Project root not found (missing Data folder)." << endl;
        return;
    }
    std::string data_root = project_root + "\\Data";
    std::string output_root = project_root + "\\output";
    hip_controller.setPIDparameters(data_root + "\\PIDController\\PlD_Parameters");//从文件中读取PID参数
    cout << "[INFO] Data root: " << data_root << endl;
    cout << "[INFO] Output root: " << output_root << endl;

    AdaptiveOscillator ao(step_size);//自适应振荡器，用离线角度驱动相位估计
    double desired_gait_period = 1.3; // 期望步态周期（秒）
    float init_freq = static_cast<float>(2.0 * 3.141592653589793 / desired_gait_period);
    float nu_omega = 20.0f;
    float eta = 5.0f;
    float nu_phi = 20.0f;
    ao.init(init_freq, nu_omega, eta, nu_phi);
    cout << "[INFO] AO initialized" << endl;

    // 使能后先读一次状态，确认总线可用
    hip.statusUpdate();//状态更新
    double theta_init = hip.Ang()[0];
    cout << "[INFO] First status update, theta_init=" << theta_init << endl;

    string offline_data_path = data_root + "\\Desired Trajectory\\offline_hip_data.txt"; // 离线髋关节角度数据
    std::vector<double> offline_data;
    if (offline_data_path.empty() || !loadOfflineData(offline_data_path, offline_data) || offline_data.size() < 2) {
        cout << "[ERROR] Offline data load failed or too short: " << offline_data_path << endl;
        return;
    }
    cout << "[INFO] Offline data loaded, samples=" << offline_data.size() << endl;

    // 记录缓冲区（完全移出实时循环）
    std::vector<double> q0_log;
    std::vector<double> dq0_log;
    std::vector<double> up0_log;
    std::vector<double> ui0_log;
    std::vector<double> ud0_log;
    std::vector<double> u0_log;
    std::vector<Eigen::Matrix<double, 5, 1>> ao_log_list; // [0]=ao_input, [1]=theta_ref, [2]=hip.Ang()[0], [3]=phi, [4]=omega

    Vector2d error_vec, d_error_vec;//位置误差，速度误差

    const int record_stride = 5; // 降低磁盘 I/O 频率，减小抖动
    Timer timer;
    timer.begin();
    double next_target = step_size;
    bool offset_inited = false;
    double theta_offset = 0.0;

    const double data_dt = desired_gait_period / (offline_data.size() - 1);

    while (cnt < cnt_max)
    {
        cnt++;

        hip.statusUpdate();

        double t = (cnt - 1) * step_size;
        double t_cycle = std::fmod(t, desired_gait_period);
        double idx_f = t_cycle / data_dt;
        size_t idx = static_cast<size_t>(idx_f);
        if (idx >= offline_data.size() - 1) {
            idx = offline_data.size() - 2;
            idx_f = static_cast<double>(idx);
        }
        double frac = idx_f - static_cast<double>(idx);
        double ao_input_deg = lerp(offline_data[idx], offline_data[idx + 1], frac);
        double ao_input = DEG2RAD(ao_input_deg);

        ao.update(ao_input); // AO 更新：估计相位/频率

        double phi = ao.getEstimatedPhase(); // 当前相位
        double omega = ao.getEstimatedFreq(); // 当前频率
        double target_amp = DEG2RAD(20.0); // 目标幅值

        if (!offset_inited) {
            theta_offset = theta_init - target_amp * std::sin(phi - 3.14159);
            offset_inited = true;
        }

        double theta_ref = target_amp * std::sin(phi - 3.14159) + theta_offset; // 位置偏置对齐
        double dtheta_ref = target_amp * omega * std::cos(phi - 3.14159); // 目标速度

        error_vec.setZero();
        d_error_vec.setZero();
        error_vec[0] = theta_ref - hip.Ang()[0]; // 单电机关节位置误差
        d_error_vec[0] = dtheta_ref - hip.AngVel()[0]; // 单电机关节速度误差

        // 影子控制器：仅计算/记录 PID，不用于实际控制
        hip_controller(error_vec, d_error_vec);

        double current_limit = 2.0; // 位置模式电流限幅
        hip.posVelCtrl(0, theta_ref, dtheta_ref, current_limit); // 位置模式控制

        if (cnt % record_stride == 0) {
            q0_log.push_back(hip.Ang()[0]);
            dq0_log.push_back(hip.AngVel()[0]);
            up0_log.push_back(hip_controller.getUp()[0]);
            ui0_log.push_back(hip_controller.getUi()[0]);
            ud0_log.push_back(hip_controller.getUd()[0]);
            u0_log.push_back(hip_controller.getU()[0]);
            Eigen::Matrix<double, 5, 1> ao_row;
            ao_row << ao_input, theta_ref, hip.Ang()[0], phi, omega;
            ao_log_list.push_back(ao_row);
        }

        if (cnt % 200 == 0) {
            cout << "[INFO] step=" << cnt << "/" << cnt_max << " theta_ref=" << theta_ref << " theta=" << hip.Ang()[0] << endl;
        }

        double now = timer();
        double remain = next_target - now;
        if (remain > 0.0) {
            Timer::usleep(remain * 1e6);
        }
        next_target += step_size;
    }

    hip.brake();
    hip.CANBus().MotorOnOff(id_get_L, CtlDat_dis);
    // hip.CANBus().MotorOnOff(id_get_R, CtlDat_dis); // 注释：只使能单电机
    cout << "[INFO] Motors disabled, writing logs..." << endl;

    // 循环结束后统一落盘
    {
        std::filesystem::create_directories(output_root + "\\Robot Status");
        std::filesystem::create_directories(output_root + "\\PIDController\\Output");
        std::filesystem::create_directories(output_root + "\\AO\\Output");

        WriteDataIntoText rec_q(output_root + "\\Robot Status\\q.txt");
        WriteDataIntoText rec_dq(output_root + "\\Robot Status\\dq.txt");
        WriteDataIntoText rec_up(output_root + "\\PIDController\\Output\\Up.txt");
        WriteDataIntoText rec_ui(output_root + "\\PIDController\\Output\\Ui.txt");
        WriteDataIntoText rec_ud(output_root + "\\PIDController\\Output\\Ud.txt");
        WriteDataIntoText rec_u(output_root + "\\PIDController\\Output\\U.txt");
        WriteDataIntoText rec_ao(output_root + "\\AO\\Output\\ao_log.txt");

        for (const auto& v : q0_log) rec_q(v);
        for (const auto& v : dq0_log) rec_dq(v);
        for (const auto& v : up0_log) rec_up(v);
        for (const auto& v : ui0_log) rec_ui(v);
        for (const auto& v : ud0_log) rec_ud(v);
        for (const auto& v : u0_log) rec_u(v);
        for (const auto& v : ao_log_list) rec_ao(v);
    }

    cout << "[INFO] MotionTest finished" << endl;
}

int main() {

    MotionTest();

    return 0;
}
