#ifndef ADAPTIVE_OSCILLATOR_H // 头文件防重复包含宏开始
#define ADAPTIVE_OSCILLATOR_H // 头文件防重复包含宏定义

#include <algorithm> // std::clamp
#include <cmath> // 三角函数与指数函数
#include <string> // std::string

#include "TEXTIO.h" // 数据记录工具
#include "UnitConv.h" // 角度单位换算宏

// 多谐波自适应振荡器：在线估计相位与频率，并基于MHF事件进行相位误差校正
class AdaptiveOscillator // 定义自适应振荡器类
{
public: // 公有接口开始
    static constexpr int NumHarmonics = 3; // 谐波数量常量
    AdaptiveOscillator(double dt_sec) : // 构造函数，传入采样周期
        dt(dt_sec) // 初始化采样周期成员
    {
        resetState(); // 重置状态
    }

    // 参数初始化：频率与学习率
    void init(double init_freq_rad, double learn_rate_omega, double learn_rate_amp, double learn_rate_phi) // 初始化参数接口
    {
        omega = init_freq_rad; // 设置初始频率
        nu_omega = learn_rate_omega; // 设置频率学习率
        eta = learn_rate_amp; // 设置幅值/偏置学习率
        nu_phi = learn_rate_phi; // 设置相位学习率

        for (int i = 0; i < NumHarmonics; ++i) // 遍历谐波
        {
            alpha[i] = 0.01; // 设置初始幅值
            phase[i] = 0.0; // 设置初始相位
        }
        offset = 0.0; // 设置偏置

        last_mhf_time = -1.0; // 重置事件时间
        sample_count = 0; // 重置采样计数
    }

    // 单步更新：输入为左右髋角差，返回重构信号
    double update(double input) // 单步更新接口
    {
        double current_time = sample_count * dt; // 计算当前时间
        double current_velocity = (input - input_last) / dt; // 近似角速度
        if (detectMHFInternal(current_time, current_velocity, omega)) // 内部MHF检测
        {
            onGaitEventDetected(); // 自动触发相位校正
        }

        input_last = input; // 记录最新输入

        last_phase_correction = computePhaseCorrection(current_time); // 计算输出端相位校正量

        double reconstruction = offset; // 初始化重构信号为偏置
        double sum_alpha = 0.0; // 初始化幅值和
        for (int i = 0; i < NumHarmonics; ++i) // 遍历谐波
        {
            sum_alpha += alpha[i]; // 累加幅值
            reconstruction += alpha[i] * std::sin(phase[i]); // AO内部仅基于相位重构信号
        }

        double error = input - reconstruction; // 计算误差
        double denom = std::fabs(sum_alpha); // 计算归一化分母
        if (denom < 1e-3) denom = 1e-3; // 防止除零

        double d_omega = nu_omega * (error / denom) * std::cos(phase[0]); // 频率自适应项
        omega += d_omega * dt; // 先更新频率（半隐式欧拉）
        omega = std::clamp(omega, min_omega, max_omega); // 频率限幅

        for (int i = 0; i < NumHarmonics; ++i) // 遍历谐波
        {
            int harmonic = i + 1; // 当前谐波序号
            double d_phase = harmonic * omega + nu_phi * (error / denom) * std::cos(phase[i]); // 相位更新项
            phase[i] += d_phase * dt; // 更新相位
            phase[i] = wrapPhase(phase[i]); // 相位归一到[0,2π)，减少数据量

            double d_alpha = eta * error * std::sin(phase[i]); // 幅值更新项
            alpha[i] += d_alpha * dt; // 更新幅值
            if (alpha[i] < 0.0) alpha[i] = 0.0; // 幅值非负约束
        }

        offset += eta * error * dt; // 更新偏置

        reconstruction_last = reconstruction; // 记录最新重构值
        sample_count++; // 增加采样计数

        return reconstruction; // 返回重构信号
    }

    // 步态事件触发：记录相位误差快照，后续指数衰减校正
    void onGaitEventDetected() // 事件触发接口
    {
        double phase_error = -wrapPhase(phase[0]); // 目标输出相位为0，对内部相位取反
        const double pi = 3.141592653589793; // π 常量
        while (phase_error > pi) phase_error -= 2.0 * pi; // 约束到[-π,π]
        while (phase_error < -pi) phase_error += 2.0 * pi; // 约束到[-π,π]
        phase_error_snapshot = phase_error; // 保存误差快照
        phase_correction_start_time = sample_count * dt; // 记录误差开始时间
        last_phase_correction = phase_error_snapshot; // 初始化校正量
    }

    double getEstimatedPhase() const // 获取相位
    {
        return wrapPhase(phase[0] + last_phase_correction); // 返回校正后的相位
    }

    double getEstimatedPhaseNormalized() const // 获取归一化相位
    {
        return wrapPhase(phase[0] + last_phase_correction) / (2.0 * 3.141592653589793); // 归一化到[0,1]
    }

    double getEstimatedFreq() const // 获取频率
    {
        return omega; // 返回频率
    }

    double getInputLast() const // 获取最近输入
    {
        return input_last; // 返回输入
    }

    double getReconstructionLast() const // 获取最近重构值
    {
        return reconstruction_last; // 返回重构值
    }

    // 数据记录路径
    void setDataRecordingPath(const std::string& path) // 设置记录路径
    {
        recorder_input.setPath(path + "\\ao_input.txt"); // 设置输入记录文件
        recorder_reconstruction.setPath(path + "\\ao_reconstruction.txt"); // 设置重构记录文件
        recorder_phase.setPath(path + "\\ao_phase.txt"); // 设置相位记录文件
        recorder_freq.setPath(path + "\\ao_frequency.txt"); // 设置频率记录文件
    }

    // 记录输入、重构、相位与频率
    void dataRecording() // 记录接口
    {
        recorder_input(input_last); // 记录输入
        recorder_reconstruction(reconstruction_last); // 记录重构
        recorder_phase(getEstimatedPhaseNormalized()); // 记录相位
        recorder_freq(omega); // 记录频率
    }

    void stopRecording() // 结束记录
    {
        recorder_input.ending(); // 关闭输入文件
        recorder_reconstruction.ending(); // 关闭重构文件
        recorder_phase.ending(); // 关闭相位文件
        recorder_freq.ending(); // 关闭频率文件
    }

private: // 私有成员开始
    double dt = 0.0; // 采样周期

    double nu_omega = 0.0; // 频率学习率
    double nu_phi = 0.0; // 相位学习率
    double eta = 0.0; // 幅值/偏置学习率

    double phase[NumHarmonics] = {}; // 各谐波相位
    double omega = 0.0; // 基频角速度
    double alpha[NumHarmonics] = {}; // 各谐波幅值
    double offset = 0.0; // 偏置

    double input_last = 0.0; // 最近输入
    double reconstruction_last = 0.0; // 最近重构

    double mhf_threshold = DEG2RAD(20.0); // MHF最小角度阈值
    double prev_velocity = 0.0; // 上一时刻角速度
    double last_mhf_time = -1.0; // 上次事件时间
    unsigned long long sample_count = 0; // 采样计数

    double min_omega = 0.5; // 频率下限
    double max_omega = 15.0; // 频率上限

    double phase_error_snapshot = 0.0; // 相位误差快照
    double phase_correction_start_time = -1.0; // 相位校正起始时间
    double last_phase_correction = 0.0; // 最近相位校正量

    WriteDataIntoText recorder_input; // 输入记录器
    WriteDataIntoText recorder_reconstruction; // 重构记录器
    WriteDataIntoText recorder_phase; // 相位记录器
    WriteDataIntoText recorder_freq; // 频率记录器

    void resetState() // 重置状态
    {
        for (int i = 0; i < NumHarmonics; ++i) // 遍历谐波
        {
            phase[i] = 0.0; // 重置相位
            alpha[i] = 0.0; // 重置幅值
        }
        omega = 0.0; // 重置频率
        offset = 0.0; // 重置偏置
        input_last = 0.0; // 重置输入
        reconstruction_last = 0.0; // 重置重构
        last_mhf_time = -1.0; // 重置事件时间
        sample_count = 0; // 重置计数
        phase_error_snapshot = 0.0; // 重置误差快照
        phase_correction_start_time = -1.0; // 重置校正起始时间
        last_phase_correction = 0.0; // 重置校正量
    }

    double wrapPhase(double value) const // 相位环绕函数
    {
        const double two_pi = 2.0 * 3.141592653589793; // 2π 常量
        while (value < 0.0) value += two_pi; // 小于0则加2π
        while (value >= two_pi) value -= two_pi; // 大于等于2π则减2π
        return value; // 返回环绕后的相位
    }

    double computePhaseCorrection(double current_time) // 计算相位校正量
    {
        if (phase_correction_start_time < 0.0) // 未触发事件
        {
            return 0.0; // 无校正
        }

        if (std::fabs(last_phase_correction) <= 1e-4) // 校正量已足够小
        {
            return 0.0; // 跳过指数计算
        }

        double dt_event = current_time - phase_correction_start_time; // 事件经过时间
        if (dt_event < 0.0) dt_event = 0.0; // 保护负值
        return phase_error_snapshot * std::exp(-omega * dt_event); // 指数衰减校正
    }

    // MHF检测：角速度过零 + 动态步态周期去抖
    bool detectMHFInternal(double current_time, double current_velocity, double current_omega) // 内部MHF检测
    {
        double stride_period = 2.0 * 3.141592653589793 / current_omega; // 动态步态周期
        bool is_far_enough = (last_mhf_time < 0.0) || ((current_time - last_mhf_time) > 0.7 * stride_period); // 动态去抖
        bool is_zero_crossing = (prev_velocity > 0.0 && current_velocity <= 0.0); // 过零检测

        if (is_zero_crossing && is_far_enough) // 满足触发条件
        {
            last_mhf_time = current_time; // 更新上次事件时间
            prev_velocity = current_velocity; // 更新速度状态
            return true; // 返回触发
        }

        prev_velocity = current_velocity; // 更新速度状态
        return false; // 未触发
    }
}; // 类定义结束

#endif // ADAPTIVE_OSCILLATOR_H // 头文件防重复包含宏结束
