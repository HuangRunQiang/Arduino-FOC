#include "lowpass_filter.h"

// 低通滤波器的构造函数
LowPassFilter::LowPassFilter(float time_constant)
    : Tf(time_constant)  // 初始化时间常数
    , y_prev(0.0f)      // 初始化前一个输出值为0
{
    timestamp_prev = _micros();  // 获取当前时间戳
}

// 重载运算符()，实现低通滤波
float LowPassFilter::operator() (float x)
{
    unsigned long timestamp = _micros();  // 获取当前时间戳
    float dt = (timestamp - timestamp_prev) * 1e-6f;  // 计算时间间隔

    // 如果时间间隔小于0，则设置为1毫秒
    if (dt < 0.0f) dt = 1e-3f;
    // 如果时间间隔大于0.3秒，直接返回当前输入值
    else if (dt > 0.3f) {
        y_prev = x;  // 更新前一个输出值
        timestamp_prev = timestamp;  // 更新前一个时间戳
        return x;  // 返回当前输入值
    }

    // 计算滤波系数
    float alpha = Tf / (Tf + dt);
    // 计算当前输出值
    float y = alpha * y_prev + (1.0f - alpha) * x;
    y_prev = y;  // 更新前一个输出值
    timestamp_prev = timestamp;  // 更新前一个时间戳
    return y;  // 返回当前输出值
}
