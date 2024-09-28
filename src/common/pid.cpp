#include "pid.h"

// PID控制器构造函数
PIDController::PIDController(float P, float I, float D, float ramp, float limit)
    : P(P)                       // 比例增益
    , I(I)                       // 积分增益
    , D(D)                       // 微分增益
    , output_ramp(ramp)         // 输出变化率限制 [伏特/秒]
    , limit(limit)              // 输出限制 [伏特]
    , error_prev(0.0f)         // 上一次误差
    , output_prev(0.0f)        // 上一次输出
    , integral_prev(0.0f)      // 上一次积分值
{
    timestamp_prev = _micros(); // 初始化时间戳
}

// PID控制器函数
float PIDController::operator() (float error){
    // 计算自上次调用以来的时间
    unsigned long timestamp_now = _micros();
    float Ts = (timestamp_now - timestamp_prev) * 1e-6f; // 时间差（秒）
    // 针对奇怪情况的快速修复（微秒溢出）
    if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

    // u(s) = (P + I/s + Ds)e(s)
    // 离散实现
    // 比例部分
    // u_p  = P * e(k)
    float proportional = P * error;
    // 积分部分的Tustin变换
    // u_ik = u_ik_1 + I*Ts/2*(ek + ek_1)
    float integral = integral_prev + I * Ts * 0.5f * (error + error_prev);
    // 反饱和 - 限制输出
    integral = _constrain(integral, -limit, limit);
    // 离散微分
    // u_dk = D(ek - ek_1)/Ts
    float derivative = D * (error - error_prev) / Ts;

    // 将所有部分相加
    float output = proportional + integral + derivative;
    // 反饱和 - 限制输出变量
    output = _constrain(output, -limit, limit);

    // 如果定义了输出变化率
    if(output_ramp > 0){
        // 通过限制输出变化率来限制加速度
        float output_rate = (output - output_prev) / Ts;
        if (output_rate > output_ramp)
            output = output_prev + output_ramp * Ts;
        else if (output_rate < -output_ramp)
            output = output_prev - output_ramp * Ts;
    }
    // 保存以备下次使用
    integral_prev = integral;
    output_prev = output;
    error_prev = error;
    timestamp_prev = timestamp_now;
    return output;
}

// 重置PID控制器
void PIDController::reset(){
    integral_prev = 0.0f; // 重置积分值
    output_prev = 0.0f;   // 重置输出
    error_prev = 0.0f;    // 重置误差
}
