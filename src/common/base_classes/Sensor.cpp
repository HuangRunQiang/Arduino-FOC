#include "Sensor.h"
#include "../foc_utils.h"
#include "../time_utils.h"

void Sensor::update() {
    float val = getSensorAngle();
    if (val < 0) // 传感器角度严格为非负。负值用于表示错误。
        return; // TODO 信号错误，例如通过标志和计数器
    angle_prev_ts = _micros();
    float d_angle = val - angle_prev;
    // 如果发生溢出，将其视为完整旋转
    if (abs(d_angle) > (0.8f * _2PI)) full_rotations += (d_angle > 0) ? -1 : 1; 
    angle_prev = val;
}

/** 获取当前角速度（弧度/秒） */
float Sensor::getVelocity() {
    // 计算采样时间
    float Ts = (angle_prev_ts - vel_angle_prev_ts) * 1e-6f;
    if (Ts < 0.0f) { // 处理 micros() 溢出 - 我们需要重置 vel_angle_prev_ts
        vel_angle_prev = angle_prev;
        vel_full_rotations = full_rotations;
        vel_angle_prev_ts = angle_prev_ts;
        return velocity;
    }
    if (Ts < min_elapsed_time) return velocity; // 如果 deltaT 太小，则不更新速度

    velocity = ((float)(full_rotations - vel_full_rotations) * _2PI + (angle_prev - vel_angle_prev)) / Ts;
    vel_angle_prev = angle_prev;
    vel_full_rotations = full_rotations;
    vel_angle_prev_ts = angle_prev_ts;
    return velocity;
}

void Sensor::init() {
    // 初始化传感器的所有内部变量，以确保“平滑”启动（没有从零跳跃）
    getSensorAngle(); // 调用一次
    delayMicroseconds(1);
    vel_angle_prev = getSensorAngle(); // 再次调用
    vel_angle_prev_ts = _micros();
    delay(1);
    getSensorAngle(); // 调用一次
    delayMicroseconds(1);
    angle_prev = getSensorAngle(); // 再次调用
    angle_prev_ts = _micros();
}

float Sensor::getMechanicalAngle() {
    return angle_prev;
}

float Sensor::getAngle() {
    return (float)full_rotations * _2PI + angle_prev;
}

double Sensor::getPreciseAngle() {
    return (double)full_rotations * (double)_2PI + (double)angle_prev;
}

int32_t Sensor::getFullRotations() {
    return full_rotations;
}

int Sensor::needsSearch() {
    return 0; // 默认返回 false
}
