#ifndef FOCUTILS_LIB_H
#define FOCUTILS_LIB_H

#include "Arduino.h"

// 符号函数
#define _sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
#ifndef _round
#define _round(x) ((x)>=0?(long)((x)+0.5f):(long)((x)-0.5f))
#endif
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _sqrt(a) (_sqrtApprox(a))
#define _isset(a) ( (a) != (NOT_SET) )
#define _UNUSED(v) (void) (v)
#define _powtwo(x) (1 << (x))

#define _swap(a, b) { auto temp = a; a = b; b = temp; }

// 工具定义
#define _2_SQRT3 1.15470053838f
#define _SQRT3 1.73205080757f
#define _1_SQRT3 0.57735026919f
#define _SQRT3_2 0.86602540378f
#define _SQRT2 1.41421356237f
#define _120_D2R 2.09439510239f
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _2PI 6.28318530718f
#define _3PI_2 4.71238898038f
#define _PI_6 0.52359877559f
#define _RPM_TO_RADS 0.10471975512f

#define NOT_SET -12345.0f
#define _HIGH_IMPEDANCE 0
#define _HIGH_Z _HIGH_IMPEDANCE
#define _ACTIVE 1
#define _NC ((int) NOT_SET)

#define MIN_ANGLE_DETECT_MOVEMENT (_2PI/101.0f)

// dq电流结构
struct DQCurrent_s
{
    float d;
    float q;
};
// 相电流结构
struct PhaseCurrent_s
{
    float a;
    float b;
    float c;
};
// dq电压结构
struct DQVoltage_s
{
    float d;
    float q;
};
// alpha beta电流结构
struct ABCurrent_s
{
    float alpha;
    float beta;
};


/**
 *  使用固定大小数组近似计算正弦值的函数
 * - 执行时间 ~40微秒 (Arduino UNO)
 *
 * @param a 角度在0和2PI之间
 */
float _sin(float a);
/**
 * 使用固定大小数组近似计算余弦值的函数
 * - 执行时间 ~50微秒 (Arduino UNO)
 *
 * @param a 角度在0和2PI之间
 */
float _cos(float a);
/**
 * 返回角度的正弦值和余弦值的函数。
 * 内部使用 _sin 和 _cos 函数，但您可以提供
 * 更优化的自定义实现。
 */
void _sincos(float a, float* s, float* c);

/**
 * 近似计算 atan2 的函数
 * 
 */
float _atan2(float y, float x);

/**
 * 将弧度角标准化到 [0,2PI]
 * @param angle - 要标准化的角度
 */
float _normalizeAngle(float angle);


/**
 * 电气角度计算
 *
 * @param shaft_angle - 电机的轴角
 * @param pole_pairs - 极对数
 */
float _electricalAngle(float shaft_angle, int pole_pairs);

/**
 * 近似计算平方根的函数
 *  - 使用快速逆平方根
 *
 * @param value - 数字
 */
float _sqrtApprox(float value);

#endif
