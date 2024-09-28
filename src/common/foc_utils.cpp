#include "foc_utils.h"

// 使用固定大小数组近似计算正弦值的函数
// 使用65个元素的查找表和插值
// 感谢@dekutree对优化工作的贡献
__attribute__((weak)) float _sin(float a) {
    // 用于正弦查找的16位整数数组。使用插值以提高精度
    // 正弦值的16位精度，8位小数值用于插值，6位查找表大小
    // 与标准库正弦函数相比，结果精度为0.00006480（在-PI到PI范围内3217步的均方根差异）
    static uint16_t sine_array[65] = {
        0, 804, 1608, 2411, 3212, 4011, 4808, 5602, 6393, 7180,
        7962, 8740, 9512, 10279, 11039, 11793, 12540, 13279, 14010,
        14733, 15447, 16151, 16846, 17531, 18205, 18868, 19520,
        20160, 20788, 21403, 22006, 22595, 23170, 23732, 24279,
        24812, 25330, 25833, 26320, 26791, 27246, 27684, 28106,
        28511, 28899, 29269, 29622, 29957, 30274, 30572, 30853,
        31114, 31357, 31581, 31786, 31972, 32138, 32286, 32413,
        32522, 32610, 32679, 32729, 32758, 32768
    };
    
    int32_t t1, t2;
    unsigned int i = (unsigned int)(a * (64 * 4 * 256.0f / _2PI));
    int frac = i & 0xff;
    i = (i >> 8) & 0xff;

    if (i < 64) {
        t1 = (int32_t)sine_array[i];
        t2 = (int32_t)sine_array[i + 1];
    } else if (i < 128) {
        t1 = (int32_t)sine_array[128 - i];
        t2 = (int32_t)sine_array[127 - i];
    } else if (i < 192) {
        t1 = -(int32_t)sine_array[-128 + i];
        t2 = -(int32_t)sine_array[-127 + i];
    } else {
        t1 = -(int32_t)sine_array[256 - i];
        t2 = -(int32_t)sine_array[255 - i];
    }
    return (1.0f / 32768.0f) * (t1 + (((t2 - t1) * frac) >> 8));
}

// 使用固定大小数组近似计算余弦值的函数
// ~55微秒（浮点数组）
// ~56微秒（整数数组）
// 精度±0.005
// 接收的角度必须在0到2PI之间
__attribute__((weak)) float _cos(float a) {
    float a_sin = a + _PI_2;
    a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
    return _sin(a_sin);
}

__attribute__((weak)) void _sincos(float a, float* s, float* c) {
    *s = _sin(a);
    *c = _cos(a);
}

// 基于https://math.stackexchange.com/a/1105038/81278的fast_atan2
// 来自Odive项目
// https://github.com/odriverobotics/ODrive/blob/master/Firmware/MotorControl/utils.cpp
// 此函数采用MIT许可证，版权归Oskar Weigl/Odrive Robotics所有
// Odrive的atan2源代码为公共领域。感谢Odive使其易于借用。
__attribute__((weak)) float _atan2(float y, float x) {
    // a := min (|x|, |y|) / max (|x|, |y|)
    float abs_y = fabsf(y);
    float abs_x = fabsf(x);
    // 在分母中注入FLT_MIN以避免除以零
    float a = min(abs_x, abs_y) / (max(abs_x, abs_y));
    // s := a * a
    float s = a * a;
    // r := ((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a
    float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
    // 如果|y| > |x|，则r := 1.57079637 - r
    if (abs_y > abs_x) r = 1.57079637f - r;
    // 如果x < 0，则r := 3.14159274 - r
    if (x < 0.0f) r = 3.14159274f - r;
    // 如果y < 0，则r := -r
    if (y < 0.0f) r = -r;

    return r;
}

// 将弧度角规范化到[0,2PI]
__attribute__((weak)) float _normalizeAngle(float angle) {
    float a = fmod(angle, _2PI);
    return a >= 0 ? a : (a + _2PI);
}

// 电气角度计算
float _electricalAngle(float shaft_angle, int pole_pairs) {
    return (shaft_angle * pole_pairs);
}

// 使用平方根近似函数
// https://reprap.org/forum/read.php?147,219210
// https://en.wikipedia.org/wiki/Fast_inverse_square_root
__attribute__((weak)) float _sqrtApprox(float number) { // 低脂肪
    union {
        float f;
        uint32_t i;
    } y = { .f = number };
    y.i = 0x5f375a86 - (y.i >> 1);
    return number * y.f;
}
