#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#include "time_utils.h"
#include "foc_utils.h"

/**
 *  低通滤波器类
 */
class LowPassFilter
{
public:
    /**
     * @param Tf - 低通滤波器的时间常数
     */
    LowPassFilter(float Tf);  // 构造函数
    ~LowPassFilter() = default;  // 默认析构函数

    float operator() (float x);  // 重载运算符，用于滤波
    float Tf; //!< 低通滤波器的时间常数

protected:
    unsigned long timestamp_prev;  //!< 上一次执行的时间戳
    float y_prev; //!< 上一次执行步骤中的滤波值 
};

#endif // LOWPASS_FILTER_H
