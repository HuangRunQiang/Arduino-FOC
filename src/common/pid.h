#ifndef PID_H
#define PID_H

#include "time_utils.h"
#include "foc_utils.h"

/**
 *  PID控制器类
 */
class PIDController
{
public:
    /**
     * 构造函数
     * @param P - 比例增益 
     * @param I - 积分增益
     * @param D - 微分增益 
     * @param ramp - 输出值的最大变化速度
     * @param limit - 最大输出值
     */
    PIDController(float P, float I, float D, float ramp, float limit);
    ~PIDController() = default; // 默认析构函数

    float operator() (float error); // 重载操作符
    void reset(); // 重置函数

    float P; //!< 比例增益 
    float I; //!< 积分增益 
    float D; //!< 微分增益 
    float output_ramp; //!< 输出值的最大变化速度
    float limit; //!< 最大输出值

protected:
    float error_prev; //!< 上一次跟踪误差值
    float output_prev; //!< 上一次PID输出值
    float integral_prev; //!< 上一次积分分量值
    unsigned long timestamp_prev; //!< 上一次执行的时间戳
};

#endif // PID_H
