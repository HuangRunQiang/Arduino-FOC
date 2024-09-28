#ifndef FOCDRIVER_H
#define FOCDRIVER_H

#include "Arduino.h"

// 相位状态枚举
enum PhaseState : uint8_t {
  PHASE_OFF = 0, // 相位两侧均关闭
  PHASE_ON = 1,  // 相位两侧均通过PWM驱动，6-PWM模式下应用死区时间
  PHASE_HI = 2,  // 仅高侧通过PWM驱动（仅限6-PWM模式）
  PHASE_LO = 3,  // 仅低侧通过PWM驱动（仅限6-PWM模式）
};

// 驱动器类型枚举
enum DriverType{
    Unknown = 0,  // 未知类型
    BLDC = 1,     // 无刷直流电机
    Stepper = 2   // 步进电机
};

/**
 * FOC驱动器类
 */
class FOCDriver{
    public:

        /** 初始化硬件 */
        virtual int init() = 0;
        /** 启用硬件 */
        virtual void enable() = 0;
        /** 禁用硬件 */
        virtual void disable() = 0;

        long pwm_frequency; //!< PWM频率值（单位：赫兹）
        float voltage_power_supply; //!< 电源电压
        float voltage_limit; //!< 设置给电机的限制电压

        bool initialized = false; //!< 如果驱动器成功初始化则为真
        void* params = 0; //!< 指向驱动器硬件特定参数的指针

        bool enable_active_high = true; //!< 启用引脚应设置为高电平以启用驱动器（默认高电平）

        /** 获取驱动器类型 */
        virtual DriverType type() = 0;
};

#endif
