#ifndef STM32_DRIVER_MCU_DEF
#define STM32_DRIVER_MCU_DEF
#include "../../hardware_api.h"

#if defined(_STM32_DEF_)

// 默认 PWM 参数
#define _PWM_RESOLUTION 12 // 12 位
#define _PWM_RANGE 4095.0f // 2^12 - 1 = 4095
#define _PWM_FREQUENCY 25000 // 25 kHz
#define _PWM_FREQUENCY_MAX 50000 // 50 kHz

// 6PWM 参数
#define _HARDWARE_6PWM 1
#define _SOFTWARE_6PWM 0
#define _ERROR_6PWM -1

typedef struct STM32DriverParams {
  HardwareTimer* timers[6] = {NULL}; // 定义定时器数组，初始化为 NULL
  uint32_t channels[6]; // 通道数组
  long pwm_frequency; // PWM 频率
  float dead_zone; // 死区
  uint8_t interface_type; // 接口类型
} STM32DriverParams;

// 定时器同步函数
void _stopTimers(HardwareTimer **timers_to_stop, int timer_num=6); // 停止定时器
void _startTimers(HardwareTimer **timers_to_start, int timer_num=6); // 启动定时器

// 定时器查询函数
bool _getPwmState(void* params); // 获取 PWM 状态

#endif
#endif
