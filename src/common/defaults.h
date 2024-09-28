// 默认配置值
// 修改此文件以优化您的应用程序

#define DEF_POWER_SUPPLY 12.0f //!< 默认电源电压
// 速度 PI 控制器参数
#define DEF_PID_VEL_P 0.5f //!< 默认 PID 控制器 P 值
#define DEF_PID_VEL_I 10.0f //!< 默认 PID 控制器 I 值
#define DEF_PID_VEL_D 0.0f //!< 默认 PID 控制器 D 值
#define DEF_PID_VEL_RAMP 1000.0f //!< 默认 PID 控制器电压斜率值
#define DEF_PID_VEL_LIMIT (DEF_POWER_SUPPLY) //!< 默认 PID 控制器电压限制

// 电流感测 PID 值
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328PB__)  || defined(__AVR_ATmega2560__)
// 对于 16Mhz 控制器，如 Arduino Uno 和 Mega
#define DEF_PID_CURR_P 2 //!< 默认 PID 控制器 P 值
#define DEF_PID_CURR_I 100 //!< 默认 PID 控制器 I 值
#define DEF_PID_CURR_D 0.0f //!< 默认 PID 控制器 D 值
#define DEF_PID_CURR_RAMP 1000.0f //!< 默认 PID 控制器电压斜率值
#define DEF_PID_CURR_LIMIT (DEF_POWER_SUPPLY) //!< 默认 PID 控制器电压限制
#define DEF_CURR_FILTER_Tf 0.01f //!< 默认速度滤波器时间常数
#else
// 对于 stm32、due、teensy、esp32 和类似设备
#define DEF_PID_CURR_P 3 //!< 默认 PID 控制器 P 值
#define DEF_PID_CURR_I 300.0f //!< 默认 PID 控制器 I 值
#define DEF_PID_CURR_D 0.0f //!< 默认 PID 控制器 D 值
#define DEF_PID_CURR_RAMP 0 //!< 默认 PID 控制器电压斜率值
#define DEF_PID_CURR_LIMIT (DEF_POWER_SUPPLY) //!< 默认 PID 控制器电压限制
#define DEF_CURR_FILTER_Tf 0.005f //!< 默认电流滤波器时间常数
#endif

// 默认电流限制值
#define DEF_CURRENT_LIM 2.0f //!< 默认电流限制为 2 安培

// 默认监控下采样
#define DEF_MON_DOWNSMAPLE 100 //!< 默认监控下采样
#define DEF_MOTION_DOWNSMAPLE 0 //!< 默认运动下采样 - 禁用

// 角度 P 参数
#define DEF_P_ANGLE_P 20.0f //!< 默认 P 控制器 P 值
#define DEF_VEL_LIM 20.0f //!< 默认角速度限制

// 索引搜索
#define DEF_INDEX_SEARCH_TARGET_VELOCITY 1.0f //!< 默认索引搜索速度
// 对齐电压
#define DEF_VOLTAGE_SENSOR_ALIGN 3.0f //!< 默认传感器和电机零对齐电压
// 低通滤波器速度
#define DEF_VEL_FILTER_Tf 0.005f //!< 默认速度滤波器时间常数

// 电流感测默认参数
#define DEF_LPF_PER_PHASE_CURRENT_SENSE_Tf 0.0f //!< 默认每相电流感测低通滤波器时间常数
