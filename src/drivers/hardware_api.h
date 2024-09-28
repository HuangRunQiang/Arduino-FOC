#ifndef HARDWARE_UTILS_DRIVER_H
#define HARDWARE_UTILS_DRIVER_H

#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "../communication/SimpleFOCDebug.h"
#include "../common/base_classes/BLDCDriver.h"

// 这些定义决定了 PWM 输出的极性。通常情况下，极性为高电平有效，
// 即期望高电平 PWM 输出能够开启 MOSFET。如果您的驱动设计需要反向极性，
// 您可以更改下面的定义，或通过构建环境或板定义文件进行设置。

// 用于 1-PWM、2-PWM、3-PWM 和 4-PWM 模式
#ifndef SIMPLEFOC_PWM_ACTIVE_HIGH
#define SIMPLEFOC_PWM_ACTIVE_HIGH true
#endif
// 用于 6-PWM 模式，高侧
#ifndef SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH
#define SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH true
#endif
// 用于 6-PWM 模式，低侧
#ifndef SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH
#define SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH true
#endif

// 如果驱动初始化失败，返回的标志
#define SIMPLEFOC_DRIVER_INIT_FAILED ((void*)-1)

// 硬件特定结构的通用实现
// 包含所有必要的驱动参数
// 将作为 void 指针从 _configurexPWM 函数返回
// 将作为 void 指针提供给 _writeDutyCyclexPWM() 
typedef struct GenericDriverParams {
  int pins[6]; // 引脚数组
  long pwm_frequency; // PWM 频率
  float dead_zone; // 死区
} GenericDriverParams;

/** 
 * 配置 PWM 频率、分辨率和对齐
 * - 步进电机驱动 - 2PWM 设置
 * - 硬件特定
 * 
 * @param pwm_frequency - 频率（赫兹） - 如果适用
 * @param pinA - pinA PWM 引脚
 * 
 * @return -1 如果失败，或指向内部驱动参数结构的指针如果成功
 */
void* _configure1PWM(long pwm_frequency, const int pinA);

/** 
 * 配置 PWM 频率、分辨率和对齐
 * - 步进电机驱动 - 2PWM 设置
 * - 硬件特定
 * 
 * @param pwm_frequency - 频率（赫兹） - 如果适用
 * @param pinA - pinA BLDC 驱动
 * @param pinB - pinB BLDC 驱动
 * 
 * @return -1 如果失败，或指向内部驱动参数结构的指针如果成功
 */
void* _configure2PWM(long pwm_frequency, const int pinA, const int pinB);

/** 
 * 配置 PWM 频率、分辨率和对齐
 * - BLDC 驱动 - 3PWM 设置
 * - 硬件特定
 * 
 * @param pwm_frequency - 频率（赫兹） - 如果适用
 * @param pinA - pinA BLDC 驱动
 * @param pinB - pinB BLDC 驱动
 * @param pinC - pinC BLDC 驱动
 * 
 * @return -1 如果失败，或指向内部驱动参数结构的指针如果成功
 */
void* _configure3PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC);

/** 
 * 配置 PWM 频率、分辨率和对齐
 * - 步进电机驱动 - 4PWM 设置
 * - 硬件特定
 * 
 * @param pwm_frequency - 频率（赫兹） - 如果适用
 * @param pin1A - pin1A 步进电机驱动
 * @param pin1B - pin1B 步进电机驱动
 * @param pin2A - pin2A 步进电机驱动
 * @param pin2B - pin2B 步进电机驱动
 * 
 * @return -1 如果失败，或指向内部驱动参数结构的指针如果成功
 */
void* _configure4PWM(long pwm_frequency, const int pin1A, const int pin1B, const int pin2A, const int pin2B);

/** 
 * 配置 PWM 频率、分辨率和对齐
 * - BLDC 驱动 - 6PWM 设置
 * - 硬件特定
 * 
 * @param pwm_frequency - 频率（赫兹） - 如果适用
 * @param dead_zone - 占空比保护区 [0, 1] - 低侧和高侧均低 - 如果适用
 * @param pinA_h - pinA 高侧 BLDC 驱动 
 * @param pinA_l - pinA 低侧 BLDC 驱动 
 * @param pinB_h - pinB 高侧 BLDC 驱动 
 * @param pinB_l - pinB 低侧 BLDC 驱动 
 * @param pinC_h - pinC 高侧 BLDC 驱动 
 * @param pinC_l - pinC 低侧 BLDC 驱动 
 * 
 * @return -1 如果失败，或指向内部驱动参数结构的指针如果成功
 */
void* _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l);

/** 
 * 设置 PWM 引脚的占空比（例如，analogWrite()）
 * - 步进电机驱动 - 2PWM 设置
 * - 硬件特定
 * 
 * @param dc_a - 占空比相 A [0, 1]
 * @param params - 驱动参数
 */ 
void _writeDutyCycle1PWM(float dc_a, void* params);

/** 
 * 设置 PWM 引脚的占空比（例如，analogWrite()）
 * - 步进电机驱动 - 2PWM 设置
 * - 硬件特定
 * 
 * @param dc_a - 占空比相 A [0, 1]
 * @param dc_b - 占空比相 B [0, 1]
 * @param params - 驱动参数
 */ 
void _writeDutyCycle2PWM(float dc_a,  float dc_b, void* params);

/** 
 * 设置 PWM 引脚的占空比（例如，analogWrite()）
 * - BLDC 驱动 - 3PWM 设置
 * - 硬件特定
 * 
 * @param dc_a - 占空比相 A [0, 1]
 * @param dc_b - 占空比相 B [0, 1]
 * @param dc_c - 占空比相 C [0, 1]
 * @param params - 驱动参数
 */ 
void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, void* params);

/** 
 * 设置 PWM 引脚的占空比（例如，analogWrite()）
 * - 步进电机驱动 - 4PWM 设置
 * - 硬件特定
 * 
 * @param dc_1a - 占空比相 1A [0, 1]
 * @param dc_1b - 占空比相 1B [0, 1]
 * @param dc_2a - 占空比相 2A [0, 1]
 * @param dc_2b - 占空比相 2B [0, 1]
 * @param params - 驱动参数
 */ 
void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, void* params);

/** 
 * 设置 PWM 引脚的占空比（例如，analogWrite()）
 * - BLDC 驱动 - 6PWM 设置
 * - 硬件特定
 * 
 * @param dc_a - 占空比相 A [0, 1]
 * @param dc_b - 占空比相 B [0, 1]
 * @param dc_c - 占空比相 C [0, 1]
 * @param phase_state - 指向 PhaseState[3] 数组的指针
 * @param params - 驱动参数
 */ 
void _writeDutyCycle6PWM(float dc_a,  float dc_b, float dc_c, PhaseState *phase_state, void* params);

#endif
