#ifndef HARDWARE_UTILS_CURRENT_H
#define HARDWARE_UTILS_CURRENT_H

#include "../common/foc_utils.h"
#include "../common/time_utils.h"

// 如果电流传感器初始化失败，返回的标志
#define SIMPLEFOC_CURRENT_SENSE_INIT_FAILED ((void*)-1)

// 硬件特定结构的通用实现
// 包含所有必要的电流传感参数
// 将作为 void 指针从 _configureADCx 函数返回
// 将作为 void 指针提供给 _readADCVoltageX() 
typedef struct GenericCurrentSenseParams {
  int pins[3]; // 引脚数组
  float adc_voltage_conv; // ADC 电压转换因子
} GenericCurrentSenseParams;


/**
 *  读取 ADC 值并返回读取的电压
 *
 * @param pinA - 要读取的 Arduino 引脚（必须是 ADC 引脚）
 * @param cs_params - 电流传感参数结构 - 硬件特定
 */
float _readADCVoltageInline(const int pinA, const void* cs_params);

/**
 *  读取 ADC 值并返回读取的电压
 *
 * @param driver_params - 驱动参数结构 - 硬件特定
 * @param pinA - ADC 引脚 A
 * @param pinB - ADC 引脚 B
 * @param pinC - ADC 引脚 C
 */
void* _configureADCInline(const void *driver_params, const int pinA, const int pinB, const int pinC = NOT_SET);

/**
 *  读取 ADC 值并返回读取的电压
 *
 * @param driver_params - 驱动参数结构 - 硬件特定
 * @param pinA - ADC 引脚 A
 * @param pinB - ADC 引脚 B
 * @param pinC - ADC 引脚 C
 */
void* _configureADCLowSide(const void *driver_params, const int pinA, const int pinB, const int pinC = NOT_SET);

void _startADC3PinConversionLowSide();

/**
 *  读取 ADC 值并返回读取的电压
 *
 * @param pinA - 要读取的 Arduino 引脚（必须是 ADC 引脚）
 * @param cs_params - 电流传感参数结构 - 硬件特定
 */
float _readADCVoltageLowSide(const int pinA, const void* cs_params);

/**
 *  将驱动程序与 ADC 同步以进行低侧检测
 * @param driver_params - 驱动参数结构 - 硬件特定
 * @param cs_params - 电流传感参数结构 - 硬件特定
 * 
 * @return void* - 返回指向电流传感参数结构的指针（未更改）
 *        - 如果初始化失败，则返回 SIMPLEFOC_CURRENT_SENSE_INIT_FAILED
 */
void* _driverSyncLowSide(void* driver_params, void* cs_params);

#endif
