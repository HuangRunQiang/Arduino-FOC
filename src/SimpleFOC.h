/*!
 * @file SimpleFOC.h
 *
 * @mainpage 简单的场定向控制 BLDC 电机控制库
 *
 * @section intro_sec 介绍
 *
 * 目前，适合低成本和低功耗的 FOC 支持板非常难以找到，甚至可能根本不存在。<br> 更难找到的是一种稳定且简单的 FOC 算法代码，能够在 Arduino 设备上运行。因此，本库的目的是：
 * - 破解 FOC 算法的神秘性，制作一个稳健而简单的 Arduino 库：Arduino SimpleFOC 库
 * - 开发一个模块化的 BLDC 驱动板：Arduino SimpleFOC 盾牌。
 *
 * @section features 特性
 *  - 兼容 Arduino：Arduino 库代码
 *  - 易于设置和配置：
 *     - 硬件配置简单
 *     - 控制回路调节简单
 *  - 模块化：
 *     - 支持尽可能多的传感器、BLDC 电机和驱动板
 *     - 支持尽可能多的应用需求
 *  - 即插即用：Arduino SimpleFOC 盾牌
 * 
 * @section dependencies 支持的硬件
 *  - 电机 
 *    - BLDC 电机
 *    - 步进电机
 * - 驱动器 
 *    - BLDC 驱动器
 *    - 云台驱动器
 *    - 步进驱动器
 * - 位置传感器 
 *    - 编码器
 *    - 磁传感器
 *    - 霍尔传感器
 *    - 开环控制
 * - 微控制器 
 *    - Arduino
 *    - STM32
 *    - ESP32
 *    - Teensy
 * 
 * @section example_code 示例代码
 * @code
#include <SimpleFOC.h>

//  BLDCMotor( 极对数 )
BLDCMotor motor = BLDCMotor(11);
//  BLDCDriver( pin_pwmA, pin_pwmB, pin_pwmC, enable (可选) )
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8);
//  Encoder(pin_A, pin_B, CPR)
Encoder encoder = Encoder(2, 3, 2048);
// 通道 A 和 B 的回调函数
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}


void setup() {  
  // 初始化编码器硬件
  encoder.init();
  // 启用硬件中断
  encoder.enableInterrupts(doA, doB);
  // 将电机与传感器连接
  motor.linkSensor(&encoder);
  
  // 电源电压 [V]
  driver.voltage_power_supply = 12;
  // 初始化驱动器硬件
  driver.init();
  // 连接驱动器
  motor.linkDriver(&driver);

  // 设置要使用的控制回路类型
  motor.controller = MotionControlType::velocity;
  // 初始化电机
  motor.init();
  
  // 对齐编码器并启动 FOC
  motor.initFOC();
}

void loop() {
  // FOC 算法函数
  motor.loopFOC();

  // 速度控制回路函数
  // 设置目标速度为 2 rad/s
  motor.move(2);
}
 * @endcode 
 *
 * @section license 许可证
 *
 * MIT 许可证，任何再分发中必须包含此处的所有文本。
 *
 */

#ifndef SIMPLEFOC_H
#define SIMPLEFOC_H

#include "BLDCMotor.h"
#include "StepperMotor.h"
#include "sensors/Encoder.h"
#include "sensors/MagneticSensorSPI.h"
#include "sensors/MagneticSensorI2C.h"
#include "sensors/MagneticSensorAnalog.h"
#include "sensors/MagneticSensorPWM.h"
#include "sensors/HallSensor.h"
#include "sensors/GenericSensor.h"
#include "drivers/BLDCDriver3PWM.h"
#include "drivers/BLDCDriver6PWM.h"
#include "drivers/StepperDriver4PWM.h"
#include "drivers/StepperDriver2PWM.h"
#include "current_sense/InlineCurrentSense.h"
#include "current_sense/LowsideCurrentSense.h"
#include "current_sense/GenericCurrentSense.h"
#include "communication/Commander.h"
#include "communication/StepDirListener.h"
#include "communication/SimpleFOCDebug.h"

#endif
