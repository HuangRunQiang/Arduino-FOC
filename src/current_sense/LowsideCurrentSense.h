#ifndef LOWSIDE_CS_LIB_H
#define LOWSIDE_CS_LIB_H

#include "Arduino.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "../common/defaults.h"
#include "../common/base_classes/CurrentSense.h"
#include "../common/base_classes/FOCMotor.h"
#include "../common/base_classes/StepperDriver.h"
#include "../common/base_classes/BLDCDriver.h"
#include "../common/lowpass_filter.h"
#include "hardware_api.h"


class LowsideCurrentSense: public CurrentSense {
  public:
    /**
      LowsideCurrentSense 类构造函数
      @param shunt_resistor 分流电阻值
      @param gain 电流感应运算放大器增益
      @param phA A 相 ADC 引脚
      @param phB B 相 ADC 引脚
      @param phC C 相 ADC 引脚（可选）
    */
    LowsideCurrentSense(float shunt_resistor, float gain, int pinA, int pinB, int pinC = _NC);
    /**
      LowsideCurrentSense 类构造函数
      @param mVpA 毫伏每安培比率
      @param phA A 相 ADC 引脚
      @param phB B 相 ADC 引脚
      @param phC C 相 ADC 引脚（可选）
    */
    LowsideCurrentSense(float mVpA, int pinA, int pinB, int pinC = _NC);

    // 实现 CurrentSense 接口的函数
    int init() override;
    PhaseCurrent_s getPhaseCurrents() override;

  private:

    // 增益变量
    float shunt_resistor; //!< 分流电阻值
    float amp_gain; //!< 安培增益值
    float volts_to_amps_ratio; //!< 伏特到安培的比率

    /**
     *  计算 ADC 的零偏移量
     */
    void calibrateOffsets();

};

#endif
