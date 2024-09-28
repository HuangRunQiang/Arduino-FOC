#ifndef INLINE_CS_LIB_H
#define INLINE_CS_LIB_H

#include "Arduino.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "../common/defaults.h"
#include "../common/base_classes/CurrentSense.h"
#include "../common/base_classes/StepperDriver.h"
#include "../common/base_classes/BLDCDriver.h"
#include "../common/lowpass_filter.h"
#include "hardware_api.h"

class InlineCurrentSense: public CurrentSense {
  public:
    /**
      InlineCurrentSense 类构造函数
      @param shunt_resistor 旁路电阻值
      @param gain 电流感应运算放大器增益
      @param phA A相 ADC 引脚
      @param phB B相 ADC 引脚
      @param phC C相 ADC 引脚（可选）
    */
    InlineCurrentSense(float shunt_resistor, float gain, int pinA, int pinB, int pinC = NOT_SET);
    
    /**
      InlineCurrentSense 类构造函数
      @param mVpA 每安培毫伏比
      @param phA A相 ADC 引脚
      @param phB B相 ADC 引脚
      @param phC C相 ADC 引脚（可选）
    */
    InlineCurrentSense(float mVpA, int pinA, int pinB, int pinC = NOT_SET);

    // 实现 CurrentSense 接口的函数 
    int init() override;
    PhaseCurrent_s getPhaseCurrents() override;

  private:
  
    // 增益变量
    float shunt_resistor; //!< 旁路电阻值
    float amp_gain; //!< 放大增益值
    float volts_to_amps_ratio; //!< 伏特到安培的比率
    
    /**
     *  查找 ADC 零偏移的函数
     */
    void calibrateOffsets();
};

#endif
