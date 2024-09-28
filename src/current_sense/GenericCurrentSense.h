#ifndef GENERIC_CS_LIB_H
#define GENERIC_CS_LIB_H

#include "Arduino.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "../common/defaults.h"
#include "../common/base_classes/CurrentSense.h"
#include "../common/lowpass_filter.h"
#include "hardware_api.h"

class GenericCurrentSense: public CurrentSense {
  public:
    /**
      GenericCurrentSense 类构造函数
    */
    GenericCurrentSense(PhaseCurrent_s (*readCallback)() = nullptr, void (*initCallback)() = nullptr);

    // 实现 CurrentSense 接口的函数
    int init() override;  // 初始化
    PhaseCurrent_s getPhaseCurrents() override;  // 获取相电流
    int driverAlign(float align_voltage, bool modulation_centered) override;  // 驱动对齐

    PhaseCurrent_s (*readCallback)() = nullptr; //!< 指向传感器读取的函数指针
    void (*initCallback)() = nullptr; //!< 指向传感器初始化的函数指针

  private:
    /**
     *  查找 ADC 的零偏移量的函数
     */
    void calibrateOffsets();  // 校准偏移量
    float offset_ia; //!< 零电流 A 电压值（ADC 读取的中心值）
    float offset_ib; //!< 零电流 B 电压值（ADC 读取的中心值）
    float offset_ic; //!< 零电流 C 电压值（ADC 读取的中心值）

};

#endif
