#include "InlineCurrentSense.h"
#include "communication/SimpleFOCDebug.h"

// InlineCurrentSensor 构造函数
//  - shunt_resistor  - 分流电阻值
//  - gain  - 电流感应运算放大器增益
//  - phA   - A 相 ADC 引脚
//  - phB   - B 相 ADC 引脚
//  - phC   - C 相 ADC 引脚（可选）
InlineCurrentSense::InlineCurrentSense(float _shunt_resistor, float _gain, int _pinA, int _pinB, int _pinC) {
    pinA = _pinA;
    pinB = _pinB;
    pinC = _pinC;

    shunt_resistor = _shunt_resistor;
    amp_gain  = _gain;
    volts_to_amps_ratio = 1.0f / _shunt_resistor / _gain; // 伏特转安培
    // 每个相位的增益
    gain_a = volts_to_amps_ratio;
    gain_b = volts_to_amps_ratio;
    gain_c = volts_to_amps_ratio;
};

InlineCurrentSense::InlineCurrentSense(float _mVpA, int _pinA, int _pinB, int _pinC) {
    pinA = _pinA;
    pinB = _pinB;
    pinC = _pinC;

    volts_to_amps_ratio = 1000.0f / _mVpA; // 毫伏转安培
    // 每个相位的增益
    gain_a = volts_to_amps_ratio;
    gain_b = volts_to_amps_ratio;
    gain_c = volts_to_amps_ratio;
};

// Inline 传感器初始化函数
int InlineCurrentSense::init() {
    // 如果没有连接的驱动器，在这种情况下是可以的
    // 至少对于初始化（init()）是可以的
    void* drv_params = driver ? driver->params : nullptr;
    // 配置 ADC 变量
    params = _configureADCInline(drv_params, pinA, pinB, pinC);
    // 如果初始化失败，返回失败
    if (params == SIMPLEFOC_CURRENT_SENSE_INIT_FAILED) return 0; 
    // 设置中心 PWM（0 电压矢量）
    if (driver_type == DriverType::BLDC)
        static_cast<BLDCDriver*>(driver)->setPwm(driver->voltage_limit / 2, driver->voltage_limit / 2, driver->voltage_limit / 2);
    // 校准零偏移
    calibrateOffsets();
    // 将所有相位的零电压设置为0
    if (driver_type == DriverType::BLDC)
        static_cast<BLDCDriver*>(driver)->setPwm(0, 0, 0);
    // 设置初始化标志
    initialized = (params != SIMPLEFOC_CURRENT_SENSE_INIT_FAILED);
    // 返回成功
    return 1;
}

// 查找 ADC 零偏移的函数
void InlineCurrentSense::calibrateOffsets() {
    const int calibration_rounds = 1000;

    // 查找 ADC 偏移 = 零电流电压
    offset_ia = 0;
    offset_ib = 0;
    offset_ic = 0;
    // 读取 ADC 电压 1000 次（任意数字）
    for (int i = 0; i < calibration_rounds; i++) {
        if (_isset(pinA)) offset_ia += _readADCVoltageInline(pinA, params);
        if (_isset(pinB)) offset_ib += _readADCVoltageInline(pinB, params);
        if (_isset(pinC)) offset_ic += _readADCVoltageInline(pinC, params);
        _delay(1);
    }
    // 计算平均偏移
    if (_isset(pinA)) offset_ia = offset_ia / calibration_rounds;
    if (_isset(pinB)) offset_ib = offset_ib / calibration_rounds;
    if (_isset(pinC)) offset_ic = offset_ic / calibration_rounds;
}

// 读取所有三个相位电流（如果可能，读取2或3个）
PhaseCurrent_s InlineCurrentSense::getPhaseCurrents() {
    PhaseCurrent_s current;
    current.a = (!_isset(pinA)) ? 0 : (_readADCVoltageInline(pinA, params) - offset_ia) * gain_a; // 安培
    current.b = (!_isset(pinB)) ? 0 : (_readADCVoltageInline(pinB, params) - offset_ib) * gain_b; // 安培
    current.c = (!_isset(pinC)) ? 0 : (_readADCVoltageInline(pinC, params) - offset_ic) * gain_c; // 安培
    return current;
}
