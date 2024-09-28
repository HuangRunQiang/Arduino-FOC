#include "LowsideCurrentSense.h"
#include "communication/SimpleFOCDebug.h"

// LowsideCurrentSensor 构造函数
//  - shunt_resistor  - shunt 电阻值
//  - gain  - 电流感应运算放大器增益
//  - phA   - A 相 ADC 引脚
//  - phB   - B 相 ADC 引脚
//  - phC   - C 相 ADC 引脚（可选）
LowsideCurrentSense::LowsideCurrentSense(float _shunt_resistor, float _gain, int _pinA, int _pinB, int _pinC){
    pinA = _pinA;
    pinB = _pinB;
    pinC = _pinC;

    shunt_resistor = _shunt_resistor;
    amp_gain  = _gain;
    volts_to_amps_ratio = 1.0f / _shunt_resistor / _gain; // 伏特转安培
    // 每个相的增益
    gain_a = volts_to_amps_ratio;
    gain_b = volts_to_amps_ratio;
    gain_c = volts_to_amps_ratio;
}

LowsideCurrentSense::LowsideCurrentSense(float _mVpA, int _pinA, int _pinB, int _pinC){
    pinA = _pinA;
    pinB = _pinB;
    pinC = _pinC;

    volts_to_amps_ratio = 1000.0f / _mVpA; // 毫伏转安培
    // 每个相的增益
    gain_a = volts_to_amps_ratio;
    gain_b = volts_to_amps_ratio;
    gain_c = volts_to_amps_ratio;
}   

// Lowside 传感器初始化函数
int LowsideCurrentSense::init(){

    if (driver == nullptr) {
        SIMPLEFOC_DEBUG("CUR: 驱动器未链接!");
        return 0;
    }

    // 配置 ADC 变量
    params = _configureADCLowSide(driver->params, pinA, pinB, pinC);
    // 如果初始化失败，返回失败
    if (params == SIMPLEFOC_CURRENT_SENSE_INIT_FAILED) return 0; 
    // 同步驱动器
    void* r = _driverSyncLowSide(driver->params, params);
    if (r == SIMPLEFOC_CURRENT_SENSE_INIT_FAILED) return 0; 
    // 设置中心 PWM（0 电压矢量）
    if (driver_type == DriverType::BLDC)
        static_cast<BLDCDriver*>(driver)->setPwm(driver->voltage_limit / 2, driver->voltage_limit / 2, driver->voltage_limit / 2);
    // 校准零偏差
    calibrateOffsets();
    // 将所有相的零电压设置为零
    if (driver_type == DriverType::BLDC)
        static_cast<BLDCDriver*>(driver)->setPwm(0, 0, 0);
    // 设置初始化标志
    initialized = (params != SIMPLEFOC_CURRENT_SENSE_INIT_FAILED);
    // 返回成功
    return 1;
}

// 查找 ADC 的零偏差的函数
void LowsideCurrentSense::calibrateOffsets(){    
    const int calibration_rounds = 2000;

    // 查找 ADC 偏差 = 零电流电压
    offset_ia = 0;
    offset_ib = 0;
    offset_ic = 0;
    // 读取 ADC 电压 1000 次（任意数字）
    for (int i = 0; i < calibration_rounds; i++) {
        _startADC3PinConversionLowSide();
        if (_isset(pinA)) offset_ia += (_readADCVoltageLowSide(pinA, params));
        if (_isset(pinB)) offset_ib += (_readADCVoltageLowSide(pinB, params));
        if (_isset(pinC)) offset_ic += (_readADCVoltageLowSide(pinC, params));
        _delay(1);
    }
    // 计算平均偏差
    if (_isset(pinA)) offset_ia = offset_ia / calibration_rounds;
    if (_isset(pinB)) offset_ib = offset_ib / calibration_rounds;
    if (_isset(pinC)) offset_ic = offset_ic / calibration_rounds;
}

// 读取所有三个相电流（如果可能，读取 2 或 3 个）
PhaseCurrent_s LowsideCurrentSense::getPhaseCurrents(){
    PhaseCurrent_s current;
    _startADC3PinConversionLowSide();
    current.a = (!_isset(pinA)) ? 0 : (_readADCVoltageLowSide(pinA, params) - offset_ia) * gain_a; // 安培
    current.b = (!_isset(pinB)) ? 0 : (_readADCVoltageLowSide(pinB, params) - offset_ib) * gain_b; // 安培
    current.c = (!_isset(pinC)) ? 0 : (_readADCVoltageLowSide(pinC, params) - offset_ic) * gain_c; // 安培
    return current;
}
