#include "GenericCurrentSense.h"

// GenericCurrentSense 构造函数
GenericCurrentSense::GenericCurrentSense(PhaseCurrent_s (*readCallback)(), void (*initCallback)()){
  // 如果提供了函数，将其添加到回调中
  if(readCallback != nullptr) this->readCallback = readCallback;
  if(initCallback != nullptr) this->initCallback = initCallback;
}

// 内联传感器初始化函数
int GenericCurrentSense::init(){
    // 配置 ADC 变量
    if(initCallback != nullptr) initCallback();
    // 校准零偏移
    calibrateOffsets();
    // 设置初始化标志
    initialized = (params != SIMPLEFOC_CURRENT_SENSE_INIT_FAILED);
    // 返回成功
    return 1;
}

// 找到 ADC 的零偏移函数
void GenericCurrentSense::calibrateOffsets(){
    const int calibration_rounds = 1000;

    // 找到 ADC 偏移 = 零电流电压
    offset_ia = 0;
    offset_ib = 0;
    offset_ic = 0;
    // 读取 ADC 电压 1000 次（任意数字）
    for (int i = 0; i < calibration_rounds; i++) {
        PhaseCurrent_s current = readCallback();
        offset_ia += current.a;
        offset_ib += current.b;
        offset_ic += current.c;
        _delay(1);
    }
    // 计算平均偏移
    offset_ia = offset_ia / calibration_rounds;
    offset_ib = offset_ib / calibration_rounds;
    offset_ic = offset_ic / calibration_rounds;
}

// 读取所有三个相电流（如果可能，读取 2 或 3 个）
PhaseCurrent_s GenericCurrentSense::getPhaseCurrents(){
    PhaseCurrent_s current = readCallback();
    current.a = (current.a - offset_ia); // 安培
    current.b = (current.b - offset_ib); // 安培
    current.c = (current.c - offset_ic); // 安培
    return current;
}

// 将电流传感器与电机驱动器对齐的函数
// 如果所有引脚连接良好，实际上没有必要执行这些操作！- 可以避免
// 返回标志
// 0 - 失败
// 1 - 成功且没有更改
int GenericCurrentSense::driverAlign(float voltage, bool modulation_centered){
    _UNUSED(voltage); // 去除未使用参数的警告
    int exit_flag = 1;
    if(skip_align) return exit_flag;
    if (!initialized) return 0;
    return exit_flag;
}
