#include "BLDCDriver3PWM.h"

BLDCDriver3PWM::BLDCDriver3PWM(int phA, int phB, int phC, int en1, int en2, int en3){
  // 引脚初始化
  pwmA = phA;
  pwmB = phB;
  pwmC = phC;

  // 使能引脚
  enableA_pin = en1;
  enableB_pin = en2;
  enableC_pin = en3;

  // 默认电源值
  voltage_power_supply = DEF_POWER_SUPPLY;
  voltage_limit = NOT_SET;
  pwm_frequency = NOT_SET;

}

// 启用电机驱动
void BLDCDriver3PWM::enable(){
    // 启用驱动器 - 如果使能引脚可用
    if ( _isset(enableA_pin) ) digitalWrite(enableA_pin, enable_active_high);
    if ( _isset(enableB_pin) ) digitalWrite(enableB_pin, enable_active_high);
    if ( _isset(enableC_pin) ) digitalWrite(enableC_pin, enable_active_high);
    // 设置PWM为零
    setPwm(0, 0, 0);
}

// 禁用电机驱动
void BLDCDriver3PWM::disable()
{
  // 设置PWM为零
  setPwm(0, 0, 0);
  // 禁用驱动器 - 如果使能引脚可用
  if ( _isset(enableA_pin) ) digitalWrite(enableA_pin, !enable_active_high);
  if ( _isset(enableB_pin) ) digitalWrite(enableB_pin, !enable_active_high);
  if ( _isset(enableC_pin) ) digitalWrite(enableC_pin, !enable_active_high);

}

// 初始化硬件引脚
int BLDCDriver3PWM::init() {
  // PWM引脚
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(pwmC, OUTPUT);
  if( _isset(enableA_pin)) pinMode(enableA_pin, OUTPUT);
  if( _isset(enableB_pin)) pinMode(enableB_pin, OUTPUT);
  if( _isset(enableC_pin)) pinMode(enableC_pin, OUTPUT);

  // 对电压限制配置进行合理性检查
  if(!_isset(voltage_limit) || voltage_limit > voltage_power_supply) voltage_limit = voltage_power_supply;

  // 设置引脚的PWM频率
  // 硬件特定函数 - 取决于驱动器和微控制器
  params = _configure3PWM(pwm_frequency, pwmA, pwmB, pwmC);
  initialized = (params != SIMPLEFOC_DRIVER_INIT_FAILED);
  return params != SIMPLEFOC_DRIVER_INIT_FAILED;
}

// 设置相位状态
void BLDCDriver3PWM::setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) {
  // 如果需要，先禁用
  if( _isset(enableA_pin) && _isset(enableB_pin) && _isset(enableC_pin) ){
    digitalWrite(enableA_pin, sa == PhaseState::PHASE_ON ? enable_active_high : !enable_active_high);
    digitalWrite(enableB_pin, sb == PhaseState::PHASE_ON ? enable_active_high : !enable_active_high);
    digitalWrite(enableC_pin, sc == PhaseState::PHASE_ON ? enable_active_high : !enable_active_high);
  }
}

// 设置PWM引脚的电压
void BLDCDriver3PWM::setPwm(float Ua, float Ub, float Uc) {

  // 限制驱动器中的电压
  Ua = _constrain(Ua, 0.0f, voltage_limit);
  Ub = _constrain(Ub, 0.0f, voltage_limit);
  Uc = _constrain(Uc, 0.0f, voltage_limit);
  
  // 计算占空比
  // 限制在[0,1]范围内
  dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
  dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
  dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

  // 硬件特定写入
  // 硬件特定函数 - 取决于驱动器和微控制器
  _writeDutyCycle3PWM(dc_a, dc_b, dc_c, params);
}
