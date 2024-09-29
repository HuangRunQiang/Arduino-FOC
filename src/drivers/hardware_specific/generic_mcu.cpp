#include "../hardware_api.h"

// 如果 MCU 没有定义 analogWrite
#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && !defined(analogWrite)
  __attribute__((weak)) void analogWrite(uint8_t pin, int value){ };
#endif

// 设置高 PWM 频率到指定引脚的函数
// - 步进电机 - 1PWM 设置
// - 硬件特定
// 在通用情况下不执行任何操作
__attribute__((weak)) void* _configure1PWM(long pwm_frequency, const int pinA) {
  GenericDriverParams* params = new GenericDriverParams {
    .pins = { pinA },
    .pwm_frequency = pwm_frequency
  };
  return params;
}

// 设置高 PWM 频率到指定引脚的函数
// - 步进电机 - 2PWM 设置
// - 硬件特定
// 在通用情况下不执行任何操作
__attribute__((weak)) void* _configure2PWM(long pwm_frequency,const int pinA, const int pinB) {
  GenericDriverParams* params = new GenericDriverParams {
    .pins = { pinA, pinB },
    .pwm_frequency = pwm_frequency
  };
  return params;
}

// 设置高 PWM 频率到指定引脚的函数
// - 无刷直流电机 - 3PWM 设置
// - 硬件特定
// 在通用情况下不执行任何操作
__attribute__((weak)) void* _configure3PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC) {
  GenericDriverParams* params = new GenericDriverParams {
    .pins = { pinA, pinB, pinC },
    .pwm_frequency = pwm_frequency
  };
  return params;
}

// 设置高 PWM 频率到指定引脚的函数
// - 步进电机 - 4PWM 设置
// - 硬件特定
// 在通用情况下不执行任何操作
__attribute__((weak)) void* _configure4PWM(long pwm_frequency, const int pin1A, const int pin1B, const int pin2A, const int pin2B) {
  GenericDriverParams* params = new GenericDriverParams {
    .pins = { pin1A, pin1B, pin2A, pin2B },
    .pwm_frequency = pwm_frequency
  };
  return params;
}

// 配置 PWM 频率、分辨率和对齐
// - 无刷直流电机驱动 - 6PWM 设置
// - 硬件特定
__attribute__((weak)) void* _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l){
  _UNUSED(pwm_frequency);
  _UNUSED(dead_zone);
  _UNUSED(pinA_h);
  _UNUSED(pinA_l);
  _UNUSED(pinB_h);
  _UNUSED(pinB_l);
  _UNUSED(pinC_h);
  _UNUSED(pinC_l);

  return SIMPLEFOC_DRIVER_INIT_FAILED;
}

// 设置 PWM 占空比到硬件的函数
// - 步进电机 - 1PWM 设置
// - 硬件特定
__attribute__((weak)) void _writeDutyCycle1PWM(float dc_a, void* params){
  // 将占空比从 [0,1] 转换为 [0,255]
  analogWrite(((GenericDriverParams*)params)->pins[0], 255.0f*dc_a);
}

// 设置 PWM 占空比到硬件的函数
// - 步进电机 - 2PWM 设置
// - 硬件特定
__attribute__((weak)) void _writeDutyCycle2PWM(float dc_a,  float dc_b, void* params){
  // 将占空比从 [0,1] 转换为 [0,255]
  analogWrite(((GenericDriverParams*)params)->pins[0], 255.0f*dc_a);
  analogWrite(((GenericDriverParams*)params)->pins[1], 255.0f*dc_b);
}

// 设置 PWM 占空比到硬件的函数
// - 无刷直流电机 - 3PWM 设置
// - 硬件特定
__attribute__((weak)) void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, void* params){
  // 将占空比从 [0,1] 转换为 [0,255]
  analogWrite(((GenericDriverParams*)params)->pins[0], 255.0f*dc_a);
  analogWrite(((GenericDriverParams*)params)->pins[1], 255.0f*dc_b);
  analogWrite(((GenericDriverParams*)params)->pins[2], 255.0f*dc_c);
}

// 设置 PWM 占空比到硬件的函数
// - 步进电机 - 4PWM 设置
// - 硬件特定
__attribute__((weak)) void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, void* params){
  // 将占空比从 [0,1] 转换为 [0,255]
  
  analogWrite(((GenericDriverParams*)params)->pins[0], 255.0f*dc_1a);
  analogWrite(((GenericDriverParams*)params)->pins[1], 255.0f*dc_1b);
  analogWrite(((GenericDriverParams*)params)->pins[2], 255.0f*dc_2a);
  analogWrite(((GenericDriverParams*)params)->pins[3], 255.0f*dc_2b);
}

// 设置 PWM 引脚的占空比的函数 (例如 analogWrite())
// - 无刷直流电机驱动 - 6PWM 设置
// - 硬件特定
__attribute__((weak)) void _writeDutyCycle6PWM(float dc_a,  float dc_b, float dc_c, PhaseState *phase_state, void* params){
  _UNUSED(dc_a);
  _UNUSED(dc_b);
  _UNUSED(dc_c);
  _UNUSED(phase_state);
  _UNUSED(params);
}
