// 无刷直流电机驱动器独立示例
#include <SimpleFOC.h>

// 无刷直流电机驱动器实例
// BLDCDriver3PWM(pwmA, pwmB, pwmC, (en optional))
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);

void setup() {
  
  // 使用串口进行监控
  Serial.begin(115200);
  // 启用更详细的调试输出
  // 如果不需要，可以注释掉
  SimpleFOCDebug::enable(&Serial);
  
  // 要使用的 PWM 频率 [Hz]
  // 对于 atmega328 固定为 32kHz
  // esp32/stm32/teensy 可配置
  driver.pwm_frequency = 50000;
  // 电源电压 [V]
  driver.voltage_power_supply = 12;
  // 允许的最大直流电压 - 默认电源电压
  driver.voltage_limit = 12;

  // 驱动器初始化
  if (!driver.init()){
    Serial.println("驱动器初始化失败！");
    return;
  }

  // 启用驱动器
  driver.enable();
  Serial.println("驱动器准备就绪！");
  _delay(1000);
}

void loop() {
    // 设置 PWM
    // 相 A: 3V
    // 相 B: 6V
    // 相 C: 5V
    driver.setPwm(3, 6, 5);
}
