// 开环电机控制示例
#include <SimpleFOC.h>

// 无刷直流电机及驱动实例
// BLDCMotor motor = BLDCMotor(极对数);
BLDCMotor motor = BLDCMotor(11);
// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, 启用(可选));
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);

// 步进电机及驱动实例
// StepperMotor motor = StepperMotor(50);
// StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6, 8);

// 目标变量
float target_velocity = 0;

// 实例化指令处理器
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }

void setup() {
  // 使用串口监控
  Serial.begin(115200);
  // 启用更详细的调试输出
  // 如果不需要，可以注释掉
  SimpleFOCDebug::enable(&Serial);

  // 驱动器配置
  // 电源电压 [V]
  driver.voltage_power_supply = 12;
  // 限制驱动器可以设置的最大直流电压
  // 作为低电阻电机的保护措施
  // 此值在启动时固定
  driver.voltage_limit = 6;
  if(!driver.init()){
    Serial.println("驱动器初始化失败!");
    return;
  }
  // 将电机与驱动器连接
  motor.linkDriver(&driver);

  // 限制电机运动
  // 限制施加到电机的电压
  // 对于高电阻电机，初始值设置得很低
  // 电流 = 电压 / 电阻，因此尽量保持在1安培以下
  motor.voltage_limit = 3;   // [V]
 
  // 开环控制配置
  motor.controller = MotionControlType::velocity_openloop;

  // 初始化电机硬件
  if(!motor.init()){
    Serial.println("电机初始化失败!");
    return;
  }

  // 添加目标命令 T
  command.add('T', doTarget, "目标速度");
  command.add('L', doLimit, "电压限制");

  Serial.println("电机准备就绪!");
  Serial.println("设置目标速度 [rad/s]");
  _delay(1000);
}

void loop() {
  // 开环速度运动
  // 使用 motor.voltage_limit 和 motor.velocity_limit
  // 要将电机“反向”转动，只需设置一个负的 target_velocity
  motor.move(target_velocity);

  // 用户通信
  command.run();
}
