#include "BLDCMotor.h"
#include "./communication/SimpleFOCDebug.h"

// 见 https://www.youtube.com/watch?v=InzXA7mWBWE 第5张幻灯片
// 每个为60度，3相的值为1=正，-1=负，0=高阻抗
int trap_120_map[6][3] = {
    {_HIGH_IMPEDANCE, 1, -1},
    {-1, 1, _HIGH_IMPEDANCE},
    {-1, _HIGH_IMPEDANCE, 1},
    {_HIGH_IMPEDANCE, -1, 1},
    {1, -1, _HIGH_IMPEDANCE},
    {1, _HIGH_IMPEDANCE, -1} 
};

// 见 https://www.youtube.com/watch?v=InzXA7mWBWE 第8张幻灯片
// 每个为30度，3相的值为1=正，-1=负，0=高阻抗
int trap_150_map[12][3] = {
    {_HIGH_IMPEDANCE, 1, -1},
    {-1, 1, -1},
    {-1, 1, _HIGH_IMPEDANCE},
    {-1, 1, 1},
    {-1, _HIGH_IMPEDANCE, 1},
    {-1, -1, 1},
    {_HIGH_IMPEDANCE, -1, 1},
    {1, -1, 1},
    {1, -1, _HIGH_IMPEDANCE},
    {1, -1, -1},
    {1, _HIGH_IMPEDANCE, -1},
    {1, 1, -1} 
};

// BLDCMotor( int pp , float R)
// - pp            - 极对数
// - R             - 电机相电阻
// - KV            - 电机KV额定值（转速/伏特）
// - L             - 电机相电感
BLDCMotor::BLDCMotor(int pp, float _R, float _KV, float _inductance)
: FOCMotor()
{
  // 保存极对数
  pole_pairs = pp;
  // 保存相电阻
  phase_resistance = _R;
  // 保存反电动势常数 KV = 1/KV
  // 1/sqrt(2) - 有效值
  KV_rating = NOT_SET;
  if (_isset(_KV))
    KV_rating = _KV;
  // 保存相电感
  phase_inductance = _inductance;

  // 默认的扭矩控制类型是电压
  torque_controller = TorqueControlType::voltage;
}

/**
	链接控制电机的驱动器
*/
void BLDCMotor::linkDriver(BLDCDriver* _driver) {
  driver = _driver;
}

// 初始化硬件引脚
int BLDCMotor::init() {
  if (!driver || !driver->initialized) {
    motor_status = FOCMotorStatus::motor_init_failed;
    SIMPLEFOC_DEBUG("MOT: 初始化失败，驱动器未初始化");
    return 0;
  }
  motor_status = FOCMotorStatus::motor_initializing;
  SIMPLEFOC_DEBUG("MOT: 初始化中");

  // 对电压限制配置进行合理性检查
  if(voltage_limit > driver->voltage_limit) voltage_limit = driver->voltage_limit;
  // 限制传感器对齐的电压
  if(voltage_sensor_align > voltage_limit) voltage_sensor_align = voltage_limit;

  // 更新控制器限制
  if(current_sense){
    // 电流控制循环控制电压
    PID_current_q.limit = voltage_limit;
    PID_current_d.limit = voltage_limit;
  }
  if(_isset(phase_resistance) || torque_controller != TorqueControlType::voltage){
    // 速度控制循环控制电流
    PID_velocity.limit = current_limit;
  }else{
    // 速度控制循环控制电压
    PID_velocity.limit = voltage_limit;
  }
  P_angle.limit = velocity_limit;

  // 如果使用开环控制，如果尚未设置，则将顺时针设置为默认方向
  if ((controller==MotionControlType::angle_openloop
     ||controller==MotionControlType::velocity_openloop)
     && (sensor_direction == Direction::UNKNOWN)) {
      sensor_direction = Direction::CW;
  }

  _delay(500);
  // 启用电机
  SIMPLEFOC_DEBUG("MOT: 启用驱动器。");
  enable();
  _delay(500);
  motor_status = FOCMotorStatus::motor_uncalibrated;
  return 1;
}

// 禁用电机驱动器
void BLDCMotor::disable()
{
  // 禁用电流传感
  if(current_sense) current_sense->disable();
  // 设置PWM为零
  driver->setPwm(0, 0, 0);
  // 禁用驱动器
  driver->disable();
  // 更新电机状态
  enabled = 0;
}

// 启用电机驱动器
void BLDCMotor::enable()
{
  // 启用驱动器
  driver->enable();
  // 设置PWM为零
  driver->setPwm(0, 0, 0);
  // 启用电流传感
  if(current_sense) current_sense->enable();
  // 重置PID
  PID_velocity.reset();
  P_angle.reset();
  PID_current_q.reset();
  PID_current_d.reset();
  // 更新电机状态
  enabled = 1;
}

/**
  FOC 函数
*/
// FOC 初始化函数
int BLDCMotor::initFOC() {
  int exit_flag = 1;

  motor_status = FOCMotorStatus::motor_calibrating;

  // 如果需要，进行电机对齐
  // 对于编码器，必须进行对齐！
  // 传感器和电机对齐 - 可以通过设置 motor.sensor_direction 和 motor.zero_electric_angle 跳过
  if(sensor){
    exit_flag *= alignSensor();
    // 更新轴角
    sensor->update();
    shaft_angle = shaftAngle();

    // 对当前传感器进行对齐 - 可以跳过
    // 检查驱动器相是否与电流传感相同
    // 并检查测量方向。
    if(exit_flag){
      if(current_sense){ 
        if (!current_sense->initialized) {
          motor_status = FOCMotorStatus::motor_calib_failed;
          SIMPLEFOC_DEBUG("MOT: 初始化 FOC 错误，电流传感未初始化");
          exit_flag = 0;
        }else{
          exit_flag *= alignCurrentSense();
        }
      }
      else { SIMPLEFOC_DEBUG("MOT: 没有电流传感器。"); }
    }

  } else {
    SIMPLEFOC_DEBUG("MOT: 没有传感器。");
    if ((controller == MotionControlType::angle_openloop || controller == MotionControlType::velocity_openloop)){
      exit_flag = 1;    
      SIMPLEFOC_DEBUG("MOT: 仅开环控制！");
    }else{
      exit_flag = 0; // 没有传感器无法进行FOC
    }
  }

  if(exit_flag){
    SIMPLEFOC_DEBUG("MOT: 准备就绪。");
    motor_status = FOCMotorStatus::motor_ready;
  }else{
    SIMPLEFOC_DEBUG("MOT: 初始化 FOC 失败。");
    motor_status = FOCMotorStatus::motor_calib_failed;
    disable();
  }

  return exit_flag;
}


// 校准电机和电流传感器相位
int BLDCMotor::alignCurrentSense() {
  int exit_flag = 1; // 成功

  SIMPLEFOC_DEBUG("MOT: 对齐电流传感器。");

  // 对齐电流传感器和驱动器
  exit_flag = current_sense->driverAlign(voltage_sensor_align, modulation_centered);
  if(!exit_flag){
    // 电流传感器错误 - 相位未测量或连接不良
    SIMPLEFOC_DEBUG("MOT: 对齐错误！");
    exit_flag = 0;
  }else{
    // 输出对齐状态标志
    SIMPLEFOC_DEBUG("MOT: 成功: ", exit_flag);
  }

  return exit_flag > 0;
}

// 编码器对齐到电气零角
int BLDCMotor::alignSensor() {
  int exit_flag = 1; // 成功
  SIMPLEFOC_DEBUG("MOT: 对齐传感器。");

  // 检查传感器是否需要零搜索
  if(sensor->needsSearch()) exit_flag = absoluteZeroSearch();
  // 如果未找到索引则停止初始化
  if(!exit_flag) return exit_flag;

  // v2.3.3 修复 R_AVR_7_PCREL 对 AVR 板的符号“ bug
  // TODO 找出为什么这样做有效
  float voltage_align = voltage_sensor_align;

  // 如果自然方向未知
  if(sensor_direction==Direction::UNKNOWN){

    // 找到自然方向
    // 向前移动一个电气周期
    for (int i = 0; i <=500; i++ ) {
      float angle = _3PI_2 + _2PI * i / 500.0f;
      setPhaseVoltage(voltage_align, 0,  angle);
      sensor->update();
      _delay(2);
    }
    // 在中间取一个角度
    sensor->update();
    float mid_angle = sensor->getAngle();
    // 向后移动一个电气周期
    for (int i = 500; i >=0; i-- ) {
      float angle = _3PI_2 + _2PI * i / 500.0f ;
      setPhaseVoltage(voltage_align, 0,  angle);
      sensor->update();
      _delay(2);
    }
    sensor->update();
    float end_angle = sensor->getAngle();
    // setPhaseVoltage(0, 0, 0);
    _delay(200);
// 确定传感器移动的方向
// 计算中间角度(mid_angle)与结束角度(end_angle)之间的绝对差值
float moved = fabs(mid_angle - end_angle);

// 检测移动的最小角度
if (moved < MIN_ANGLE_DETECT_MOVEMENT) { 
    // 如果移动的角度小于设定的最小检测角度
    SIMPLEFOC_DEBUG("MOT: 未能注意到移动"); // 输出调试信息，表示未能检测到移动
    return 0; // 返回0，表示校准失败
} 
else if (mid_angle < end_angle) {
    // 如果中间角度小于结束角度，表示传感器是逆时针移动
    SIMPLEFOC_DEBUG("MOT: sensor_direction==CCW"); // 输出调试信息，表明传感器方向为逆时针
    sensor_direction = Direction::CCW; // 设置传感器方向为逆时针
} 
else {
    // 如果中间角度大于结束角度，表示传感器是顺时针移动
    SIMPLEFOC_DEBUG("MOT: sensor_direction==CW"); // 输出调试信息，表明传感器方向为顺时针
    sensor_direction = Direction::CW; // 设置传感器方向为顺时针
}

    // 检查极对数
    pp_check_result = !(fabs(moved*pole_pairs - _2PI) > 0.5f); // 0.5f 是一个任意值，可以更低或更高！
    if( pp_check_result==false ) {
      SIMPLEFOC_DEBUG("MOT: PP 检查: 失败 - 估计的极对数: ", _2PI/moved);
    } else {
      SIMPLEFOC_DEBUG("MOT: PP 检查: 成功！");
    }

  } else { SIMPLEFOC_DEBUG("MOT: 跳过方向校准。"); }

  // 零电气角未知
  if(!_isset(zero_electric_angle)){
    // 对齐电机和传感器的电气相位
    // 设置角度 -90（270 = 3PI/2）度
    setPhaseVoltage(voltage_align, 0,  _3PI_2);
    _delay(700);
    // 读取传感器
    sensor->update();
    // 获取当前零电气角
    zero_electric_angle = 0;
    zero_electric_angle = electricalAngle();
    // zero_electric_angle =  _normalizeAngle(_electricalAngle(sensor_direction*sensor->getAngle(), pole_pairs));
    _delay(20);
    if(monitor_port){
      SIMPLEFOC_DEBUG("MOT: 零电气角: ", zero_electric_angle);
    }
    // 停止一切
    setPhaseVoltage(0, 0, 0);
    _delay(200);
  } else { SIMPLEFOC_DEBUG("MOT: 跳过偏移校准。"); }
  return exit_flag;
}

// 编码器对齐绝对零角
// - 到索引
int BLDCMotor::absoluteZeroSearch() {
  // 传感器精度：这都没问题，因为搜索发生在零角附近，浮点数的精度
  //                    是足够的。
  SIMPLEFOC_DEBUG("MOT: 索引搜索...");
  // 以小速度搜索绝对零
  float limit_vel = velocity_limit;
  float limit_volt = voltage_limit;
  velocity_limit = velocity_index_search;
  voltage_limit = voltage_sensor_align;
  shaft_angle = 0;
  while(sensor->needsSearch() && shaft_angle < _2PI){
    angleOpenloop(1.5f*_2PI);
    // 对于某些传感器，调用很重要，以免丢失计数
    // 对于搜索来说不是必需的
    sensor->update();
  }
  // 禁用电机
  setPhaseVoltage(0, 0, 0);
  // 重新初始化限制
  velocity_limit = limit_vel;
  voltage_limit = limit_volt;
  // 检查是否找到零
  if(monitor_port){
    if(sensor->needsSearch()) { SIMPLEFOC_DEBUG("MOT: 错误: 未找到！"); }
    else { SIMPLEFOC_DEBUG("MOT: 成功！"); }
  }
  return !sensor->needsSearch();
}

// 迭代函数循环 FOC 算法，在电机上设置 Uq
// 运行得越快越好
void BLDCMotor::loopFOC() {
  // 更新传感器 - 即使在开环模式下也要这样做，因为用户可能在模式之间切换，我们可能会丢失跟踪
  //                 完整的旋转。
  if (sensor) sensor->update();

  // 如果是开环则不做任何操作
  if( controller==MotionControlType::angle_openloop || controller==MotionControlType::velocity_openloop ) return;
  
  // 如果禁用则不做任何操作
  if(!enabled) return;

  // 需要先调用 update()
  // 此函数不会有数值问题，因为它使用 Sensor::getMechanicalAngle() 
  // 该值范围在 0-2PI 之间
  electrical_angle = electricalAngle();
  switch (torque_controller) {
    case TorqueControlType::voltage:
      // 实际上不需要做任何事情
      break;
    case TorqueControlType::dc_current:
      if(!current_sense) return;
      // 读取整体电流幅度
      current.q = current_sense->getDCCurrent(electrical_angle);
      // 对值进行滤波
      current.q = LPF_current_q(current.q);
      // 计算相电压
      voltage.q = PID_current_q(current_sp - current.q);
      // d 电压 - 滞后补偿
      if(_isset(phase_inductance)) voltage.d = _constrain( -current_sp*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
      else voltage.d = 0;
      break;
    case TorqueControlType::foc_current:
      if(!current_sense) return;
      // 读取 dq 电流
      current = current_sense->getFOCCurrents(electrical_angle);
      // 滤波值
      current.q = LPF_current_q(current.q);
      current.d = LPF_current_d(current.d);
      // 计算相电压
      voltage.q = PID_current_q(current_sp - current.q);
      voltage.d = PID_current_d(-current.d);
      // d 电压 - 滞后补偿 - TODO 验证
      // if(_isset(phase_inductance)) voltage.d = _constrain( voltage.d - current_sp*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
      break;
    default:
      // 未选择扭矩控制
      SIMPLEFOC_DEBUG("MOT: 未选择扭矩控制！");
      break;
  }

  // 设置相电压 - FOC 核心功能 :)
  setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
}

// 迭代函数运行 FOC 算法的外部循环
// 此函数的行为由 motor.controller 变量决定
// 它运行角度、速度或扭矩循环
// - 需要迭代调用，这是异步函数
// - 如果目标未设置，则使用 motor.target 值
void BLDCMotor::move(float new_target) {

  // 设置内部目标变量
  if(_isset(new_target)) target = new_target;
  
  // 下采样（可选）
  if(motion_cnt++ < motion_downsample) return;
  motion_cnt = 0;

  // 轴角/速度需要先调用 update()
  // 获取轴角
  // TODO 传感器精度：shaft_angle 实际上存储了完整的位置，包括完整的旋转，作为浮点数
  //                        因此在角度变大时并不精确。
  //                        此外，LPF 在角度上的工作方式是精度问题，当切换到 2 组件表示时，角度-LPF 是一个问题。
  if( controller!=MotionControlType::angle_openloop && controller!=MotionControlType::velocity_openloop ) 
    shaft_angle = shaftAngle(); // 读取值，即使电机被禁用也保持监控更新，但在开环模式下不这样做
  // 获取角速度 TODO 速度读取可能也不应该发生在开环模式中？
  shaft_velocity = shaftVelocity(); // 读取值，即使电机被禁用也保持监控更新

  // 如果禁用则不做任何操作
  if(!enabled) return;
  
  // 如果 KV_rating 可用，则计算反电动势电压 U_bemf = vel*(1/KV)
  if (_isset(KV_rating)) voltage_bemf = shaft_velocity/(KV_rating*_SQRT3)/_RPM_TO_RADS;
  // 如果相位电阻可用且当前传感器不可用，则估算电机电流
  if(!current_sense && _isset(phase_resistance)) current.q = (voltage.q - voltage_bemf)/phase_resistance;

  // 基于电流的电压限制升级
  switch (controller) {
    case MotionControlType::torque:
      if(torque_controller == TorqueControlType::voltage){ // 如果是电压扭矩控制
        if(!_isset(phase_resistance))  voltage.q = target;
        else  voltage.q =  target*phase_resistance + voltage_bemf;
        voltage.q = _constrain(voltage.q, -voltage_limit, voltage_limit);
        // 设置 d 组件（如果已知电感的滞后补偿）
        if(!_isset(phase_inductance)) voltage.d = 0;
        else voltage.d = _constrain( -target*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
      }else{
        current_sp = target; // 如果是电流/foc_current 扭矩控制
      }
      break;
    case MotionControlType::angle:
      // TODO 传感器精度：此计算在数值上不精确。当角度较大时，目标值无法表示精确位置。
      //                        这导致在高位置值时无法命令小变化。
      //                        要解决此问题，必须以数值精确的方式计算增量角。
      // 角度设定点
      shaft_angle_sp = target;
      // 计算速度设定点
      shaft_velocity_sp = feed_forward_velocity + P_angle( shaft_angle_sp - shaft_angle );
      shaft_velocity_sp = _constrain(shaft_velocity_sp,-velocity_limit, velocity_limit);
      // 计算扭矩命令 - 传感器精度：此计算是可以的，但基于之前计算的错误值
      current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity); // 如果是电压扭矩控制
      // 如果通过电压控制扭矩
      if(torque_controller == TorqueControlType::voltage){
        // 如果未提供相位电阻，则使用电压
        if(!_isset(phase_resistance))  voltage.q = current_sp;
        else  voltage.q =  _constrain( current_sp*phase_resistance + voltage_bemf , -voltage_limit, voltage_limit);
        // 设置 d 组件（如果已知电感的滞后补偿）
        if(!_isset(phase_inductance)) voltage.d = 0;
        else voltage.d = _constrain( -current_sp*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
      }
      break;
    case MotionControlType::velocity:
      // 速度设定点 - 传感器精度：此计算在数值上是精确的。
      shaft_velocity_sp = target;
      // 计算扭矩命令
      current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity); // 如果是电流/foc_current 扭矩控制
      // 如果通过电压控制扭矩
      if(torque_controller == TorqueControlType::voltage){
        // 如果未提供相位电阻，则使用电压
        if(!_isset(phase_resistance))  voltage.q = current_sp;
        else  voltage.q = _constrain( current_sp*phase_resistance + voltage_bemf , -voltage_limit, voltage_limit);
        // 设置 d 组件（如果已知电感的滞后补偿）
        if(!_isset(phase_inductance)) voltage.d = 0;
        else voltage.d = _constrain( -current_sp*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
      }
      break;
    case MotionControlType::velocity_openloop:
      // 开环中的速度控制 - 传感器精度：此计算在数值上是精确的。
      shaft_velocity_sp = target;
      voltage.q = velocityOpenloop(shaft_velocity_sp); // 返回设置到电机的电压
      voltage.d = 0;
      break;
    case MotionControlType::angle_openloop:
      // 开环中的角度控制 - 
      // TODO 传感器精度：此计算在数值上不精确，并且受到
      //                        在高角度时小设定点变化的相同问题的影响
      shaft_angle_sp = target;
      voltage.q = angleOpenloop(shaft_angle_sp); // 返回设置到电机的电压
      voltage.d = 0;
      break;
  }
}



// 使用 FOC 方法在最佳角度设置 Uq 和 Ud 到电机
// 实现空间矢量 PWM 和正弦 PWM 算法的函数
//
// 使用正弦近似的函数
// 常规 sin + cos ~300us    (无内存使用)
// 近似 _sin + _cos ~110us  (400Byte ~ 20% 的内存)
void BLDCMotor::setPhaseVoltage(float Uq, float Ud, float angle_el) {

  float center;
  int sector;
  float _ca, _sa;

  switch (foc_modulation)
  {
    case FOCModulationType::Trapezoid_120 :
      // 参见 https://www.youtube.com/watch?v=InzXA7mWBWE 第 5 幻灯片
      // 确定扇区
      sector = 6 * (_normalizeAngle(angle_el + _PI_6) / _2PI); // 添加 PI/6 以与其他模式对齐
      // 电压居中
      // modulation_centered == true > driver.voltage_limit/2
      // modulation_centered == false > 或可适应的居中，当 Uq=0 时所有相位拉到 0
      center = modulation_centered ? (driver->voltage_limit) / 2 : Uq;

      if (trap_120_map[sector][0] == _HIGH_IMPEDANCE) {
        Ua = center;
        Ub = trap_120_map[sector][1] * Uq + center;
        Uc = trap_120_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_ON, PhaseState::PHASE_ON); // 尽可能禁用相位
      } else if (trap_120_map[sector][1] == _HIGH_IMPEDANCE) {
        Ua = trap_120_map[sector][0] * Uq + center;
        Ub = center;
        Uc = trap_120_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_OFF, PhaseState::PHASE_ON); // 尽可能禁用相位
      } else {
        Ua = trap_120_map[sector][0] * Uq + center;
        Ub = trap_120_map[sector][1] * Uq + center;
        Uc = center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_OFF); // 尽可能禁用相位
      }
      break;

    case FOCModulationType::Trapezoid_150 :
      // 参见 https://www.youtube.com/watch?v=InzXA7mWBWE 第 8 幻灯片
      // 确定扇区
      sector = 12 * (_normalizeAngle(angle_el + _PI_6) / _2PI); // 添加 PI/6 以与其他模式对齐
      // 电压居中
      center = modulation_centered ? (driver->voltage_limit) / 2 : Uq;

      if (trap_150_map[sector][0] == _HIGH_IMPEDANCE) {
        Ua = center;
        Ub = trap_150_map[sector][1] * Uq + center;
        Uc = trap_150_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_ON, PhaseState::PHASE_ON); // 尽可能禁用相位
      } else if (trap_150_map[sector][1] == _HIGH_IMPEDANCE) {
        Ua = trap_150_map[sector][0] * Uq + center;
        Ub = center;
        Uc = trap_150_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_OFF, PhaseState::PHASE_ON); // 尽可能禁用相位
      } else if (trap_150_map[sector][2] == _HIGH_IMPEDANCE) {
        Ua = trap_150_map[sector][0] * Uq + center;
        Ub = trap_150_map[sector][1] * Uq + center;
        Uc = center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_OFF); // 尽可能禁用相位
      } else {
        Ua = trap_150_map[sector][0] * Uq + center;
        Ub = trap_150_map[sector][1] * Uq + center;
        Uc = trap_150_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_ON); // 启用所有相位
      }
      break;

    case FOCModulationType::SinePWM :
    case FOCModulationType::SpaceVectorPWM :
      // 正弦 PWM 调制
      // 逆帕克 + 克拉克变换
      _sincos(angle_el, &_sa, &_ca);

      // 逆帕克变换
      Ualpha = _ca * Ud - _sa * Uq;  // -sin(angle) * Uq;
      Ubeta = _sa * Ud + _ca * Uq;    // cos(angle) * Uq;

      // 克拉克变换
      Ua = Ualpha;
      Ub = -0.5f * Ualpha + _SQRT3_2 * Ubeta;
      Uc = -0.5f * Ualpha - _SQRT3_2 * Ubeta;

      center = driver->voltage_limit / 2;
      if (foc_modulation == FOCModulationType::SpaceVectorPWM) {
        // 这里讨论过: https://community.simplefoc.com/t/embedded-world-2023-stm32-cordic-co-processor/3107/165?u=candas1
        // 这里有更多信息: https://microchipdeveloper.com/mct5001:which-zsm-is-best
        // 中点夹紧
        float Umin = min(Ua, min(Ub, Uc));
        float Umax = max(Ua, max(Ub, Uc));
        center -= (Umax + Umin) / 2;
      } 

      if (!modulation_centered) {
        float Umin = min(Ua, min(Ub, Uc));
        Ua -= Umin;
        Ub -= Umin;
        Uc -= Umin;
      } else {
        Ua += center;
        Ub += center;
        Uc += center;
      }
      break;
  }

  // 在驱动器中设置电压
  driver->setPwm(Ua, Ub, Uc);
}

// 生成开环运动以达到目标速度的函数（迭代）
// - target_velocity - 弧度/秒
// 使用 voltage_limit 变量
float BLDCMotor::velocityOpenloop(float target_velocity) {
  // 获取当前时间戳
  unsigned long now_us = _micros();
  // 计算上次调用的采样时间
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;
  // 针对奇怪情况的快速修复（微秒溢出 + 时间戳未定义）
  if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

  // 计算达到目标速度所需的角度
  shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);
  // 出于显示目的
  shaft_velocity = target_velocity;

  // 使用电压限制或电流限制
  float Uq = voltage_limit;
  if (_isset(phase_resistance)) {
    Uq = _constrain(current_limit * phase_resistance + fabs(voltage_bemf), -voltage_limit, voltage_limit);
    // 重新计算电流  
    current.q = (Uq - fabs(voltage_bemf)) / phase_resistance;
  }
  // 使用所需角度设置最大允许电压（voltage_limit）
  setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle, pole_pairs));

  // 保存时间戳以便下次调用
  open_loop_timestamp = now_us;

  return Uq;
}

// 生成开环运动以达到目标角度的函数（迭代）
// - target_angle - 弧度
// 使用 voltage_limit 和 velocity_limit 变量
float BLDCMotor::angleOpenloop(float target_angle) {
  // 获取当前时间戳
  unsigned long now_us = _micros();
  // 计算上次调用的采样时间
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;
  // 针对奇怪情况的快速修复（微秒溢出 + 时间戳未定义）
  if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

  // 计算从当前位置移动到目标角度所需的角度
  // 最大速度（velocity_limit）
  // TODO 传感器精度：此计算在数值上不精确。当总位置较大时，角度可能增长到小位置变化无法通过浮点数精度捕获的程度。
  if (abs(target_angle - shaft_angle) > abs(velocity_limit * Ts)) {
    shaft_angle += _sign(target_angle - shaft_angle) * abs(velocity_limit) * Ts;
    shaft_velocity = velocity_limit;
  } else {
    shaft_angle = target_angle;
    shaft_velocity = 0;
  }

  // 使用电压限制或电流限制
  float Uq = voltage_limit;
  if (_isset(phase_resistance)) {
    Uq = _constrain(current_limit * phase_resistance + fabs(voltage_bemf), -voltage_limit, voltage_limit);
    // 重新计算电流  
    current.q = (Uq - fabs(voltage_bemf)) / phase_resistance;
  }
  // 使用所需角度设置最大允许电压（voltage_limit）
  // 传感器精度：此计算是可以的，因为进行了归一化
  setPhaseVoltage(Uq, 0, _electricalAngle(_normalizeAngle(shaft_angle), pole_pairs));

  // 保存时间戳以便下次调用
  open_loop_timestamp = now_us;

  return Uq;
}
