#include "FOCMotor.h"
#include "../../communication/SimpleFOCDebug.h"

/**
 * 默认构造函数 - 将所有变量设置为默认值
 */
FOCMotor::FOCMotor()
{
  // 用于定位的最大角速度
  velocity_limit = DEF_VEL_LIM;
  // 设置给电机的最大电压
  voltage_limit = DEF_POWER_SUPPLY;
  // 最开始未设置
  current_limit = DEF_CURRENT_LIM;

  // 索引搜索速度
  velocity_index_search = DEF_INDEX_SEARCH_TARGET_VELOCITY;
  // 传感器和电机对准电压
  voltage_sensor_align = DEF_VOLTAGE_SENSOR_ALIGN;

  // 默认调制方式为SinePWM
  foc_modulation = FOCModulationType::SinePWM;

  // 默认目标值
  target = 0;
  voltage.d = 0;
  voltage.q = 0;
  // 当前目标值
  current_sp = 0;
  current.q = 0;
  current.d = 0;

  // 电压反电动势
  voltage_bemf = 0;

  // 初始化用于逆Park和Clarke变换的相电压U alpha和U beta
  Ualpha = 0;
  Ubeta = 0;
  
  // 监控端口
  monitor_port = nullptr;
  // 传感器
  sensor_offset = 0.0f;
  sensor = nullptr;
  // 电流传感器
  current_sense = nullptr;
}


/**
 * 传感器链接方法
 */
void FOCMotor::linkSensor(Sensor* _sensor) {
  sensor = _sensor;
}

/**
 * 电流传感器链接方法
 */
void FOCMotor::linkCurrentSense(CurrentSense* _current_sense) {
  current_sense = _current_sense;
}

// 轴角计算
float FOCMotor::shaftAngle() {
  // 如果没有链接传感器，则返回之前的值（用于开环控制）
  if(!sensor) return shaft_angle;
  return sensor_direction*LPF_angle(sensor->getAngle()) - sensor_offset;
}

// 轴速度计算
float FOCMotor::shaftVelocity() {
  // 如果没有链接传感器，则返回之前的值（用于开环控制）
  if(!sensor) return shaft_velocity;
  return sensor_direction*LPF_velocity(sensor->getVelocity());
}

float FOCMotor::electricalAngle() {
  // 如果没有链接传感器，则返回之前的值（用于开环控制）
  if(!sensor) return electrical_angle;
  return  _normalizeAngle( (float)(sensor_direction * pole_pairs) * sensor->getMechanicalAngle()  - zero_electric_angle );
}

/**
 * 监控功能
 */
// 实现monitor_port设置器的函数
void FOCMotor::useMonitoring(Print &print){
  monitor_port = &print; // 操作print的地址
  #ifndef SIMPLEFOC_DISABLE_DEBUG
  SimpleFOCDebug::enable(&print);
  SIMPLEFOC_DEBUG("MOT: 监控已启用!");
  #endif
}

// 旨在与串行绘图仪一起使用的实用函数，以监控电机变量
// 会显著降低执行速度!!!!
void FOCMotor::monitor() {
  if( !monitor_downsample || monitor_cnt++ < (monitor_downsample-1) ) return;
  monitor_cnt = 0;
  if(!monitor_port) return;
  bool printed = 0;

  if(monitor_variables & _MON_TARGET){
    if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
    monitor_port->print(target,monitor_decimals);    
    printed= true;
  }
  if(monitor_variables & _MON_VOLT_Q) {
    if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
    else if(printed) monitor_port->print(monitor_separator);
    monitor_port->print(voltage.q,monitor_decimals);
    printed= true;
  }
  if(monitor_variables & _MON_VOLT_D) {
    if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
    else if(printed) monitor_port->print(monitor_separator);
    monitor_port->print(voltage.d,monitor_decimals);
    printed= true;
  }
  // 如果可能，读取电流 - 即使在电压模式下（如果有current_sense可用）
  if(monitor_variables & _MON_CURR_Q || monitor_variables & _MON_CURR_D) {
    DQCurrent_s c = current;
    if( current_sense && torque_controller != TorqueControlType::foc_current ){
      c = current_sense->getFOCCurrents(electrical_angle);
      c.q = LPF_current_q(c.q);
      c.d = LPF_current_d(c.d);
    }
    if(monitor_variables & _MON_CURR_Q) {
      if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
      else if(printed) monitor_port->print(monitor_separator);
      monitor_port->print(c.q*1000, monitor_decimals); // 毫安
      printed= true;
    }
    if(monitor_variables & _MON_CURR_D) {
      if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
      else if(printed) monitor_port->print(monitor_separator);
      monitor_port->print(c.d*1000, monitor_decimals); // 毫安
      printed= true;
    }
  }
 
  if(monitor_variables & _MON_VEL) {
    if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
    else if(printed) monitor_port->print(monitor_separator);
    monitor_port->print(shaft_velocity,monitor_decimals);
    printed= true;
  }
  if(monitor_variables & _MON_ANGLE) {
    if(!printed && monitor_start_char) monitor_port->print(monitor_start_char);
    else if(printed) monitor_port->print(monitor_separator);
    monitor_port->print(shaft_angle,monitor_decimals);
    printed= true;
  }
  if(printed){
    if(monitor_end_char) monitor_port->println(monitor_end_char);
    else monitor_port->println("");
  }
}
