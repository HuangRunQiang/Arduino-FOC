#ifndef FOCMOTOR_H
#define FOCMOTOR_H

#include "Arduino.h"
#include "Sensor.h"
#include "CurrentSense.h"

#include "../time_utils.h"
#include "../foc_utils.h"
#include "../defaults.h"
#include "../pid.h"
#include "../lowpass_filter.h"

// 监控位图
#define _MON_TARGET 0b1000000 // 监控目标值
#define _MON_VOLT_Q 0b0100000 // 监控电压 q 值
#define _MON_VOLT_D 0b0010000 // 监控电压 d 值
#define _MON_CURR_Q 0b0001000 // 监控电流 q 值 - 如果已测量
#define _MON_CURR_D 0b0000100 // 监控电流 d 值 - 如果已测量
#define _MON_VEL    0b0000010 // 监控速度值
#define _MON_ANGLE  0b0000001 // 监控角度值

/**
 *  运动控制类型
 */
enum MotionControlType : uint8_t {
  torque            = 0x00,     //!< 力矩控制
  velocity          = 0x01,     //!< 速度运动控制
  angle             = 0x02,     //!< 位置/角度运动控制
  velocity_openloop = 0x03,
  angle_openloop    = 0x04
};

/**
 *  力矩控制类型
 */
enum TorqueControlType : uint8_t { 
  voltage            = 0x00,     //!< 使用电压的力矩控制
  dc_current         = 0x01,     //!< 使用直流电流的力矩控制（一个电流幅度）
  foc_current        = 0x02,     //!< 使用 dq 电流的力矩控制
};

/**
 *  FOC 调制类型
 */
enum FOCModulationType : uint8_t {
  SinePWM            = 0x00,     //!< 正弦 PWM 调制
  SpaceVectorPWM     = 0x01,     //!< 空间矢量调制方法
  Trapezoid_120      = 0x02,     
  Trapezoid_150      = 0x03,     
};

enum FOCMotorStatus : uint8_t {
  motor_uninitialized = 0x00,     //!< 电机尚未初始化
  motor_initializing  = 0x01,     //!< 电机初始化正在进行
  motor_uncalibrated  = 0x02,     //!< 电机已初始化，但未校准（可以开放回路）
  motor_calibrating   = 0x03,     //!< 电机校准正在进行
  motor_ready         = 0x04,     //!< 电机已初始化并校准（可以闭环）
  motor_error         = 0x08,     //!< 电机处于错误状态（可恢复，例如过流保护已启用）
  motor_calib_failed  = 0x0E,     //!< 电机校准失败（可能可恢复）
  motor_init_failed   = 0x0F,     //!< 电机初始化失败（不可恢复）
};

/**
 * 通用电机类
 */
class FOCMotor
{
  public:
    /**
     * 默认构造函数 - 将所有变量设置为默认值
     */
    FOCMotor();

    /** 电机硬件初始化函数 */
    virtual int init()=0;
    /** 电机禁用函数 */
    virtual void disable()=0;
    /** 电机启用函数 */
    virtual void enable()=0;

    /**
     * 将电机与传感器链接的函数 
     * 
     * @param sensor 传感器类，用于 FOC 算法读取电机角度和速度
     */
    void linkSensor(Sensor* sensor);

    /**
     * 将电机与电流感应链接的函数 
     * 
     * @param current_sense CurrentSense 类，用于 FOC 算法读取电机电流测量
     */
    void linkCurrentSense(CurrentSense* current_sense);

    /**
     * 初始化 FOC 算法的函数
     * 并对传感器和电机的零位置进行对齐 
     * 
     * - 如果设置了 zero_electric_offset 参数，则跳过对齐过程
     */  
    virtual int initFOC()=0;
    
    /**
     * 实时运行 FOC 算法的函数
     * 计算电机角度并设置相应的电压 
     * 到相位 PWM 信号
     * - 运行得越快越好 Arduino UNO ~1ms, Bluepill ~ 100us
     */ 
    virtual void loopFOC()=0;

    /**
     * 执行由 BLDCMotor 的控制器参数设置的控制回路的函数。
     * 
     * @param target 目标值，基于电机.controller 的电压、角度或速度
     *                如果未设置，电机将使用其变量 motor.target 中设置的目标
     * 
     * 此函数不需要在每次循环执行时运行 - 取决于使用情况
     */
    virtual void move(float target = NOT_SET)=0;

    /**
    * 使用 FOC 设置 Uq 到电机的最佳角度
    * FOC 算法的核心
    * 
    * @param Uq 当前设置到电机的 q 轴电压
    * @param Ud 当前设置到电机的 d 轴电压
    * @param angle_el 当前电机的电气角度
    */
    virtual void setPhaseVoltage(float Uq, float Ud, float angle_el)=0;
    
    // 状态计算方法 
    /** 轴角度计算，单位为弧度 [rad] */
    float shaftAngle();
    
    /** 
     * 轴角度计算函数，单位为每秒弧度 [rad/s]
     * 实现低通滤波
     */
    float shaftVelocity();

    /** 
     * 电气角度计算  
     */
    float electricalAngle();

    // 状态变量
    float target; //!< 当前目标值 - 取决于控制器
    float feed_forward_velocity = 0.0f; //!< 当前前馈速度
    float shaft_angle; //!< 当前电机角度
    float electrical_angle; //!< 当前电气角度
    float shaft_velocity; //!< 当前电机速度 
    float current_sp; //!< 目标电流 (q 电流)
    float shaft_velocity_sp; //!< 当前目标速度
    float shaft_angle_sp; //!< 当前目标角度
    DQVoltage_s voltage; //!< 当前设置到电机的 d 和 q 电压
    DQCurrent_s current; //!< 当前测量的 d 和 q 电流
    float voltage_bemf; //!< 估计的反电动势电压（如果提供 KV 常数）
    float Ualpha, Ubeta; //!< 用于逆帕克和克拉克变换的相电压 U alpha 和 U beta

    // 电机配置参数
    float voltage_sensor_align; //!< 传感器和电机对齐电压参数
    float velocity_index_search; //!< 索引搜索的目标速度 
    
    // 电机物理参数
    float phase_resistance; //!< 电机相电阻
    int pole_pairs; //!< 电机极对数
    float KV_rating; //!< 电机 KV 额定值
    float phase_inductance; //!< 电机相电感

    // 限制变量
    float voltage_limit; //!< 电压限制变量 - 全局限制
    float current_limit; //!< 电流限制变量 - 全局限制
    float velocity_limit; //!< 速度限制变量 - 全局限制

    // 电机状态变量
    int8_t enabled = 0; //!< 启用或禁用电机标志
    FOCMotorStatus motor_status = FOCMotorStatus::motor_uninitialized; //!< 电机状态
    
    // PWM 调制相关变量
    FOCModulationType foc_modulation; //!< 确定调制算法的参数
    int8_t modulation_centered = 1; //!< 标志 (1) 在驱动器限制 /2 周围的中心调制或 (0) 拉到 0

    // 配置结构
    TorqueControlType torque_controller; //!< 确定力矩控制类型的参数
    MotionControlType controller; //!< 确定使用的控制回路的参数

    // 控制器和低通滤波器
    PIDController PID_current_q{DEF_PID_CURR_P,DEF_PID_CURR_I,DEF_PID_CURR_D,DEF_PID_CURR_RAMP, DEF_POWER_SUPPLY}; //!< 确定 q 电流 PID 配置的参数
    PIDController PID_current_d{DEF_PID_CURR_P,DEF_PID_CURR_I,DEF_PID_CURR_D,DEF_PID_CURR_RAMP, DEF_POWER_SUPPLY}; //!< 确定 d 电流 PID 配置的参数
    LowPassFilter LPF_current_q{DEF_CURR_FILTER_Tf}; //!< 确定电流低通滤波器配置的参数 
    LowPassFilter LPF_current_d{DEF_CURR_FILTER_Tf}; //!< 确定电流低通滤波器配置的参数 
    PIDController PID_velocity{DEF_PID_VEL_P,DEF_PID_VEL_I,DEF_PID_VEL_D,DEF_PID_VEL_RAMP,DEF_PID_VEL_LIMIT}; //!< 确定速度 PID 配置的参数
    PIDController P_angle{DEF_P_ANGLE_P,0,0,0,DEF_VEL_LIM}; //!< 确定位置 PID 配置的参数 
    LowPassFilter LPF_velocity{DEF_VEL_FILTER_Tf}; //!< 确定速度低通滤波器配置的参数 
    LowPassFilter LPF_angle{0.0}; //!< 确定角度低通滤波器配置的参数 
    unsigned int motion_downsample = DEF_MOTION_DOWNSMAPLE; //!< 定义移动命令的下采样比率的参数
    unsigned int motion_cnt = 0; //!< 移动命令下采样的计数变量

    // 传感器相关变量
    float sensor_offset; //!< 用户定义的传感器零偏移
    float zero_electric_angle = NOT_SET; //!< 绝对零电气角度 - 如果可用
    Direction sensor_direction = Direction::UNKNOWN; //!< 默认是顺时针。如果 sensor_direction == Direction::CCW，则方向将与顺时针相反。设置为 UNKNOWN 以通过校准设置
    bool pp_check_result = false; //!< PP 检查的结果，如果在 loopFOC 中运行

    /**
     * 提供 BLDCMotor 类与 
     * 串口接口并启用监控模式的函数
     * 
     * @param serial 监控串口类引用
     */
    void useMonitoring(Print &serial);

    /**
     * 用于与串口绘图仪一起监控电机变量的实用函数
     * 显著降低执行速度!!!!
     */
    void monitor();
    unsigned int monitor_downsample = DEF_MON_DOWNSMAPLE; //!< 每次 monitor_downsample 调用显示监控输出 
    char monitor_start_char = '\0'; //!< 监控开始字符 
    char monitor_end_char = '\0'; //!< 监控输出结束字符 
    char monitor_separator = '\t'; //!< 监控输出分隔字符
    unsigned int monitor_decimals = 4; //!< 监控输出的小数位数
    // 初始监控将显示目标、电压、速度和角度
    uint8_t monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE; //!< 持有用户希望监控的变量图的位数组
   
    /** 
      * 传感器链接：
      * - 编码器 
      * - 磁传感器
      * - 霍尔传感器
    */
    Sensor* sensor; 
    /** 
      * 电流感应链接
    */
    CurrentSense* current_sense; 

    // 监控函数
    Print* monitor_port; //!< 如果提供的串口终端变量
  private:
    // 监控计数变量
    unsigned int monitor_cnt = 0; //!< 计数变量
};

#endif
