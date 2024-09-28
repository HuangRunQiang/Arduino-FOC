#ifndef BLDCMotor_h
#define BLDCMotor_h

#include "Arduino.h"
#include "common/base_classes/FOCMotor.h"
#include "common/base_classes/Sensor.h"
#include "common/base_classes/FOCDriver.h"
#include "common/base_classes/BLDCDriver.h"
#include "common/foc_utils.h"
#include "common/time_utils.h"
#include "common/defaults.h"

/**
 * BLDC电机类
 */
class BLDCMotor: public FOCMotor
{
  public:
    /**
     * BLDCMotor类构造函数
     * @param pp 极对数
     * @param R 电机相位电阻 - [欧姆]
     * @param KV 电机KV额定值 (1/K_bemf) - rpm/V
     * @param L 电机相位电感 - [亨利]
     */ 
    BLDCMotor(int pp,  float R = NOT_SET, float KV = NOT_SET, float L = NOT_SET);
    
    /**
     * 连接电机和FOC驱动器的函数 
     * 
     * @param driver 实现所有硬件特定功能的BLDCDriver类，必要的PWM设置
     */
    virtual void linkDriver(BLDCDriver* driver);

    /** 
     * BLDCDriver链接:
     * - 3PWM 
     * - 6PWM 
     */
    BLDCDriver* driver; 
    
    /** 电机硬件初始化函数 */
  	int init() override;
    /** 禁用电机的函数 */
  	void disable() override;
    /** 启用电机的函数 */
    void enable() override;

    /**
     * 初始化FOC算法的函数
     * 并对传感器和电机的零位置进行对齐 
     */  
    int initFOC() override;
    /**
     * 实时运行FOC算法的函数
     * 计算电机角度并设置适当的电压 
     * 到相位PWM信号
     * - 运行得越快越好，Arduino UNO ~1ms，Bluepill ~ 100us
     */ 
    void loopFOC() override;

    /**
     * 执行由BLDCMotor控制器参数设置的控制循环的函数。
     * 
     * @param target  基于电机.controller的电压、角度或速度
     *                 如果未设置，电机将使用其变量motor.target中设置的目标
     * 
     * 此函数不需要在每次循环执行时运行 - 取决于使用情况
     */
    void move(float target = NOT_SET) override;
    
    float Ua, Ub, Uc; //!< 当前相位电压Ua, Ub和Uc设置到电机
    
  /**
    * 使用FOC在最佳角度设置Uq到电机的方法
    * FOC算法的核心
    * 
    * @param Uq 当前在q轴上设置到电机的电压
    * @param Ud 当前在d轴上设置到电机的电压
    * @param angle_el 当前电机的电气角度
    */
    void setPhaseVoltage(float Uq, float Ud, float angle_el) override;

  private:
    // FOC方法 

    /** 将传感器对齐到电机的电气零角 */
    int alignSensor();
    /** 当前感应和电机相位对齐 */
    int alignCurrentSense();
    /** 电机和传感器对齐到传感器的绝对零角 */
    int absoluteZeroSearch();

        
    // 开环运动控制    
    /**
     * 生成目标速度的开环运动的函数（迭代）
     * 使用电压限制变量
     * 
     * @param target_velocity - rad/s
     */
    float velocityOpenloop(float target_velocity);
    /**
     * 生成朝向目标角度的开环运动的函数（迭代）
     * 使用电压限制和速度限制变量
     * 
     * @param target_angle - rad
     */
    float angleOpenloop(float target_angle);
    // 开环变量
    long open_loop_timestamp;
};

#endif
