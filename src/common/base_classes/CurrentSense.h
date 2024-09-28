#ifndef CURRENTSENSE_H
#define CURRENTSENSE_H

#include "FOCDriver.h"
#include "../foc_utils.h"
#include "../time_utils.h"
#include "StepperDriver.h"
#include "BLDCDriver.h"

/**
 *  电流感应抽象类定义
 * 每个电流感应实现都需要扩展此接口
 */
class CurrentSense {
    public:

    /**
     *  初始化CurrentSense类的函数
     *   - 所有必要的ADC和同步初始化应在此实现
     *   
     * @returns -  0 - 失败 &  1 - 成功 
     */
    virtual int init() = 0;
    
    /**
     * 将电流感应与电机驱动器链接
     * 仅在需要两者之间的同步时才需要
     */
    void linkDriver(FOCDriver *driver);

    // 变量
    bool skip_align = false; //!< 变量，指示在initFOC()期间应验证相电流方向
    
    FOCDriver* driver = nullptr; //!< 驱动器链接
    bool initialized = false; // 如果电流感应成功初始化为true   
    void* params = 0; //!< 指向电流感应硬件特定参数的指针
    DriverType driver_type = DriverType::Unknown; //!< 驱动器类型（BLDC或步进电机）
    
    // 每个相的ADC测量增益
    // 支持不同相的不同增益，通常是反向相电流
    // 这应该在以后自动化
    float gain_a; //!< 相A增益
    float gain_b; //!< 相B增益
    float gain_c; //!< 相C增益

    float offset_ia; //!< 零电流A电压值（ADC读取的中心）
    float offset_ib; //!< 零电流B电压值（ADC读取的中心）
    float offset_ic; //!< 零电流C电压值（ADC读取的中心）

    // 硬件变量
  	int pinA; //!< 用于电流测量的A引脚模拟引脚
  	int pinB; //!< 用于电流测量的B引脚模拟引脚
  	int pinC; //!< 用于电流测量的C引脚模拟引脚

    /**
     * 验证以下内容的函数：
     *   - 相电流是否正确定向 
     *   - 它们的顺序是否与驱动器相同
     * 
     * 如果可能，此函数将纠正对齐错误；如果不需要，可以留空（返回1）
     * @returns -  
            0 - 失败
            1 - 成功且未更改
            2 - 成功但引脚重新配置
            3 - 成功但增益反转
            4 - 成功但引脚重新配置且增益反转
     * 
     * 重要提示：在CurrentSense类中提供了默认实现，但可以在子类中重写
     */
    virtual int driverAlign(float align_voltage, bool modulation_centered = false);

    /**
     *  读取相电流a、b和c的函数
     *   此函数将与FOC控制一起使用，通过函数 
     *   CurrentSense::getFOCCurrents(electrical_angle)
     *   - 如果仅有两个相的测量值，则返回电流c等于0
     * 
     *  @return PhaseCurrent_s 当前值
     */
    virtual PhaseCurrent_s getPhaseCurrents() = 0;
    
    /**
     * 读取施加到电机的电流幅度的函数
     *  如果可能，返回绝对值或带符号幅度
     *  可以接收电机电气角度以帮助计算
     *  此函数用于电流控制（非FOC）
     *  
     * @param angle_el - 电机的电气角度（可选） 
     */
    virtual float getDCCurrent(float angle_el = 0);

    /**
     * 用于FOC控制的函数，读取电机的DQ电流 
     *   内部使用函数getPhaseCurrents
     * 
     * @param angle_el - 电机电气角度
     */
    DQCurrent_s getFOCCurrents(float angle_el);

    /**
     * 用于FOC控制中的Clarke变换的函数
     *   读取电机的相电流 
     *   返回alpha和beta电流
     * 
     * @param current - 相电流
     */
    ABCurrent_s getABCurrents(PhaseCurrent_s current);

    /**
     * 用于FOC控制中的Park变换的函数
     *   读取Alpha Beta电流和电机电气角度 
     *   返回D和Q电流
     * 
     * @param current - 相电流
     */
    DQCurrent_s getDQCurrents(ABCurrent_s current, float angle_el);

    /**
     * 启用电流感应。默认实现不执行任何操作，但您可以
     * 重写它以执行一些有用的操作。
     */
    virtual void enable();

    /**
     * 禁用电流感应。默认实现不执行任何操作，但您可以
     * 重写它以执行一些有用的操作。
     */
    virtual void disable();

    /**
     * 用于将电流感应与BLDC电机驱动器对齐的函数
    */
    int alignBLDCDriver(float align_voltage, BLDCDriver* driver, bool modulation_centered);
    
    /**
     * 用于将电流感应与步进电机驱动器对齐的函数
    */
    int alignStepperDriver(float align_voltage, StepperDriver* driver, bool modulation_centered);
    
    /**
     * 用于读取N个样本平均电流值的函数
    */
    PhaseCurrent_s readAverageCurrents(int N=100);

};

#endif
