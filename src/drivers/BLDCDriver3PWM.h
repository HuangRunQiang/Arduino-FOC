#ifndef BLDCDriver3PWM_h
#define BLDCDriver3PWM_h

#include "../common/base_classes/BLDCDriver.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "../common/defaults.h"
#include "hardware_api.h"

/**
  三相PWM无刷直流电机驱动类
*/
class BLDCDriver3PWM: public BLDCDriver
{
  public:
    /**
      BLDCDriver类构造函数
      @param phA A相PWM引脚
      @param phB B相PWM引脚
      @param phC C相PWM引脚
      @param en1 使能引脚（可选输入）
      @param en2 使能引脚（可选输入）
      @param en3 使能引脚（可选输入）
    */
    BLDCDriver3PWM(int phA, int phB, int phC, int en1 = NOT_SET, int en2 = NOT_SET, int en3 = NOT_SET);
    
    /** 电机硬件初始化函数 */
    int init() override;
    /** 电机禁用函数 */
    void disable() override;
    /** 电机启用函数 */
    void enable() override;

    // 硬件变量
    int pwmA; //!< A相PWM引脚编号
    int pwmB; //!< B相PWM引脚编号
    int pwmC; //!< C相PWM引脚编号
    int enableA_pin; //!< 使能引脚编号
    int enableB_pin; //!< 使能引脚编号
    int enableC_pin; //!< 使能引脚编号

    /** 
     * 设置相电压到硬件 
     * 
     * @param Ua - A相电压
     * @param Ub - B相电压
     * @param Uc - C相电压
    */
    void setPwm(float Ua, float Ub, float Uc) override;

    /** 
     * 设置相电压到硬件
     * > 只有在驱动器为所有相有单独的使能引脚时才可能！  
     * 
     * @param sc - A相状态：激活 / 禁用（高阻抗）
     * @param sb - B相状态：激活 / 禁用（高阻抗）
     * @param sa - C相状态：激活 / 禁用（高阻抗）
    */
    virtual void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) override;
  
  private:
};

#endif
