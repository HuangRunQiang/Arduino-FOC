#ifndef STEPPERDRIVER_H
#define STEPPERDRIVER_H

#include "Arduino.h"
#include "FOCDriver.h"

class StepperDriver: public FOCDriver {
    public:
        
        /** 
         * 设置相电压到硬件 
         * 
         * @param Ua 相A电压
         * @param Ub 相B电压
        */
        virtual void setPwm(float Ua, float Ub) = 0;

        /**
         * 设置相状态，启用/禁用
         *
         * @param sa - 相A状态 : 激活 / 禁用（高阻抗）
         * @param sb - 相B状态 : 激活 / 禁用（高阻抗）
        */
        virtual void setPhaseState(PhaseState sa, PhaseState sb) = 0;
        
        /** 驱动器类型获取函数 */
        virtual DriverType type() override { return DriverType::Stepper; };
};

#endif
