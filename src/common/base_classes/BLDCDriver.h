#ifndef BLDCDRIVER_H
#define BLDCDRIVER_H

#include "Arduino.h"
#include "FOCDriver.h"

class BLDCDriver: public FOCDriver{
    public:

        float dc_a; //!< 当前设置的相A占空比
        float dc_b; //!< 当前设置的相B占空比
        float dc_c; //!< 当前设置的相C占空比

        /**
         * 将相电压设置为硬件
         *
         * @param Ua - 相A电压
         * @param Ub - 相B电压
         * @param Uc - 相C电压
        */
        virtual void setPwm(float Ua, float Ub, float Uc) = 0;

        /**
         * 设置相状态，启用/禁用
         *
         * @param sc - 相A状态 : 活跃 / 禁用（高阻抗）
         * @param sb - 相B状态 : 活跃 / 禁用（高阻抗）
         * @param sa - 相C状态 : 活跃 / 禁用（高阻抗）
        */
        virtual void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) = 0;

        /** 驱动器类型获取函数 */
        virtual DriverType type() override { return DriverType::BLDC; };
};

#endif
