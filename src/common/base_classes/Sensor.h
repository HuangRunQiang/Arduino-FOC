#ifndef SENSOR_H
#define SENSOR_H

#include <inttypes.h>

/**
 *  方向结构体
 */
enum Direction : int8_t {
    CW      = 1,  // 顺时针
    CCW     = -1, // 逆时针
    UNKNOWN = 0   // 尚未知道或无效状态
};


/**
 *  上拉配置结构体
 */
enum Pullup : uint8_t {
    USE_INTERN = 0x00, //!< 使用内部上拉
    USE_EXTERN = 0x01  //!< 使用外部上拉
};

/**
 *  传感器抽象类定义
 * 
 *  此类故意保持简单，作为各种传感器的基础。目前我们有
 *  编码器、磁编码器和霍尔传感器的实现。这个基类提取了
 *  最基本的共同特性，以便FOC驱动程序可以获取其操作所需的数据。
 * 
 *  要实现您自己的传感器，请创建此类的子类，并实现getSensorAngle()
 *  方法。getSensorAngle()返回一个浮点值，以弧度表示当前轴角，
 *  范围为0到2*PI（一个完整的旋转）。
 * 
 *  为了正确工作，传感器类的update()方法必须足够快速地被调用。通常，
 *  BLDCMotor的loopFOC()函数每次迭代调用一次，因此您必须确保快速调用
 *  loopFOC()，以确保电机和传感器的正确操作。
 * 
 *  传感器基类提供了getVelocity()的实现，并以精确的方式处理完整
 *  旋转的计数，但如果您愿意，可以额外重写这些方法，以便为您的硬件
 *  提供更优的实现。
 * 
 */
class Sensor{
    friend class SmoothingSensor;
    public:
        /**
         * 获取机械轴角，范围为0到2PI。此值将尽可能精确地与
         * 硬件匹配。基本实现使用update()返回的值，以便
         * 在再次调用update()之前返回相同的值。
         */
        virtual float getMechanicalAngle();

        /**
         * 获取当前位置（以弧度为单位），包括完整旋转和轴角。
         * 基本实现使用update()返回的值，以便在再次调用
         * update()之前返回相同的值。
         * 注意，随着旋转次数的增加，此值的精度有限，
         * 因为浮点数的有限精度无法同时捕捉完整旋转的大角度
         * 和轴角的小角度。
         */
        virtual float getAngle();
        
        /** 
         * 在支持的架构上，这将返回一个双精度位置值，
         * 对于大位置值应具有更好的精度。
         * 基本实现使用update()返回的值，以便在再次调用
         * update()之前返回相同的值。
         */
        virtual double getPreciseAngle();

        /** 
         * 获取当前角速度（弧度/秒）
         * 可以在子类中重写。基本实现使用update()返回的值，
         * 因此只有在此期间调用update()时才有意义。
         */
        virtual float getVelocity();

        /**
         * 获取完整旋转的数量
         * 基本实现使用update()返回的值，以便在再次调用
         * update()之前返回相同的值。 
         */
        virtual int32_t getFullRotations();

        /**
         * 通过读取硬件传感器更新传感器值。
         * 一些实现可能使用中断，不需要此操作。
         * 基本实现调用getSensorAngle()，并更新内部
         * 字段的角度、时间戳和完整旋转。
         * 此方法必须足够频繁地调用，以确保由于
         * 轮询不频繁而不会“错过”完整旋转。
         * 如果您的传感器硬件需要其他行为，请在子类中重写。
         */
        virtual void update();

        /** 
         * 如果不需要搜索绝对零点，则返回0
         * 0 - 磁传感器（和找到的带有索引的编码器）
         * 1 - 带有索引的编码器（索引尚未找到）
         */
        virtual int needsSearch();

        /**
         * 更新速度之间的最小时间。如果经过的时间低于此值，则速度不会更新。
         */
        float min_elapsed_time = 0.000100; // 默认是100微秒，或10kHz

    protected:
        /** 
         * 从传感器硬件获取当前轴角，并
         * 以弧度形式返回，范围为0到2PI。
         * 
         * 此方法是纯虚拟的，必须在子类中实现。
         * 直接调用此方法不会更新基类的内部字段。
         * 从外部代码调用时请使用update()。
         */
        virtual float getSensorAngle()=0;
        
        /**
         * 如果您希望启动更平滑，请从传感器子类的init方法中调用Sensor::init()。
         * 基类的init()方法多次调用getSensorAngle()以初始化内部字段
         * 为当前值，确保在第一次调用sensor.getAngle()和sensor.getVelocity()
         * 时没有不连续性（“跳跃到零”）。
         */
        virtual void init();

        // 速度计算变量
        float velocity=0.0f;
        float angle_prev=0.0f; // 上次调用getSensorAngle()的结果，用于完整旋转和速度
        long angle_prev_ts=0; // 上次调用getAngle的时间戳，用于速度
        float vel_angle_prev=0.0f; // 上次调用getVelocity时的角度，用于速度
        long vel_angle_prev_ts=0; // 上次速度计算的时间戳
        int32_t full_rotations=0; // 完整旋转跟踪
        int32_t vel_full_rotations=0; // 用于速度计算的上次完整旋转值
};

#endif
