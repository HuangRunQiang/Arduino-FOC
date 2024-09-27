#ifndef ENCODER_LIB_H
#define ENCODER_LIB_H

#include "Arduino.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "../common/base_classes/Sensor.h"

/**
 *  正交模式配置结构
 */
enum Quadrature : uint8_t {
  ON    = 0x00, //!< 启用正交模式 CPR = 4xPPR
  OFF   = 0x01  //!< 禁用正交模式 / CPR = PPR
};

class Encoder: public Sensor{
 public:
    /**
    编码器类构造函数
    @param encA  编码器 A 引脚
    @param encB  编码器 B 引脚
    @param ppr   每转脉冲数（cpr=ppr*4）
    @param index 索引引脚编号（可选输入）
    */
    Encoder(int encA, int encB , float ppr, int index = 0);

    /** 初始化编码器引脚 */
    void init() override;
    
    /**
     *  启用编码器通道的硬件中断，并提供回调函数
     *  如果未提供回调，则不启用中断
     * 
     * @param doA 指向 A 通道中断处理函数的指针
     * @param doB 指向 B 通道中断处理函数的指针
     * @param doIndex 指向索引通道中断处理函数的指针
     * 
     */
    void enableInterrupts(void (*doA)() = nullptr, void(*doB)() = nullptr, void(*doIndex)() = nullptr);
    
    // 编码器中断回调函数
    /** A 通道回调函数 */
    void handleA();
    /** B 通道回调函数 */
    void handleB();
    /** 索引通道回调函数 */
    void handleIndex();
    
    // A 和 B 引脚
    int pinA; //!< 编码器硬件引脚 A
    int pinB; //!< 编码器硬件引脚 B
    int index_pin; //!< 索引引脚

    // 编码器配置
    Pullup pullup; //!< 配置参数：内部或外部上拉
    Quadrature quadrature; //!< 配置参数：启用或禁用正交模式
    float cpr; //!< 编码器每转计数

    // Sensor 类的抽象函数实现
    /** 获取当前角度（弧度） */
    float getSensorAngle() override;
    /** 获取当前角速度（弧度/秒） */
    float getVelocity() override;
    virtual void update() override;

    /**
     * 如果需要搜索绝对零点则返回 1
     * 0 - 没有索引的编码器 
     * 1 - 有索引的编码器
     */
    int needsSearch() override;

  private:
    int hasIndex(); //!< 函数返回 1 如果编码器有索引引脚，返回 0 如果没有。

    volatile long pulse_counter; //!< 当前脉冲计数器
    volatile long pulse_timestamp; //!< 最近脉冲时间戳（微秒）
    volatile int A_active; //!< 当前 A 通道的活动状态
    volatile int B_active; //!< 当前 B 通道的活动状态
    volatile int I_active; //!< 当前索引通道的活动状态
    volatile bool index_found = false; //!< 标志，表示索引已被找到

    // 速度计算变量
    float prev_Th, pulse_per_second;
    volatile long prev_pulse_counter, prev_timestamp_us;
};

#endif
