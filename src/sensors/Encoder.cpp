#include "Encoder.h"

/*
  Encoder(int encA, int encB , int cpr, int index)
  - encA, encB    - 编码器 A 和 B 引脚
  - cpr           - 每转计数（cpm=ppm*4）
  - index pin     - （可选输入）
*/

Encoder::Encoder(int _encA, int _encB , float _ppr, int _index){

  // 编码器测量结构初始化
  // 硬件引脚
  pinA = _encA;
  pinB = _encB;
  // 计数器设置
  pulse_counter = 0;
  pulse_timestamp = 0;

  cpr = _ppr;
  A_active = 0;
  B_active = 0;
  I_active = 0;
  // 索引引脚
  index_pin = _index; // 如果未使用则为 0

  // 速度计算变量
  prev_Th = 0;
  pulse_per_second = 0;
  prev_pulse_counter = 0;
  prev_timestamp_us = _micros();

  // 默认使用外部上拉
  pullup = Pullup::USE_EXTERN;
  // 默认启用正交编码器
  quadrature = Quadrature::ON;
}

// 编码器中断回调函数
// A 通道
void Encoder::handleA() {
  bool A = digitalRead(pinA);
  switch (quadrature){
    case Quadrature::ON:
      // CPR = 4xPPR
      if ( A != A_active ) {
        pulse_counter += (A_active == B_active) ? 1 : -1;
        pulse_timestamp = _micros();
        A_active = A;
      }
      break;
    case Quadrature::OFF:
      // CPR = PPR
      if(A && !digitalRead(pinB)){
        pulse_counter++;
        pulse_timestamp = _micros();
      }
      break;
  }
}

// B 通道
void Encoder::handleB() {
  bool B = digitalRead(pinB);
  switch (quadrature){
    case Quadrature::ON:
      // CPR = 4xPPR
      if ( B != B_active ) {
        pulse_counter += (A_active != B_active) ? 1 : -1;
        pulse_timestamp = _micros();
        B_active = B;
      }
      break;
    case Quadrature::OFF:
      // CPR = PPR
      if(B && !digitalRead(pinA)){
        pulse_counter--;
        pulse_timestamp = _micros();
      }
      break;
  }
}

// 索引通道
void Encoder::handleIndex() {
  if(hasIndex()){
    bool I = digitalRead(index_pin);
    if(I && !I_active){
      index_found = true;
      // 在每个索引上对齐编码器
      long tmp = pulse_counter;
      // 修正计数器值
      pulse_counter = round((double)pulse_counter/(double)cpr)*cpr;
      // 保留相对速度
      prev_pulse_counter += pulse_counter - tmp;
    }
    I_active = I;
  }
}

// 传感器更新函数。安全地将易失性中断变量复制到传感器基类状态变量中。
void Encoder::update() {
  // 在最小持续时间的阻塞部分中复制易失性变量，以确保没有中断被遗漏
  noInterrupts();
  angle_prev_ts = pulse_timestamp;
  long copy_pulse_counter = pulse_counter;
  interrupts();
  // TODO: 如果 pulse_counter 溢出，角度将丢失，这里存在数值精度问题
  full_rotations = copy_pulse_counter / (int)cpr;
  angle_prev = _2PI * ((copy_pulse_counter) % ((int)cpr)) / ((float)cpr);
}

/*
  轴角度计算
*/
float Encoder::getSensorAngle(){
  return _2PI * (pulse_counter) / ((float)cpr);
}

/*
  轴速度计算
  使用混合时间和频率测量技术的函数
*/
float Encoder::getVelocity(){
  // 在最小持续时间的阻塞部分中复制易失性变量，以确保没有中断被遗漏
  noInterrupts();
  long copy_pulse_counter = pulse_counter;
  long copy_pulse_timestamp = pulse_timestamp;
  interrupts();
  // 时间戳
  long timestamp_us = _micros();
  // 采样时间计算
  float Ts = (timestamp_us - prev_timestamp_us) * 1e-6f;
  // 对于奇怪情况的快速修复（微秒溢出）
  if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

  // 上一个脉冲的时间
  float Th = (timestamp_us - copy_pulse_timestamp) * 1e-6f;
  long dN = copy_pulse_counter - prev_pulse_counter;

  // 每秒脉冲计算 (Eq.3.)
  // dN - 接收到的脉冲
  // Ts - 采样时间 - 函数调用之间的时间
  // Th - 上一个脉冲的时间
  // Th_1 - 上一个调用的最后脉冲时间
  // 只有在接收到一些脉冲时才递增
  float dt = Ts + prev_Th - Th;
  pulse_per_second = (dN != 0 && dt > Ts/2) ? dN / dt : pulse_per_second;

  // 如果在脉冲之间经过超过 0.05f 的时间
  if ( Th > 0.1f) pulse_per_second = 0;

  // 速度计算
  float velocity = pulse_per_second / ((float)cpr) * (_2PI);

  // 保存变量以备下次使用
  prev_timestamp_us = timestamp_us;
  // 保存速度计算变量
  prev_Th = Th;
  prev_pulse_counter = copy_pulse_counter;
  return velocity;
}

// 获取索引引脚
// 如果没有索引则返回 -1
int Encoder::needsSearch(){
  return hasIndex() && !index_found;
}

// 私有函数，用于确定编码器是否有索引
int Encoder::hasIndex(){
  return index_pin != 0;
}

// 编码器硬件引脚的初始化
// 和计算变量
void Encoder::init(){

  // 编码器 - 检查是否需要为您的编码器上拉
  if(pullup == Pullup::USE_INTERN){
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    if(hasIndex()) pinMode(index_pin,INPUT_PULLUP);
  }else{
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
    if(hasIndex()) pinMode(index_pin,INPUT);
  }

  // 计数器设置
  pulse_counter = 0;
  pulse_timestamp = _micros();
  // 速度计算变量
  prev_Th = 0;
  pulse_per_second = 0;
  prev_pulse_counter = 0;
  prev_timestamp_us = _micros();

  // 初始 cpr = PPR
  // 如果模式为正交，则更改
  if(quadrature == Quadrature::ON) cpr = 4*cpr;

  // 我们不在这里调用 Sensor::init()，因为初始化在 Encoder 类中处理。
}

// 启用硬件中断的函数，提供回调
// 如果未提供回调，则不启用中断
void Encoder::enableInterrupts(void (*doA)(), void(*doB)(), void(*doIndex)()){
  // 如果提供了函数，则附加中断
  switch(quadrature){
    case Quadrature::ON:
      // A 回调和 B 回调
      if(doA != nullptr) attachInterrupt(digitalPinToInterrupt(pinA), doA, CHANGE);
      if(doB != nullptr) attachInterrupt(digitalPinToInterrupt(pinB), doB, CHANGE);
      break;
    case Quadrature::OFF:
      // A 回调和 B 回调
      if(doA != nullptr) attachInterrupt(digitalPinToInterrupt(pinA), doA, RISING);
      if(doB != nullptr) attachInterrupt(digitalPinToInterrupt(pinB), doB, RISING);
      break;
  }

  // 如果使用索引，初始化索引中断
  if(hasIndex() && doIndex != nullptr) attachInterrupt(digitalPinToInterrupt(index_pin), doIndex, CHANGE);
}
