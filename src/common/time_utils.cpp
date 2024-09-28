#include "time_utils.h"

// 函数：缓冲延迟
// Arduino Uno 的 delay() 函数在中断情况下表现不佳
void _delay(unsigned long ms){
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__)
  // 如果是 Arduino Uno 和其他 ATmega328P 芯片
  // 使用 while 循环代替 delay，
  // 因为基于更改的 timer0 的错误测量
  unsigned long t = _micros() + ms * 1000; // 计算目标时间
  while (_micros() < t) {}; // 循环直到达到目标时间
#else
  // 常规的 micros
  delay(ms); // 使用标准的延迟函数
#endif
}

// 函数：缓冲 _micros()
// Arduino 的 micros() 函数在中断情况下表现不佳
unsigned long _micros(){
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__)
  // 如果是 Arduino Uno 和其他 ATmega328P 芯片
  // 根据分频器返回值
  if ((TCCR0B & 0b00000111) == 0x01) return (micros() / 32); // 如果分频器设置为 1
  else return (micros()); // 否则返回常规的 micros
#else
  // 常规的 micros
  return micros(); // 使用标准的 micros 函数
#endif
}
