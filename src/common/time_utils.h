#ifndef TIME_UTILS_H
#define TIME_UTILS_H

#include "foc_utils.h"

/** 
 * 实现延迟函数（毫秒）
 * - 阻塞函数
 * - 硬件特定

 * @param ms 要等待的毫秒数
 */
void _delay(unsigned long ms);

/** 
 * 实现获取时间戳的函数（微秒）
 * - 硬件特定
 */
unsigned long _micros();

#endif
