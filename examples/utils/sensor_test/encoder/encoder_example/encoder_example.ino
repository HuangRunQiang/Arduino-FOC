/**
 *  编码器示例代码 
 * 
 *  这段代码用于测试编码器连接并演示编码器的设置。
 * 
 */

#include <SimpleFOC.h>

Encoder encoder = Encoder(2, 3, 8192); // 创建编码器对象，使用引脚 2 和 3，8192 为每转脉冲数

// 中断例程初始化
void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }

void setup() {
  // 监控端口
  Serial.begin(115200);

  // 启用/禁用正交模式
  encoder.quadrature = Quadrature::ON;

  // 检查是否需要内部上拉
  encoder.pullup = Pullup::USE_EXTERN;
  
  // 初始化编码器硬件
  encoder.init();
  
  // 启用硬件中断
  encoder.enableInterrupts(doA, doB);

  Serial.println("编码器准备就绪");
  _delay(1000);
}

void loop() {
  // 循环函数更新传感器内部变量
  // 通常在 motor.loopFOC() 中调用
  // 对于编码器而言，这里并没有做太多事情
  encoder.update();
  
  // 将角度和角速度显示到终端
  Serial.print(encoder.getAngle());
  Serial.print("\t");
  Serial.println(encoder.getVelocity());
}
