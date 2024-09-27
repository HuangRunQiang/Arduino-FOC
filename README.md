# SimpleFOClibrary - **简单**的场定向控制（FOC）**库** <br>
### 基于 Arduino IDE 和 PlatformIO 的跨平台 FOC 实现，用于无刷直流电机和步进电机<br>

[![AVR build](https://github.com/simplefoc/Arduino-FOC/actions/workflows/arduino.yml/badge.svg)](https://github.com/simplefoc/Arduino-FOC/actions/workflows/arduino.yml)
[![STM32 build](https://github.com/simplefoc/Arduino-FOC/actions/workflows/stm32.yml/badge.svg)](https://github.com/simplefoc/Arduino-FOC/actions/workflows/stm32.yml)
[![ESP32 build](https://github.com/simplefoc/Arduino-FOC/actions/workflows/esp32.yml/badge.svg)](https://github.com/simplefoc/Arduino-FOC/actions/workflows/esp32.yml)
[![RP2040 build](https://github.com/simplefoc/Arduino-FOC/actions/workflows/rpi.yml/badge.svg)](https://github.com/simplefoc/Arduino-FOC/actions/workflows/rpi.yml)
[![SAMD build](https://github.com/simplefoc/Arduino-FOC/actions/workflows/samd.yml/badge.svg)](https://github.com/simplefoc/Arduino-FOC/actions/workflows/samd.yml)
[![Teensy build](https://github.com/simplefoc/Arduino-FOC/actions/workflows/teensy.yml/badge.svg)](https://github.com/simplefoc/Arduino-FOC/actions/workflows/teensy.yml)

![GitHub release (latest by date)](https://img.shields.io/github/v/release/simplefoc/arduino-foc)
![GitHub Release Date](https://img.shields.io/github/release-date/simplefoc/arduino-foc?color=blue)
![GitHub commits since tagged version](https://img.shields.io/github/commits-since/simplefoc/arduino-foc/latest/dev)
![GitHub commit activity (branch)](https://img.shields.io/github/commit-activity/m/simplefoc/arduino-foc/dev)

[![arduino-library-badge](https://ardubadge.simplefoc.com?lib=Simple%20FOC)](https://www.ardu-badge.com/badge/Simple%20FOC.svg)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/askuric/library/Simple%20FOC.svg)](https://registry.platformio.org/libraries/askuric/Simple%20FOC)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![status](https://joss.theoj.org/papers/4382445f249e064e9f0a7f6c1bb06b1d/status.svg)](https://joss.theoj.org/papers/4382445f249e064e9f0a7f6c1bb06b1d)

我们生活在一个非常激动人心的时代 😃！无刷直流电机（BLDC）越来越多地进入爱好者社区，许多出色的项目已经利用了它们远超于传统直流电机的动态性能和功率能力。BLDC 电机相较于普通直流电机有许多优点，但也有一个大缺点，那就是控制的复杂性。尽管设计和制造 PCB 以及创建我们自己的驱动 BLDC 电机的硬件解决方案变得相对简单，但合适的低成本解决方案仍然未能出现。这部分原因在于编写 BLDC 驱动算法的复杂性，场定向控制（FOC）就是其中一种最有效的算法之一。
在线上可以找到的解决方案几乎都是针对特定硬件配置和微控制器架构的。此外，目前大多数努力仍然集中在 BLDC 电机的高功率应用上，适合低成本和低功率的 FOC 支持板在今天非常难以找到，甚至可能根本不存在。<br>
因此，这是一个尝试：
- 🎯 解密 FOC 算法，并创建一个强大但简单的 Arduino 库：[Arduino *SimpleFOClibrary*](https://docs.simplefoc.com/arduino_simplefoc_library_showcase)
  - <i>支持尽可能多的 <b>电机 + 传感器 + 驱动器 + 微控制器</b> 组合</i>
- 🎯 开发模块化和易于使用的 FOC 支持 BLDC 驱动板
   - 有关官方驱动板，请参见 [<span class="simple">Simple<span class="foc">FOC</span>Boards</span>](https://docs.simplefoc.com/boards)
   - 社区成员开发的更多板，请参见 [<span class="simple">Simple<span class="foc">FOC</span>Community</span>](https://community.simplefoc.com/)

> 下一个版本 📢 : <span class="simple">Simple<span class="foc">FOC</span>library</span> v2.3.4
> - 扩展 ESP32 微控制器的支持 [#414](https://github.com/simplefoc/Arduino-FOC/pull/414)
>   - 过渡到 arduino-esp32 版本 v3.x (ESP-IDF v5.x) [#387](https://github.com/espressif/arduino-esp32/releases)
>   - 新增对 MCPWM 驱动的支持
>   - 新增对 LEDC 驱动的支持 - 中心对齐 PWM 和 6PWM 可用
>   - 重写并简化快速 ADC 驱动代码 (`adcRead`) - 用于低侧和内联电流检测。
> - 步进电机电流检测支持 [#421](https://github.com/simplefoc/Arduino-FOC/pull/421)
>   - 支持电流检测（低侧和内联） - [见文档](https://docs.simplefoc.com/current_sense)
>   - 支持真实的 FOC 控制 - `foc_current` 扭矩控制 - [见文档](https://docs.simplefoc.com/motion_control)
> - 新的电流检测对齐程序 [#422](https://github.com/simplefoc/Arduino-FOC/pull/422) - [见文档](https://docs.simplefoc.com/current_sense_align)
>   - 支持步进电机
>   - 更加稳健和可靠
>   - 更加详细和信息丰富
> - 支持无中断的霍尔传感器 [#424](https://docs.simplefoc.com/https://github.com/simplefoc/Arduino-FOC/pull/424) - [见文档](hall_sensors)
> - 文档
>   - 常见问题的调试简要指南
>   - 库中单位的简要指南 - [见文档](https://docs.simplefoc.com/library_units)
> - 查看 v2.3.4 的完整错误修复和新功能列表 [修复和 PRs](https://github.com/simplefoc/Arduino-FOC/milestone/11)


## Arduino *SimpleFOClibrary* v2.3.4

<p align="">
<a href="https://youtu.be/Y5kLeqTc6Zk">
<img src="https://docs.simplefoc.com/extras/Images/youtube.png"  height="320px">
</a>
</p>

该视频演示了 *Simple**FOC**library* 的基本用法、电子连接，并展示了其功能。

### 特性
- **简单安装**：
   - Arduino IDE：集成 Arduino 库管理器
   - PlatformIO
- **开源**：完整代码和文档可在 GitHub 上获取
- **目标**：
   - 支持尽可能多的 [传感器](https://docs.simplefoc.com/position_sensors) + [电机](https://docs.simplefoc.com/motors) + [驱动器](https://docs.simplefoc.com/drivers) + [电流检测](https://docs.simplefoc.com/current_sense) 组合。
   - 提供最新和深入的文档，包括 API 参考和示例
- **易于设置和配置**：
   - 简单的硬件配置
   - 每个硬件组件都是一个 C++ 对象（易于理解）
   - 易于 [调节控制回路](https://docs.simplefoc.com/motion_control)
   - [*Simple**FOC**Studio*](https://docs.simplefoc.com/studio) 配置 GUI 工具
   - 内置通信和监控
- **跨平台**：
   - 从一个微控制器系列到另一个系列的无缝代码迁移
   - 支持多种 [MCU 架构](https://docs.simplefoc.com/microcontrollers)：
      - Arduino: UNO R4、UNO、MEGA、DUE、Leonardo、Nano、Nano33 等
      - STM32
      - ESP32
      - Teensy
      - 还有更多 ...

<p align=""> <img src="https://docs.simplefoc.com/extras/Images/uno_l6234.jpg"  height="170px">  <img src="https://docs.simplefoc.com/extras/Images/hmbgc_v22.jpg" height="170px">  <img src="https://docs.simplefoc.com/extras/Images/foc_shield_v13.jpg"  height="170px"></p>

## 文档
完整的 API 代码文档以及示例项目和逐步指南可以在我们的 [文档网站](https://docs.simplefoc.com/) 找到。

![image](https://user-images.githubusercontent.com/36178713/168475410-105e4e3d-082a-4015-98ff-d380c7992dfd.png)

## 入门
根据您是否希望将此库用作即插即用的 Arduino 库，或者希望深入了解算法并进行更改，有两种安装此代码的方法。

- 完整库安装 [文档](https://docs.simplefoc.com/library_download)
- PlatformIO [文档](https://docs.simplefoc.com/library_platformio)

### 将 Arduino *SimpleFOClibrary* 安装到 Arduino IDE
#### Arduino 库管理器
获取库最简单的方法是直接使用 Arduino IDE 及其集成的库管理器。
- 打开 Arduino IDE，点击 `工具 > 管理库...` 启动 Arduino 库管理器。
- 搜索 `Simple FOC` 库并安装最新版本。
- 重新打开 Arduino IDE，您应该在 `文件 > 示例 > Simple FOC` 中看到库示例。

#### 使用 GitHub 网站
- 前往 [GitHub 仓库](https://github.com/simplefoc/Arduino-FOC)
- 首先点击 `Clone or Download > Download ZIP`。
- 解压缩并将其放入 `Arduino Libraries` 文件夹。Windows: `文档 > Arduino > libraries`。
- 重新打开 Arduino IDE，您应该在 `文件 > 示例 > Simple FOC` 中看到库示例。

#### 使用终端
- 打开终端并运行
```sh  
cd #Arduino libraries folder
git clone https://github.com/simplefoc/Arduino-FOC.git
```
- 重新打开 Arduino IDE，您应该在 `文件 > 示例 > Simple FOC` 中看到库示例。

## 社区与贡献

对于有关潜在实现、应用、支持硬件等问题，请访问我们的 [社区论坛](https://community.simplefoc.com) 或我们的 [Discord 服务器](https://discord.gg/kWBwuzY32n)。

听取实施代码的人的故事/问题/建议总是很有帮助，您可能会发现那里已经有很多问题得到了回答！

### GitHub 问题与拉取请求

如果您在代码方面遇到问题或有建议，请随时留下问题！

欢迎提交拉取请求，但请先在 [社区论坛](https://community.simplefoc.com) 中讨论！

如果您希望为此项目做出贡献，但对 GitHub 不太熟悉，请不要担心，通过在社区论坛发帖、提交 GitHub 问题或在我们的 Discord 服务器上告知我们即可。

如果您熟悉，我们接受对开发分支的拉取请求！

## Arduino 代码示例
这是一个简单的 Arduino 代码示例，实现了带编码器的 BLDC 电机的速度控制程序。

注意：此程序使用所有默认控制参数。

```cpp
#include <SimpleFOC.h>

//  BLDCMotor( pole_pairs )
BLDCMotor motor = BLDCMotor(11);
//  BLDCDriver( pin_pwmA, pin_pwmB, pin_pwmC, enable (optional) )
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8);
//  Encoder(pin_A, pin_B, CPR)
Encoder encoder = Encoder(2, 3, 2048);
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

void setup() {  
  // 初始化编码器硬件
  encoder.init();
  // 启用硬件中断
  encoder.enableInterrupts(doA, doB);
  // 将电机与传感器链接
  motor.linkSensor(&encoder);
  
  // 电源电压 [V]
  driver.voltage_power_supply = 12;
  // 初始化驱动器硬件
  driver.init();
  // 链接驱动器
  motor.linkDriver(&driver);

  // 设置使用的控制回路类型
  motor.controller = MotionControlType::velocity;
  // 初始化电机
  motor.init();
  
  // 对齐编码器并启动 FOC
  motor.initFOC();
}

void loop() {
  // FOC 算法函数
  motor.loopFOC();

  // 速度控制回路函数
  // 设置目标速度为 2rad/s
  motor.move(2);
}
```
您可以在 [SimpleFOC 文档](https://docs.simplefoc.com/) 中找到更多详细信息。

## 示例项目
以下是一些 *Simple**FOC**library* 和 *Simple**FOC**Shield* 应用示例。 
<p align="center">
<a href="https://youtu.be/Ih-izQyXJCI">
<img src="https://docs.simplefoc.com/extras/Images/youtube_pendulum.png"  height="200px" >
</a>
<a href="https://youtu.be/xTlv1rPEqv4">
<img src="https://docs.simplefoc.com/extras/Images/youtube_haptic.png"  height="200px" >
</a>
<a href="https://youtu.be/RI4nNMF608I">
<img src="https://docs.simplefoc.com/extras/Images/youtube_drv8302.png"  height="200px" >
</a>
<a href="https://youtu.be/zcb86TRxTxc">
<img src="https://docs.simplefoc.com/extras/Images/youtube_stepper.png"  height="200px" >
</a>
</p>

## 引用 *SimpleFOC*

我们很高兴 *Simple**FOC**library* 已被用作多个研究项目的组成部分，并已出现在多篇科学论文中。我们希望这一趋势在项目成熟和变得更加稳健的过程中能够继续下去！
关于 *Simple**FOC*** 的简短论文已在《开放源代码软件杂志》上发表：
<p>
  <b><i>SimpleFOC</i></b>: 一种用于控制无刷直流电机（BLDC）和步进电机的场定向控制（FOC）库。<br>
  A. Skuric, HS. Bank, R. Unger, O. Williams, D. González-Reyes<br>
《开放源代码软件杂志》，7(74)，4232，https://doi.org/10.21105/joss.04232
</p>

如果您有兴趣在您的研究中引用 *Simple**FOC**library* 或 *Simple**FOC**project* 的其他组件，我们建议您引用我们的论文：

```bib
@article{simplefoc2022,
  doi = {10.21105/joss.04232},
  url = {https://doi.org/10.21105/joss.04232},
  year = {2022},
  publisher = {The Open Journal},
  volume = {7},
  number = {74},
  pages = {4232},
  author = {Antun Skuric and Hasan Sinan Bank and Richard Unger and Owen Williams and David González-Reyes},
  title = {SimpleFOC: A Field Oriented Control (FOC) Library for Controlling Brushless Direct Current (BLDC) and Stepper Motors},
  journal = {Journal of Open Source Software}
}
```

```
