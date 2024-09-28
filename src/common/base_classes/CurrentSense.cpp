#include "CurrentSense.h"
#include "../../communication/SimpleFOCDebug.h"

// 获取电流幅度
//   - 绝对值  - 如果没有提供电气角
//   - 符号值  - 如果提供了角度
float CurrentSense::getDCCurrent(float motor_electrical_angle) {
    // 读取相电流
    PhaseCurrent_s current = getPhaseCurrents();
    
    // 计算克拉克变换
    ABCurrent_s ABcurrent = getABCurrents(current);

    // 电流符号 - 如果没有提供电机角度，则幅度始终为正
    float sign = 1;

    // 如果提供了电机角度，函数返回电流的符号值
    // 确定电流的符号
    // sign(atan2(current.q, current.d)) 与 c.q > 0 ? 1 : -1 相同  
    if (motor_electrical_angle) {
        float ct;
        float st;
        _sincos(motor_electrical_angle, &st, &ct);
        sign = (ABcurrent.beta * ct - ABcurrent.alpha * st) > 0 ? 1 : -1;  
    }
    // 返回电流幅度
    return sign * _sqrt(ABcurrent.alpha * ABcurrent.alpha + ABcurrent.beta * ABcurrent.beta);
}

// 与FOC算法一起使用的函数
//   从相电流计算DQ电流
//   - 函数计算相电流的帕克和克拉克变换 
//   - 内部使用getPhaseCurrents和getABCurrents
DQCurrent_s CurrentSense::getFOCCurrents(float angle_el) {
    // 读取相电流
    PhaseCurrent_s current = getPhaseCurrents();

    // 计算克拉克变换
    ABCurrent_s ABcurrent = getABCurrents(current);
    
    // 计算帕克变换
    DQCurrent_s return_current = getDQCurrents(ABcurrent, angle_el);

    return return_current;
}

// 与FOC算法一起使用的函数
//   从相电流计算阿尔法贝塔电流
//   - 函数计算相电流的克拉克变换
ABCurrent_s CurrentSense::getABCurrents(PhaseCurrent_s current) {

    // 检查驱动程序是否为StepperDriver的实例
    // 如果是，则无需进行克拉克变换
    if (driver_type == DriverType::Stepper) {
        ABCurrent_s return_ABcurrent;
        return_ABcurrent.alpha = current.a;
        return_ABcurrent.beta = current.b;
        return return_ABcurrent;
    }

    // 否则，它是BLDC电机，计算克拉克变换
    float i_alpha, i_beta;
    if (!current.c) {
        // 如果只测量了两个电流
        i_alpha = current.a;  
        i_beta = _1_SQRT3 * current.a + _2_SQRT3 * current.b;
    } else if (!current.a) {
        // 如果只测量了两个电流
        float a = -current.c - current.b;
        i_alpha = a;  
        i_beta = _1_SQRT3 * a + _2_SQRT3 * current.b;
    } else if (!current.b) {
        // 如果只测量了两个电流
        float b = -current.a - current.c;
        i_alpha = current.a;  
        i_beta = _1_SQRT3 * current.a + _2_SQRT3 * b;
    } else {
        // 使用恒等式a + b + c = 0进行信号滤波。假设测量误差是正态分布的。
        float mid = (1.f / 3) * (current.a + current.b + current.c);
        float a = current.a - mid;
        float b = current.b - mid;
        i_alpha = a;
        i_beta = _1_SQRT3 * a + _2_SQRT3 * b;
    }

    ABCurrent_s return_ABcurrent;
    return_ABcurrent.alpha = i_alpha;
    return_ABcurrent.beta = i_beta;
    return return_ABcurrent;
}

// 与FOC算法一起使用的函数
//   从阿尔法贝塔电流和电气角计算D和Q电流
//   - 函数计算相电流的克拉克变换
DQCurrent_s CurrentSense::getDQCurrents(ABCurrent_s current, float angle_el) {
    // 计算帕克变换
    float ct;
    float st;
    _sincos(angle_el, &st, &ct);
    DQCurrent_s return_current;
    return_current.d = current.alpha * ct + current.beta * st;
    return_current.q = current.beta * ct - current.alpha * st;
    return return_current;
}

/**
	驱动程序链接到电流传感器
*/
void CurrentSense::linkDriver(FOCDriver* _driver) {
    driver = _driver;
    // 保存驱动程序类型以便更方便访问
    driver_type = driver->type();
}

void CurrentSense::enable() {
    // 此处不执行任何操作，但可以覆盖此函数
};

void CurrentSense::disable() {
    // 此处不执行任何操作，但可以覆盖此函数
};

// 函数对齐电流传感器与电机驱动程序
// 如果所有引脚连接良好，实际上没有必要执行这些操作！- 可以避免
// 返回标志
// 0 - 失败
// 1 - 成功且没有更改
// 2 - 成功但引脚重新配置
// 3 - 成功但增益反转
// 4 - 成功但引脚重新配置且增益反转
// 重要提示，此函数可以在子类中重写
int CurrentSense::driverAlign(float voltage, bool modulation_centered) {
        
    int exit_flag = 1;
    if (skip_align) return exit_flag;

    if (!initialized) return 0;

    // 检查是步进电机还是BLDC 
    if (driver_type == DriverType::Stepper)
        return alignStepperDriver(voltage, (StepperDriver*)driver, modulation_centered);
    else
        return alignBLDCDriver(voltage, (BLDCDriver*)driver, modulation_centered);
}

// 辅助函数读取和平均相电流
PhaseCurrent_s CurrentSense::readAverageCurrents(int N) {
    PhaseCurrent_s c = getPhaseCurrents();
    for (int i = 0; i < N; i++) {
        PhaseCurrent_s c1 = getPhaseCurrents();
        c.a = c.a * 0.6f + 0.4f * c1.a;
        c.b = c.b * 0.6f + 0.4f * c1.b;
        c.c = c.c * 0.6f + 0.4f * c1.c;
        _delay(3);
    }
    return c;
};

// 函数对齐电流传感器与电机驱动程序
// 如果所有引脚连接良好，实际上没有必要执行这些操作！- 可以避免
// 返回标志
// 0 - 失败
// 1 - 成功且没有更改
// 2 - 成功但引脚重新配置
// 3 - 成功但增益反转
// 4 - 成功但引脚重新配置且增益反转
int CurrentSense::alignBLDCDriver(float voltage, BLDCDriver* bldc_driver, bool modulation_centered) {
        
    bool phases_switched = 0;
    bool phases_inverted = 0;
    
    float zero = 0;
    if (modulation_centered) zero = driver->voltage_limit / 2.0;

    // 设置相A为活动，相B和C为关闭
    // 300毫秒的升压
    for (int i = 0; i < 100; i++) {
        bldc_driver->setPwm(voltage / 100.0 * ((float)i) + zero, zero, zero);
        _delay(3);
    }
    _delay(500);
    PhaseCurrent_s c_a = readAverageCurrents();
    bldc_driver->setPwm(zero, zero, zero);
    // 检查电流是否过低（低于100mA） 
    // TODO 根据ADC分辨率计算100mA阈值
    // 如果是，则抛出错误并返回0
    // 电流传感器未连接或电流过低，无法进行校准（应提高motor.voltage_sensor_align）
    if ((fabs(c_a.a) < 0.1f) && (fabs(c_a.b) < 0.1f) && (fabs(c_a.c) < 0.1f)) {
        SIMPLEFOC_DEBUG("CS: Err too low current, rise voltage!");
        return 0; // 测量电流过低
    }

    // 现在我们必须确定 
    // 1) 哪个引脚对应于BLDC驱动程序的哪个相
    // 2) 如果测量的电流具有良好的极性
    // 
    // > 当我们对驱动程序的相A施加电压时，我们期望测量的是相A的电流I
    //   和相B的电流-I/2，以及相C的电流I/2

    // 找到c_a中的最大幅度
    // 并确保它比其他两个大约高2倍（至少1.5倍）
    float ca[3] = {fabs(c_a.a), fabs(c_a.b), fabs(c_a.c)};
    uint8_t max_i = -1; // 最大索引
    float max_c = 0; // 最大电流
    float max_c_ratio = 0; // 最大电流比
    for (int i = 0; i < 3; i++) {
        if (!ca[i]) continue; // 电流未测量
        if (ca[i] > max_c) {
            max_c = ca[i];
            max_i = i;
            for (int j = 0; j < 3; j++) {
                if (i == j) continue;
                if (!ca[j]) continue; // 电流未测量
                float ratio = max_c / ca[j];
                if (ratio > max_c_ratio) max_c_ratio = ratio;
            }
        }
    }

    // 检查电流幅度比
    // 1) 如果有一个电流大约比其他两个高2倍
    //    这就是相A的电流
    // 2) 如果最大电流至少比其他两个高1.5倍
    //    我们有两种情况：
    //    - 要么我们只测量了两个电流，第三个未测量 - 那么相A未测量
    //    - 或者电流传感器未正确连接

    if (max_c_ratio >= 1.5f) {
        switch (max_i) {
            case 1: // 相B是最大电流
                SIMPLEFOC_DEBUG("CS: Switch A-B");
                // 交换相A和相B
                _swap(pinA, pinB);
                _swap(offset_ia, offset_ib);
                _swap(gain_a, gain_b);
                _swap(c_a.b, c_a.b);
                phases_switched = true; // 标记引脚已交换
                break;
            case 2: // 相C是最大电流
                SIMPLEFOC_DEBUG("CS: Switch A-C");
                // 交换相A和相C
                _swap(pinA, pinC);
                _swap(offset_ia, offset_ic);
                _swap(gain_a, gain_c);
                _swap(c_a.a, c_a.c);
                phases_switched = true; // 标记引脚已交换
                break;
        }
        // 检查电流是否为负，如果是，则反转增益
        if (_sign(c_a.a) < 0) {
            SIMPLEFOC_DEBUG("CS: Inv A");
            gain_a *= -1;
            phases_inverted = true; // 标记引脚已反转
        }
    } else if (_isset(pinA) && _isset(pinB) && _isset(pinC)) {
        // 如果所有三个电流都被测量且没有一个显著更高
        // 我们有电流传感器的问题
        SIMPLEFOC_DEBUG("CS: Err A - all currents same magnitude!");
        return 0;
    } else { // 相A未测量，所以将_NC连接到相A
        if (_isset(pinA) && !_isset(pinB)) {
            SIMPLEFOC_DEBUG("CS: Switch A-(B)NC");
            _swap(pinA, pinB);
            _swap(offset_ia, offset_ib);
            _swap(gain_a, gain_b);
            _swap(c_a.b, c_a.b);
            phases_switched = true; // 标记引脚已交换
        } else if (_isset(pinA) && !_isset(pinC)) {
            SIMPLEFOC_DEBUG("CS: Switch A-(C)NC");
            _swap(pinA, pinC);
            _swap(offset_ia, offset_ic);
            _swap(gain_a, gain_c);
            _swap(c_a.b, c_a.c);
            phases_switched = true; // 标记引脚已交换
        }
    }
    // 此时，相A的电流感应可以是：
    // - 与驱动程序相A对齐
    // - 或者相A未测量而_NC连接到相A
    //
    // 在任何情况下，A都已完成，现在我们必须检查相B和C 

    // 设置相B为活动，相A和C为关闭
    // 300毫秒的升压
    for (int i = 0; i < 100; i++) {
        bldc_driver->setPwm(zero, voltage / 100.0 * ((float)i) + zero, zero);
        _delay(3);
    }
    _delay(500);
    PhaseCurrent_s c_b = readAverageCurrents();
    bldc_driver->setPwm(zero, zero, zero);

    // 检查相B
    // 找到c_b中的最大幅度
    // 并确保它比其他两个大约高2倍（至少1.5倍）
    float cb[3] = {fabs(c_b.a), fabs(c_b.b), fabs(c_b.c)};
    max_i = -1; // 最大索引
    max_c = 0; // 最大电流
    max_c_ratio = 0; // 最大电流比
    for (int i = 0; i < 3; i++) {
        if (!cb[i]) continue; // 电流未测量
        if (cb[i] > max_c) {
            max_c = cb[i];
            max_i = i;
            for (int j = 0; j < 3; j++) {
                if (i == j) continue;
                if (!cb[j]) continue; // 电流未测量
                float ratio = max_c / cb[j];
                if (ratio > max_c_ratio) max_c_ratio = ratio;
            }
        }
    }
    if (max_c_ratio >= 1.5f) {
        switch (max_i) {
            case 0: // 相A是最大电流
                // 这是一个错误，因为相A已经对齐
                SIMPLEFOC_DEBUG("CS: Err align B");
                return 0;
            case 2: // 相C是最大电流
                SIMPLEFOC_DEBUG("CS: Switch B-C");
                _swap(pinB, pinC);
                _swap(offset_ib, offset_ic);
                _swap(gain_b, gain_c);
                _swap(c_b.b, c_b.c);
                phases_switched = true; // 标记引脚已交换
                break;
        }
        // 检查电流是否为负，如果是，则反转增益
        if (_sign(c_b.b) < 0) {
            SIMPLEFOC_DEBUG("CS: Inv B");
            gain_b *= -1;
            phases_inverted = true; // 标记引脚已反转
        }
    } else if (_isset(pinB) && _isset(pinC)) {
        // 如果所有三个电流都被测量且没有一个显著更高
        // 我们有电流传感器的问题
        SIMPLEFOC_DEBUG("CS: Err B - all currents same magnitude!");
        return 0;
    } else { // 相B未测量，所以将_NC连接到相B
        if (_isset(pinB) && !_isset(pinC)) {
            SIMPLEFOC_DEBUG("CS: Switch B-(C)NC");
            _swap(pinB, pinC);
            _swap(offset_ib, offset_ic);
            _swap(gain_b, gain_c);
            _swap(c_b.b, c_b.c);
            phases_switched = true; // 标记引脚已交换
        }
    }
    // 此时，相A和B的电流感应可以是：
    // - 与驱动程序相A和B对齐
    // - 或者相A和B未测量而_NC连接到相A和B
    //
    // 在任何情况下，A和B都已完成，现在我们必须检查相C
    // 如果相C被测量（不是_NC），它也已对齐
    // 我们必须检查电流是否为负，如果是，则反转增益
    if (_isset(pinC)) {
        if (_sign(c_b.c) > 0) { // 预期电流为-I/2（如果相A和B已对齐且C具有正确极性）
            SIMPLEFOC_DEBUG("CS: Inv C");
            gain_c *= -1;
            phases_inverted = true; // 标记引脚已反转
        }
    }

    // 构建返回标志
    // 如果相位已交换返回2
    // 如果增益已反转返回3
    // 如果两者都返回4
    uint8_t exit_flag = 1;
    if (phases_switched) exit_flag += 1;
    if (phases_inverted) exit_flag += 2;
    return exit_flag;
}

// 函数对齐电流传感器与电机驱动程序
// 如果所有引脚连接良好，实际上没有必要执行这些操作！- 可以避免
// 返回标志
// 0 - 失败
// 1 - 成功且没有更改
// 2 - 成功但引脚重新配置
// 3 - 成功但增益反转
// 4 - 成功但引脚重新配置且增益反转
int CurrentSense::alignStepperDriver(float voltage, StepperDriver* stepper_driver, bool modulation_centered) {
    
    _UNUSED(modulation_centered);

    bool phases_switched = 0;
    bool phases_inverted = 0;

    if (!_isset(pinA) || !_isset(pinB)) {
        SIMPLEFOC_DEBUG("CS: Pins A & B not specified!");
        return 0;
    }

    // 设置相A为活动，相B为关闭
    // 升压300毫秒
    for (int i = 0; i < 100; i++) {
        stepper_driver->setPwm(voltage / 100.0 * ((float)i), 0);
        _delay(3);
    }
    _delay(500);
    PhaseCurrent_s c = readAverageCurrents();
    // 禁用相
    stepper_driver->setPwm(0, 0);        
    if (fabs(c.a) < 0.1f && fabs(c.b) < 0.1f) {
        SIMPLEFOC_DEBUG("CS: Err too low current!");
        return 0; // 测量电流过低
    }
    // 对齐相A
    // 1) 只能测量一个相，因此我们首先通过比较幅度来测量哪个ADC引脚对应于相A
    if (fabs(c.a) < fabs(c.b)) {
        SIMPLEFOC_DEBUG("CS: Switch A-B");
        // 交换相A和相B
        _swap(pinA, pinB);
        _swap(offset_ia, offset_ib);
        _swap(gain_a, gain_b);
        phases_switched = true; // 标记引脚已交换
    }
    // 2) 检查测量的电流a是否为正，如果不是则反转
    if (c.a < 0) {
        SIMPLEFOC_DEBUG("CS: Inv A");
        gain_a *= -1;
        phases_inverted = true; // 标记引脚已反转
    }

    // 此时，驱动程序的相A已与ADC引脚A对齐
    // 引脚B应为相B

    // 设置相B为活动，相A为关闭
    // 升压300毫秒
    for (int i = 0; i < 100; i++) {
        stepper_driver->setPwm(0, voltage / 100.0 * ((float)i));
        _delay(3);
    }
    _delay(500);
    c = readAverageCurrents();
    stepper_driver->setPwm(0, 0);

    // 相B应已对齐
    // 1) 我们只需验证它是否已被测量
    if (fabs(c.b) < 0.1f) {
        SIMPLEFOC_DEBUG("CS: Err too low current on B!");
        return 0; // 测量电流过低
    }
    // 2) 检查测量的电流b是否为正，如果不是则反转
    if (c.b < 0) {
        SIMPLEFOC_DEBUG("CS: Inv B");
        gain_b *= -1;
        phases_inverted = true; // 标记引脚已反转
    }

    // 构建返回标志
    // 如果成功且没有更改返回1 
    // 如果相位已交换返回2
    // 如果增益已反转返回3
    // 如果两者都返回4
    uint8_t exit_flag = 1;
    if (phases_switched) exit_flag += 1;
    if (phases_inverted) exit_flag += 2;
    return exit_flag;
}
