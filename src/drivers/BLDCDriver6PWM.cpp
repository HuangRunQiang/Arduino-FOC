#include "BLDCDriver6PWM.h"

BLDCDriver6PWM::BLDCDriver6PWM(int phA_h,int phA_l,int phB_h,int phB_l,int phC_h,int phC_l, int en){
  // Pin initialization
  pwmA_h = phA_h;
  pwmB_h = phB_h;
  pwmC_h = phC_h;
  pwmA_l = phA_l;
  pwmB_l = phB_l;
  pwmC_l = phC_l;

  // enable_pin pin
  enable_pin = en;

  // default power-supply value
  voltage_power_supply = DEF_POWER_SUPPLY;
  voltage_limit = NOT_SET;
 
  // dead zone initial - 2%
  dead_zone = 0.02;

}

// enable motor driver
void  BLDCDriver6PWM::enable(){
    // enable_pin the driver - if enable_pin pin available
    if ( enable_pin != NOT_SET ) digitalWrite(enable_pin, HIGH);
    // set zero to PWM
    setPwm(0, 0, 0);
}

// disable motor driver
void BLDCDriver6PWM::disable()
{
  // set zero to PWM
  setPwm(0, 0, 0);
  // disable the driver - if enable_pin pin available
  if ( enable_pin != NOT_SET ) digitalWrite(enable_pin, LOW);

}

// init hardware pins   
int BLDCDriver6PWM::init() {

  // PWM pins
  pinMode(pwmA_l, OUTPUT);
  pinMode(pwmB_h, OUTPUT);
  pinMode(pwmC_h, OUTPUT);
  pinMode(pwmA_l, OUTPUT);
  pinMode(pwmB_l, OUTPUT);
  pinMode(pwmC_l, OUTPUT);
  if(enable_pin != NOT_SET) pinMode(enable_pin, OUTPUT);


  // sanity check for the voltage limit configuration
  if(voltage_limit == NOT_SET || voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;

  // configure 6pwm 
  // hardware specific function - depending on driver and mcu
  return _configure6PWM(pwm_frequency, dead_zone, pwmA_h,pwmA_l, pwmB_h,pwmB_l, pwmC_h,pwmC_l);
}


// Set voltage to the pwm pin
void BLDCDriver6PWM::setPwm(float Ua, float Ub, float Uc) {  
  // limit the voltage in driver
  Ua = _constrain(Ua, -voltage_limit, voltage_limit);
  Ub = _constrain(Ub, -voltage_limit, voltage_limit);
  Uc = _constrain(Uc, -voltage_limit, voltage_limit);    
  // calculate duty cycle
  // limited in [0,1]
  float dc_a = _constrain(Ua / voltage_power_supply, 0 , 1 );
  float dc_b = _constrain(Ub / voltage_power_supply, 0 , 1 );
  float dc_c = _constrain(Uc / voltage_power_supply, 0 , 1 );
  // hardware specific writing
  // hardware specific function - depending on driver and mcu
  _writeDutyCycle6PWM(dc_a, dc_b, dc_c, dead_zone, pwmA_h,pwmA_l, pwmB_h,pwmB_l, pwmC_h,pwmC_l);
}