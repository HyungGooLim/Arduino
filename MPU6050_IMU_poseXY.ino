#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>

const AP_HAL :: HAL & hal = AP_HAL_AVR_APM2;

AP_InertialSensor_MPU6000 ins ;

void setup()
{
  hal.console -> println("AP_InertialSensor startup...");
  ins.init(AP_InertialSensor::COLD_START , AP_InertialSensor::RATE_100HZ,NULL);
  hal.console ->println("\n Complete.Reading:");
  run_test();
}

void run_test()
{
  Vector3f accel;
  Vector3f gyro;
  float Pitch;
  float Roll;
  float Yaw;
  int Deno_P;
  int Deno_R;
  
  while(!hal.console -> available())
  {
    while(ins.num_samples_available()==0);
    ins.update();
    accel = ins.get_accel();
    gyro = ins.get_gyro();
    Deno_P = sq(accel.y)+sq(accel.z);
    Deno_R = sq(accel.x)+sq(accel.z);
    
    Pitch = atan(accel.x/pow(Deno_P ,0.5));
    Roll = atan(-1*accel.y/pow(Deno_R ,0.5));
    
    hal.console -> print("Pitch:\t\t");     hal.console -> print("Roll:\n");
    hal.console ->print(ToDeg(Pitch));       hal.console -> print("\t\t");  hal.console ->print(ToDeg(Roll)); 
    hal.console -> print("\n");
    hal.scheduler -> delay(1000);
  }
}

void loop()
{
}

AP_HAL_MAIN();
