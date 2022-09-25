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
  while(!hal.console -> available())
  {
    while(ins.num_samples_available()==0);
    ins.update();
    accel = ins.get_accel();
    gyro = ins.get_gyro();
    hal.console -> print("Accel :");
    hal.console ->print("\t x = "); hal.console ->print(accel.x);
    hal.console -> print("\t y = "); hal.console ->print(accel.y);
    hal.console -> print("\t z = "); hal.console ->print(accel.z);

    
    hal.console -> print("\tGyro : " );
    hal.console -> print("\t p = "); hal.console ->print(ToDeg(gyro.x));
    hal.console -> print("\t q = "); hal.console -> print(ToDeg(gyro.y));
    hal.console -> print("\t r = " ); hal.console -> print(ToDeg(gyro.z));
    hal.console -> print("\n");
    hal.scheduler -> delay(1000);
  }
}

void loop()
{
}

AP_HAL_MAIN();
