#include <MessageSender.h>
#include <Helper.h>

#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <Filter.h>
#include <AP_Baro.h>

#include <GCS_MAVLink.h>
#include <GCS_Console.h>
#include <mavlink.h>
#include <mavlink_types.h>
#include <protocol.h>

#define STABILIZED_MODE 0

const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
static uint32_t timer;
static uint32_t baro_timer;

AP_InertialSensor_MPU6000 ins;

AP_Baro_MS5611 baro(&AP_Baro_MS5611::spi);

float roll, pitch, yaw;

Vector3f gyro, accel;

float h_Baro;




int8_t flightMode = STABILIZED_MODE;
int mCount = 0;

void setup()
{
  hal.uartA->begin(57600, 256, 256);
  mavlink_comm_0_port = hal.uartA;
  
  char hello[] = "Hello\r\n";
  
  
  hal.scheduler->delay(1000);
  hal.gpio->pinMode(63, GPIO_OUTPUT);
  hal.gpio->write(63,1);
  
  baro.init();
  baro.calibrate();
  baro_timer=hal.scheduler->micros();
  
  ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ,NULL);
  
  Vector3f accel_offsets;
  accel_offsets.x = 0.160347;
  accel_offsets.y = 0.2228888;
  accel_offsets.z = 0.6614504;
  
  hal.scheduler->suspend_timer_procs();
  ins.dmp_init();
  ins.set_accel_offsets(accel_offsets);
  ins.push_accel_offsets_to_dmp();
  ins.dmp_set_bias_from_no_motion();
  hal.scheduler->resume_timer_procs();
}

void loop()
{
 while (ins.num_samples_available() == 0);
 ins.update();
ins.quaternion.to_euler(&roll, &pitch, &yaw);

gyro = ins.get_gyro();
float angular_rate_p = gyro.x, angular_rate_q = gyro.y, angular_rate_r = gyro.z;
accel = ins.get_accel();

if((hal.scheduler->micros() - baro_timer) > 100000UL) {
  baro_timer = hal.scheduler->micros();
  baro.read();
}
h_Baro = baro.get_altitude();

if(mCount>100){
  
  hal.console->printf_P(PSTR("\n Barometric Altitude = %f \n"), h_Baro);
  
  mCount=0;
}
mCount++;
}

AP_HAL_MAIN();
