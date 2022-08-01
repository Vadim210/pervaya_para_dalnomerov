#include <Wire.h>

#include <VL53L0X.h>

VL53L0X sensor;

VL53L0X sensor2;



int a;

int b;

uint32_t sensorTime = 100000;
float sensorSignalRate = 0.35;
uint8_t periodPreRange = 14, periodFinalRange = 10;

void setup()
{



pinMode(7, OUTPUT);

pinMode(6, OUTPUT);

digitalWrite(6, LOW);

digitalWrite(7, LOW);



delay(500);

Wire.begin();

Serial.begin (115200);

digitalWrite(6, HIGH);

delay(150);

Serial.println("00");

sensor.init(true);

Serial.println("01");

delay(100);

sensor.setAddress((uint8_t)01);

Serial.println("02");

digitalWrite(7, HIGH);

delay(150);

sensor2.init(true);

Serial.println("03");

delay(100);

sensor2.setAddress((uint8_t)02);

Serial.println("04");


Serial.println("addresses set");

sensor.setTimeout(100);
sensor2.setTimeout(100);

sensor.setMeasurementTimingBudget(sensorTime);
sensor2.setMeasurementTimingBudget(sensorTime);

sensor.setSignalRateLimit(sensorSignalRate);
sensor2.setSignalRateLimit(sensorSignalRate);

sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, periodPreRange);
sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, periodFinalRange);
sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, periodPreRange);
sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, periodFinalRange);

sensor.startContinuous();

sensor2.startContinuous();

}

void loop()

{

//if(!sensor.timeoutOccurred())
  a=sensor.readRangeContinuousMillimeters();

Serial.print("a:");
Serial.print(a);

Serial.println(" ");

//if(!sensor2.timeoutOccurred())
  b=sensor2.readRangeContinuousMillimeters();

Serial.print("b:");
Serial.print(b);

Serial.println(" ");

//delay(100);

}
