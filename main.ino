/* PID controller two fan retrofit for silent operation of testing devices with prevention of over-cooling
 *  Gasper Zakelj Fotona d.o.o. Stegne 7
 *  13.6.2022
 *  
 *  Z dvema ventilatorjema namesto šestih na Dynamis-u, pri max load (neodym laser, 10 minutno testiranje laserskih glav) 
 *  pri setpointu 38 stopinj, dosežemo temperaturo do največ 41.5 stopinj. Kar je v toleranci.
 *  
 */

#include <PID_v1.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS  4      //senzorji so priključeni na port 4
#define PWM_pin 6            // pwm pin

#define Kp 300
#define Ki 0 
#define Kd 0 

double temp_setpoint = 38.0;// glavni setpoint
double tempC = 20.0;

double PID_output;

PID _PID(&tempC, &PID_output, &temp_setpoint, Kp, Ki, Kd, REVERSE);

OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature temp_senzor(&oneWire);



void setup() {
  pinMode(PWM_pin, OUTPUT);
  Serial.begin(9600);
  Serial.println("Hi");
  temp_senzor.begin();
  _PID.SetOutputLimits(0, 255); //pwm omejitve
  _PID.SetMode(AUTOMATIC); // turn on
}

void loop() {
  // Koda Gasper Zakelj
  temp_senzor.requestTemperatures();            // Send the command to get temperatures
  tempC = temp_senzor.getTempCByIndex(0); // prebere temperaturo
  _PID.Compute();
  analogWrite(PWM_pin, PID_output);
  Serial.print(tempC);
  Serial.print("\t");
  Serial.print(PID_output);//Serial.print(map(PID_output, 0, 255, 37, 39));
  Serial.print("\r\n");
  delay(500);
}
