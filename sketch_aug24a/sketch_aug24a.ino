int counter;

#include <SPI.h>
#include <AS5047P.h>

AS5047P as5047p(D4); // Chip select pin 9

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
counter = 0;

SPI.begin();
as5047p.initSPI();

 pinMode(D33, OUTPUT); 

}

void loop() {
  // put your main code here, to run repeatedly:
// Serial.print("hello ");
// Serial.print(counter++, DEC);
// Serial.println();

uint16_t angle = as5047p.readAngleDegree();
// Serial.print("Angle: ");
Serial.println(angle);
// Serial.println();

digitalWrite(D33, HIGH);
delay(100); 
digitalWrite(D33, LOW);
delay(100);
}
