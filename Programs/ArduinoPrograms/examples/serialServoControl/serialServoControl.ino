/************************************************
 * By william Brines November 13, 2018.
 * This program oporates the servos on the 
 * 16-channel PCA9685 servo driver. By using 
 * serial input to get the channel and pulse width for the 
 * servo.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);

double pulse = -1000;
int8_t channle = -10;

void setup() 
{
  Serial.begin(9600);
  Serial.println("Initalizing servo driver....");
  
  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  delay(10);
  
  Serial.println("Input servo<servo chanal> pulse<pulse>");
  
}

void loop() 
{

  // send data only when you receive data:
  if (Serial.available() > 0) 
  {
    if(channle == -10)
    {
      
      // read the incoming byte:
      channle = Serial.parseInt();

    }
    else
    {
      // read the incoming byte:
      pulse = Serial.parseFloat();
    }

    if(pulse != -1000 && channle != -10)
    {
      
        Serial.print("Channel<");
        Serial.print(channle);
        Serial.print("> Pulse<");
        Serial.print(pulse);
        Serial.println(">");

        pwm.setPWM(channle, 0, pulse);
        
        pulse = -1000;
        channle = -10;
    }
  }
}

