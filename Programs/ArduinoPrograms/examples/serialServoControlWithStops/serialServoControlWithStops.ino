/************************************************
 * By william Brines November 13, 2018.
 * This program oporates the servos on the 
 * 16-channel PCA9685 servo driver. By using 
 * serial input to get the channel and pulse width for the 
 * servo. This includes motion smothing and position limits
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

float servoPos[5];

void moveServo(const int, const float);
void smothMove(int, float, int, int);

void setup() 
{
  Serial.begin(9600);
  Serial.println("Initalizing servo driver....");
  
  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  delay(10);

  Serial.println("Seting servos to orgin....");
  moveServo(0, 440);
  servoPos[0] = 440;
 // moveServo(1, 200);
 // servoPos[1] = 200;
  moveServo(2, 320);
  servoPos[2] = 320;
  moveServo(3, 500);
  servoPos[3] = 500;
  moveServo(4, 110);
  servoPos[4] = 110;
  moveServo(5, 110);
  servoPos[6] = 110;
  
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

        Serial.print("Theretical end pos : ");
        
        Serial.println(servoPos[channle] + pulse);         
          
        smothMove(channle, pulse, 50, 1000);

        pulse = -1000;
        channle = -10;
    }
  }
}

void moveServo(int channle, float pos)
{
  switch(channle)
  {
    case 0:
      if(pos < 491 && pos > 189)
      {
        pwm.setPWM(channle, 0, pos);
        servoPos[channle] = pos;
      }
    break;
    case 1:
      if(pos < 451 && pos > 124)
      {
        pwm.setPWM(channle, 0, pos);
        servoPos[channle] = pos;
      }
    break;
    case 2:
      if(pos < 551 && pos > 159)
      {
        pwm.setPWM(channle, 0, pos);
        servoPos[channle] = pos;
      }
    break;
    case 3:
      if(pos < 576 && pos > 144)
      {
        pwm.setPWM(channle, 0, pos);
        servoPos[channle] = pos;
      }
    break;
    case 4:
      if(pos < 531 && pos > 109)
      {
        pwm.setPWM(channle, 0, pos);
        servoPos[channle] = pos;
      }
    break;
    case 5:
      if(pos < 526 && pos > 109)
      {
        pwm.setPWM(channle, 0, pos);
        servoPos[channle] = pos;
      }
    break;

    default:
    break;
  }
}

void smothMove(int channle, float dis,  int res, int t)
{
  int c = servoPos[channle];

  if(dis == 0) return;
  
  if(res <= 1)
  {
    moveServo(channle, dis);
    return;
  }
  
  Serial.print("Dis ");
  Serial.println(dis);

  Serial.print("C ");
  Serial.println(c);
  
  for(int i = 1; i <= t; i += t/res)
  {
    Serial.println(servoPos[channle]);
    moveServo(channle, c + ( (dis/2) * cos(((i*PI)/t)+PI)+(dis/2)));   
    
    Serial.print("Time ");
    Serial.println(t/res);    
    
    delay(t/res);
  }
  
  moveServo(channle, c + dis);


  Serial.print("Actual end pos : ");
  Serial.println(servoPos[channle]);
}

