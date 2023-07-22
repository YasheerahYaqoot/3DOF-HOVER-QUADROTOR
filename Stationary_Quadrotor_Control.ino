#include <Servo.h>
const int encoderPinA = 18;
const int encoderPinB = 19;
const int encoderPinC = 20;
const int encoderPinD = 21;

Servo back_prop;
Servo front_prop;
Servo right_prop;
Servo left_prop;

volatile long int currentPosition = 0;
double Pitch_Angle = 0.0;
volatile long int currentPosition1 = 0;
double Roll_Angle = 0.0; 
//float Total_angle;

float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;

float PID, PID1, pwmLeft, pwmRight, pwmFront, pwmBack, error, error1, previous_error, previous_error1;
float pid_p=0;
float pid_i=0;
float pid_d=0;
float pid_p1=0;
float pid_i1=0;
float pid_d1=0;
/////////////////PITCH PID CONSTANTS/////////////////
double kp=15.4;//3.55
double ki=0.002;//0.003
double kd=4.2;//2.05
/////////////////ROLL PID CONSTANTS/////////////////
double kp1=12;//3.55
double ki1=0.002;//0.003
double kd1=3;//2.05
///////////////////////////////////////////////

double throttle=1300; //initial value of throttle to the pitch motors
float desired_angle = 0; //This is the angle in which we whant the
                         //balance to stay steady
double throttle1=1300; //initial value of throttle to the roll motors
float desired_angle1 = 0; //This is the angle in which we whant the
                         //balance to stay steady


void setup() {
  Serial.begin(57600);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(18), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(19), doEncoderB, CHANGE);
  right_prop.attach(11); //attatch the right motor to pin 11
  left_prop.attach(10);  //attatch the left motor to pin 10
  
  pinMode(encoderPinC, INPUT_PULLUP);
  pinMode(encoderPinD, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(20), doEncoderC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(21), doEncoderD, CHANGE);
  //Serial.begin(57600);
  back_prop.attach(12); //attatch the right motor to pin 12
  front_prop.attach(13);  //attatch the left motor to pin 13

  time = millis(); //Start counting time in milliseconds
  /*In order to start up the ESCs we have to send a min value
   * of PWM to them before connecting the battery. Otherwise
   * the ESCs won't start up or enter in the configure mode.0
   * The min value is 1000us and max is 2000us, REMEMBER!*/
  
  front_prop.writeMicroseconds(1000); 
  back_prop.writeMicroseconds(1000);
  left_prop.writeMicroseconds(1000); 
  right_prop.writeMicroseconds(1000);
  delay(7000); /*Give some delay, 7s, to have time to connect
                *the propellers and let everything start up*/ 
}//end of setup void

void loop() {

    timePrev = time;  // the previous time is stored before the actual time read
    time = millis();  // actual time read
    elapsedTime = (time - timePrev) / 1000; 
  
  /*The tiemStep is the time that elapsed since the previous loop. 
   * This is the value that we will use in the formulas as "elapsedTime" 
   * in seconds. We work in ms so we haveto divide the value by 1000 
   to obtain seconds*/
   
    Pitch_Angle = (0.36 * (currentPosition/4));
   // Serial.print(" | Counter: ");
    //Serial.print(currentPosition);
   // Serial.print(" | Pitch Angle: ");
    //Serial.print(Pitch_Angle);
    
    Roll_Angle = (0.36 * (currentPosition1/4));
   // Serial.print(" | Counter: ");
    //Serial.print(currentPosition);
    Serial.print(" | Roll Angle: ");
    Serial.println(Roll_Angle);
  
/*///////////////////////////P I D///////////////////////////////////*/

/*First calculate the error between the desired angle and 
*the real measured angle*/
error = Pitch_Angle - desired_angle;
error1 = Roll_Angle - desired_angle1;
    
/*Next the proportional value of the PID is just a proportional constant
*multiplied by the error*/

pid_p = kp*error;
pid_p1 = kp1*error1;

/*The integral part should only act if we are close to the
desired position but we want to fine tune the error. That's
why I've made a if operation for an error between -2 and 2 degree.
To integrate we just sum the previous integral value with the
error multiplied by  the integral constant. This will integrate (increase)
the value each loop till we reach the 0 point*/
if(-3 <error <3)
{
  pid_i = pid_i+(ki*error);  
}
if(-3 <error1 <3)
{
  pid_i1 = pid_i1+(ki1*error1);  
}

/*The last part is the derivate. The derivate acts upon the speed of the error.
As we know the speed is the amount of error that produced in a certain amount of
time divided by that time. For taht we will use a variable called previous_error.
We substract that value from the actual error and divide all by the elapsed time. 
Finnaly we multiply the result by the derivate constant*/

pid_d = kd*((error - previous_error)/elapsedTime);
pid_d1 = kd1*((error1 - previous_error1)/elapsedTime);

/*The final PID values is the sum of each of this 3 parts*/
PID = pid_p + pid_i + pid_d;
PID1 = pid_p1 + pid_i1 + pid_d1;

/*We know taht the min value of PWM signal is 1000us and the max is 2000. So that
tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
have a value of 2000us the maximum value taht we could sybstract is 1000 and when
we have a value of 1000us for the PWM sihnal, the maximum value that we could add is 1000
to reach the maximum 2000us*/
if(PID < -1000)
{
  PID=-1000;
}
if(PID > 1000)
{
  PID=1000;
}
if(PID1 < -1000)
{
  PID1=-1000;
}
if(PID1 > 1000)
{
  PID1=1000;
}
/*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
pwmFront = throttle + PID1;
pwmBack = throttle - PID1;
pwmLeft = throttle1 - PID;
pwmRight = throttle1 + PID;


/*Once again we map the PWM values to be sure that we won't pass the min
and max values. Yes, we've already maped the PID values. But for example, for 
throttle value of 1300, if we sum the max PID value we would have 2300us and
that will mess up the ESC.*/
//Back
if(pwmBack <1000)
{
  pwmBack= 1000;
}
if(pwmBack > 2000)
{
  pwmBack=2000;
}
//Front
if(pwmFront < 1000)
{
  pwmFront= 1000;
}
if(pwmFront > 2000)
{
  pwmFront=2000;
}
//Right
if(pwmRight <1000)
{
  pwmRight= 1000;
}
if(pwmRight > 2000)
{
  pwmRight=2000;
}
//Left
if(pwmLeft < 1000)
{
  pwmLeft= 1000;
}
if(pwmLeft > 2000)
{
  pwmLeft=2000;
}

/*Finnaly using the servo function we create the PWM pulses with the calculated
width for each pulse*/
front_prop.writeMicroseconds(pwmFront);
back_prop.writeMicroseconds(pwmBack);
left_prop.writeMicroseconds(pwmLeft);
right_prop.writeMicroseconds(pwmRight);
previous_error = error; //Remember to store the previous error.
previous_error1 = error1; //Remember to store the previous error.
Serial.print(" | Left Motor PWM: ");
Serial.println(pwmLeft);
Serial.print(" | Right Motor PWM: ");
Serial.println(pwmRight);
Serial.print(" | Front Motor PWM: ");
Serial.println(pwmFront);
Serial.print(" | Back Motor PWM: ");
Serial.println(pwmBack);
}//end of loop void

void doEncoderA()
{
  if(digitalRead(encoderPinA) != digitalRead(encoderPinB))
  {
    currentPosition++;
  }
  else
  {
    currentPosition--;
  }
}
void doEncoderB()
{
  if(digitalRead(encoderPinA) == digitalRead(encoderPinB))
  {
    currentPosition++;
  }
  else
  {
    currentPosition--;
  }
}
void doEncoderC()
{
  if(digitalRead(encoderPinC) != digitalRead(encoderPinD))
  {
    currentPosition1++;
  }
  else
  {
    currentPosition1--;
  }
}
void doEncoderD()
{
  if(digitalRead(encoderPinC) == digitalRead(encoderPinD))
  {
    currentPosition1++;
  }
  else
  {
    currentPosition1--;
  }
}
