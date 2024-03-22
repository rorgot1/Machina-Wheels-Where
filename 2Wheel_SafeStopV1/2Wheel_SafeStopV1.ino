#include <Servo.h>
#include <VescUart.h>
#define RCPinY 2
#define RCPinX 3
#define CH5 20
//#define CH6 20
Servo mL, mR; 
VescUart UART_L, UART_R;

const uint8_t motorL = 6;
const uint8_t motorR = 7;

//PID variable
volatile double out_old_L = 0, err_old_L = 0, err_L, output_L, PWM_L ;
volatile double out_old_R = 0, err_old_R = 0, err_R, output_R, PWM_R ;
volatile double setpoint_L, feedback_L ;
volatile double setpoint_R, feedback_R ;
volatile double Kp_L, Ki_L, Kd_L ; //Kp_L = 0.5, Ki_L = 0.15, Kd_L = 0;
volatile double Kp_R, Ki_R, Kd_R ;

unsigned long t3 = millis(); //SamplingRate
int t_sampling = 75;

int diffL, iL=0, roundL, old_setpoint_L = 0, next_setpoint_L ; //Sudden speed change 
int diffR, iR, roundR, old_setpoint_R = 0, next_setpoint_R ; //Sudden speed change 

int buzzerPin = 24;
byte trigger = 1, getout = 1;
int L_val, R_val, stopstate ;

///////////////////
volatile long startTimeY = 0;
volatile long currentTimeY = 0;
volatile long pulseY = 0;
int Y_val = 0;

volatile long startTimeX = 0;
volatile long currentTimeX = 0;
volatile long pulseX = 0;
int X_val = 0;

volatile long startTimeCH5 = 0;
volatile long currentTimeCH5 = 0;
volatile long pulseCH5 = 0;
int CH5_val = 0;

volatile long startTimeCH6 = 0;
volatile long currentTimeCH6 = 0;
volatile long pulseCH6 = 0;
int CH6_val = 0;

int LDR=0, sum_a, count;

int RemoteControl_v[3] ; //สร้าง array เก็บค่าความเร็วล้อ [ซ้าย ขวา หยุด] 
int readReal[3];

int* RemoteControl(int x, int y, int z) //สร้างฟังก์ชันในการแปลงค่าที่อ่านได้จากรีโมทเพื่อไปหาความเร็วต่อ
{    
  int Map_X, Map_Y;
  
  //Filter unusable 
  if(y > 1980) y = 1980;
  if(y < 904) y = 904;
  if(x > 1980) x = 1980;
  if(x < 928) x = 928;  

  //Change Value to RPM
  Map_Y = map(y, 1980, 904, -95, 95);
  Map_X = map(x, 1980, 928, -95, 95);
  //Set zero area
  if(Map_Y < 15 && Map_Y > -15) Map_Y = 0;
  if(Map_X < 14 && Map_X > -14) Map_X = 0;

  //Calculate motor Left and Right RPM
  if(Map_X == 0)          //Forward-Backward
  {
    RemoteControl_v[0] = Map_Y;
    RemoteControl_v[1] = Map_Y;
  }
  else if(Map_X > 0 && Map_Y > 0)  //Left-Top
  {
    RemoteControl_v[0] = Map_Y - Map_X;
    RemoteControl_v[1] = Map_Y;    
  }
  else if(Map_X < 0 && Map_Y > 0)  //Right-Top
  {
    RemoteControl_v[0] = Map_Y;
    RemoteControl_v[1] = Map_Y + Map_X;    
  }  
  else if(Map_X > 0 && Map_Y < 0)  //Left-Bottom
  {
    RemoteControl_v[0] = Map_Y + Map_X;
    RemoteControl_v[1] = Map_Y;    
  }  
  else if(Map_X < 0 && Map_Y < 0)  //Right-Bottom
  {
    RemoteControl_v[0] = Map_Y;
    RemoteControl_v[1] = Map_Y - Map_X;    
  } 

  //Zero speed set
  if(z < 1800) RemoteControl_v[2] = 1;
  else RemoteControl_v[2] = 0; 

  return RemoteControl_v;
}

void pulsetimeY()
{
  currentTimeY = micros() ;
  if(currentTimeY > startTimeY)
  {
    pulseY = currentTimeY - startTimeY ;
    startTimeY = currentTimeY ;
  }
}

void pulsetimeX()
{
  currentTimeX = micros() ;
  if(currentTimeX > startTimeX)
  {
    pulseX = currentTimeX - startTimeX ;
    startTimeX = currentTimeX ;
  }
}

/*void pulsetimeCH6()
{
  currentTimeCH6 = micros() ;
  if(currentTimeCH6 > startTimeCH6)
  {
    pulseCH6 = currentTimeCH6 - startTimeCH6 ;
    startTimeCH6 = currentTimeCH6 ;
  }  
}*/

void pulsetimeCH5()
{
  currentTimeCH5 = micros() ;
  if(currentTimeCH5 > startTimeCH5)
  {
    pulseCH5 = currentTimeCH5 - startTimeCH5 ;
    startTimeCH5 = currentTimeCH5 ;
  }  
}

int* joy()
{
  if (pulseY < 2000) Y_val = pulseY ;
  if (pulseX < 2000) X_val = pulseX ;
  if (pulseCH5 < 2000) CH5_val = pulseCH5 ;
  //if (pulseCH6 < 2000) CH6_val = pulseCH6 ;
  readReal[0] = X_val;
  readReal[1] = Y_val;
  readReal[2] = CH5_val;
  
  return  readReal;
}

int Incre_PI_L(float setpoint, float feedback, float Kp, float Ki, float Max, float Min, byte trig)
{
  if (trig == 1)
  {
    setpoint = 0;
    feedback = 0;
    out_old_L = 0;
  }
  err_L = setpoint - feedback ;
  output_L = out_old_L + Kp * (err_L - err_old_L) + Ki * err_L ;
  err_old_L = err_L ;
  out_old_L = output_L ;

  if (output_L > Max) output_L = Max;
  if (output_L < Min) output_L = Min;

  return output_L ;
}

int Incre_PI_R(float setpoint, float feedback, float Kp, float Ki, float Max, float Min, byte trig)
{
  if (trig == 1)
  {
    setpoint = 0;
    feedback = 0;
    out_old_R = 0;
  }
  err_R = setpoint - feedback ;
  output_R = out_old_R + Kp * (err_R - err_old_R) + Ki * err_R ;
  err_old_R = err_R ;
  out_old_R = output_R ;

  if (output_R > Max) output_R = Max;
  if (output_R < Min) output_R = Min;

  return output_R ;
}

void emergencysound_light(int buzzer_pin)
{
  tone(buzzer_pin, 200);
  digitalWrite(23, HIGH);
  delay(500);
  tone(buzzer_pin, 1000);
  digitalWrite(23, LOW);
  delay(500);
  tone(buzzer_pin, 200);
  digitalWrite(23, HIGH);
  delay(500);
  tone(buzzer_pin, 1000);
  digitalWrite(23, LOW);
  delay(500);
  noTone(buzzer_pin);
}

void setup() 
{
  mL.attach(6);
  mR.attach(7);
  mL.writeMicroseconds(1500);
  mR.writeMicroseconds(1500);

  Serial.begin(9600);
  Serial1.begin(115200);
  Serial2.begin(115200);

  UART_L.setSerialPort(&Serial1);
  UART_R.setSerialPort(&Serial2);

  pinMode(22, OUTPUT) ; //Green
  pinMode(23, OUTPUT) ; //Red
  pinMode(RCPinY, INPUT_PULLUP);
  pinMode(RCPinX, INPUT_PULLUP);
  pinMode(CH5, INPUT_PULLUP);
  //pinMode(CH6, INPUT_PULLUP);  
  pinMode(45, INPUT) ;  //LDR sensor for detecting Reciever
  pinMode(44, INPUT) ;  //Emergency Stop
  attachInterrupt(digitalPinToInterrupt(RCPinY), pulsetimeY, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RCPinX), pulsetimeX, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH5), pulsetimeCH5, CHANGE);
  
  Serial.println("Starting System...");
  delay(1000);
}

void loop() 
{
  sum_a = 0; count = 0; t3 = millis();
  while(LDR == 0)     //Detecting Reciever
  {
    Serial.println("Remote Checking...");
    if(millis() - t3 <= 1200)
    {
      sum_a += !digitalRead(45);
      count ++ ;
    }
    else if(sum_a == count)
    {
      Serial.print(sum_a); Serial.print("\t"); Serial.println(count);
      delay(500);
      LDR = 1 ;      
    }
    else
    {
      Serial.print(sum_a); Serial.print("\t"); Serial.println(count);
      delay(500);
      break;      
    }
  }

  while(LDR == 1 && UART_L.getVescValues() && UART_R.getVescValues() && digitalRead(44) == 0) //PID Control with 4 prepared conditions(LDR[Reciever ON] && UART_L ON && UART_R OFF && Emergency button OFF)  
  {
    if(digitalRead(45) == 1) LDR = 0;
    
    digitalWrite(23, LOW);
    if (millis() - t3 >= 5)
    {
      feedback_L = UART_L.data.rpm / 15 ;
      feedback_R = UART_R.data.rpm / 15 ;
      
      stopstate = RemoteControl(joy()[0],joy()[1],joy()[2])[2];
      
      setpoint_L = RemoteControl(joy()[0],joy()[1],joy()[2])[0];  //map(RemoteControl(4,5)[1], 0, 1023, -150, 150);
      setpoint_R = RemoteControl(joy()[0],joy()[1],joy()[2])[1];  //map(RemoteControl(4,5)[0], 0, 1023, -150, 150);

      Kp_L = 0.5 ;
      Ki_L = 0.02 ;
      Kp_R = 0.5 ;
      Ki_R = 0.02 ;

      PWM_L = Incre_PI_L(setpoint_L, feedback_L, Kp_L, Ki_L, 500, -500, stopstate);
      PWM_R = Incre_PI_R(setpoint_R, feedback_R, Kp_R, Ki_R, 500, -500, stopstate);

      mL.writeMicroseconds(1500 + PWM_L);
      mR.writeMicroseconds(1500 + PWM_R);
      Serial.print(setpoint_L); Serial.print("\t"); Serial.print(feedback_L); Serial.print("\t"); Serial.println(stopstate);

      t3 = millis();

    }
      roundL = PWM_L/10 ;
      roundR = PWM_R/10 ;

  }

  while(PWM_L != 0 || PWM_R != 0) //Speed decreasing smoothly
  {
    diffL = Incre_PI_L(0, 0, Kp_L, Ki_L, 500, -500, 1);
    diffR = Incre_PI_R(0, 0, Kp_R, Ki_R, 500, -500, 1);

    digitalWrite(22, HIGH);
    if (millis() - t3 >= 100)
    {
      Serial.println(PWM_L);
      PWM_L = PWM_L - roundL;
      PWM_R = PWM_R - roundR;
      if(PWM_L <= 0) PWM_L = 0;
      if(PWM_R <= 0) PWM_R = 0;
      mL.writeMicroseconds(1500 + PWM_L);
      mR.writeMicroseconds(1500 + PWM_R);  
      t3 = millis();    
    }
    
  }

  PWM_L = 0; PWM_R = 0;
  digitalWrite(22, LOW);
  emergencysound_light(buzzerPin);
  mL.writeMicroseconds(1500);
  mR.writeMicroseconds(1500);
  delay(10);
  Serial.println("Immovability");

}
