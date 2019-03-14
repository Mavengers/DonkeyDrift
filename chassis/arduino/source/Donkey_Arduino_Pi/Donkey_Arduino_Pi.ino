/*
[File Name]   Donkey_Arduino_v8.ino
[Description] Support DonkeyCar on Raspberry Pi via PCA9685 servo controller.

[Change Notes]
Donkey_Arduino_v5 >> Replaced PPMrcIn by pulseIn(), added RBD_Timer.h for timeline control
Donkey_Arduino_v6 >> Restore Servo.h for servo control
Donkey_Arduino_v7 >> Add Fail-safe for wire loosen or wrong data from PCA9685
Donkey_Arduino_v8 >> Modify Turn-off voltage to 11.3V for battery protection
*/

#include <SSD1306.h>
#include <Servo.h>
#include <RBD_Timer.h> // https://github.com/alextaujenis/RBD_Timer

////////OLED显示屏引脚////
#define OLED_DC     10
#define OLED_CLK    19
#define OLED_MOSI   13
#define OLED_RESET  12
/////////TB6612驱动引脚////
#define AIN1        11
#define AIN2         5
#define BIN1         6
#define BIN2         3
#define SERVO        9
/////////编码器引脚////////
#define ENCODER_L    8  
#define DIRECTION_L  4
#define ENCODER_R    7
#define DIRECTION_R  2
/////////按键引脚/////////
#define KEY 18
#define T 0.156f
#define L 0.1445f
#define pi 3.1415926f

Servo myservo;  //创建一个舵机控制对象
RBD::Timer timer_Channel_0;
RBD::Timer timer_Channel_1;
RBD::Timer timer_OLED;
SSD1306 oled(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, 0);

/****************************自定义变量***********************/
unsigned char Flag_Stop = 1, Flag_Way = 1; //停止和配置标志位
int Velocity, Angle;   //车轮速度与转角
int Battery_Voltage; //电池电压采样变量
int Votltage_Threshold = 1110;
int servo;
int Motor_A, Motor_B;
int SERVO_MID = 88;
int SERVO_AMP = 50;
float Servo_K_L = 0.5f;
float Servo_K_R = 1.2f;
float Angle_Real = 0.0f;
float Angle_K = 0.72f; // 转向角比例系数
int PWM_Channel_0_Raw = 0;
int PWM_Channel_1_Raw = 0;
int PWM_Channel_0 = 0;
int PWM_Channel_1 = 0;
int Channel_Min = 1000;
int Channel_Max = 2000;
float Speed_K = 9.0f;
int Yuzhi = 5;


// volatile long Velocity_L, Velocity_R ;   //左右轮编码器数据
// int Velocity_Left = 0, Velocity_Right = 0; 
// float Velocity_KP = 0.3, Velocity_KI =  0.3;  // PI闭环控制系数

/**************************************************************************
函数功能：按键扫描
入口参数：无
返回  值：按键状态 0：无动作 1：单击
**************************************************************************/
unsigned char My_click (void)
{
  byte flag_key = 1; //按键按松开标志
  if (flag_key && (digitalRead(KEY) == 1)) { //如果发生单击事件
    if (digitalRead(KEY) == 1) {
      flag_key = 1;
    }
  }
  else if (digitalRead(KEY) == 0) {
    flag_key = 0;
  }
  return flag_key;    //M键
}
/**************************************************************************
函数功能：求次方的函数
入口参数：m,n
返回  值：m的n次幂
**************************************************************************/
uint32_t oled_pow(uint8_t m, uint8_t n)
{
  uint32_t result = 1;
  while (n--)result *= m;
  return result;
}
/**************************************************************************
函数功能：显示变量
入口参数：x:x坐标   y:行     num：显示的变量   len ：变量的长度
**************************************************************************/
void OLED_ShowNumber(uint8_t x, uint8_t y, uint32_t num, uint8_t len)
{
  uint8_t t, Stop_State;
  uint8_t enshow = 0;
  for (t = 0; t < len; t++)  {
    Stop_State = (num / oled_pow(10, len - t - 1)) % 10;
    oled.drawchar(x + 6 * t, y, Stop_State + '0');
  }
}

/********函数功能：OLED显示*********/
void OLED()
{
  char  i, t;
  static char offset_1 = 0, offset_2 = 1;

  oled.drawstring(00, 00, "DRIVING");  //编码器数据
  if (Flag_Way == 0)      oled.drawstring( 55, 00, "PS2");
  else if (Flag_Way == 1) oled.drawstring( 55, 00, "PI ");
  if (Flag_Stop == 0)     oled.drawstring(100, 00, "ON ");
  else                    oled.drawstring(100, 00, "OFF");


  oled.drawstring(00, 01 + offset_1, "HEADING");  //编码器数据
  if ( Angle_Real < 0)
  {
    oled.drawstring( 80, 01 + offset_1, "-");
    OLED_ShowNumber(100, 01 + offset_1, -Angle_Real, 3);
  }
  else
  {
    oled.drawstring( 80, 01 + offset_1, "+");
    OLED_ShowNumber(100, 01 + offset_1, Angle_Real, 3);
  }

  oled.drawstring(00, 02 + offset_1, "PWM-LEFT");  //PWM数据
  if ( Motor_A < 0)
  {
    oled.drawstring( 80, 02 + offset_1, "-");
    OLED_ShowNumber(100, 02 + offset_1, -Motor_A, 3);
  }
  else
  {
    oled.drawstring( 80, 02 + offset_1, "+");
    OLED_ShowNumber(100, 02 + offset_1, Motor_A, 3);
  }

  oled.drawstring(00, 03 + offset_1, "PWM-RIGHT");  //PWM数据
  if ( Motor_B < 0)
  {
    oled.drawstring( 80, 03 + offset_1, "-");
    OLED_ShowNumber(100, 03 + offset_1, -Motor_B, 3);
  }
  else
  {
    oled.drawstring( 80, 03 + offset_1, "+");
    OLED_ShowNumber(100, 03 + offset_1, Motor_B, 3);
  }

  oled.drawstring(00, 04 + offset_1, "VOLTAGE:");
  oled.drawstring(71, 04 + offset_1, ".");
  oled.drawstring(93, 04 + offset_1, "V");
  OLED_ShowNumber(58, 04 + offset_1, Battery_Voltage / 100, 2);
  OLED_ShowNumber(81, 04 + offset_1, Battery_Voltage % 100, 2);

  oled.drawstring(00, 06, "CH0");
  OLED_ShowNumber(25, 06, PWM_Channel_0_Raw, 4);
  oled.drawstring(00, 07, "VAL0");
  OLED_ShowNumber(30, 07, PWM_Channel_0, 3);

  oled.drawstring(60, 06, "CH1");
  OLED_ShowNumber(85, 06, PWM_Channel_1_Raw, 4);
  oled.drawstring(60, 07, "VAL1");
  OLED_ShowNumber(90, 07, PWM_Channel_1, 3);

  oled.display();
}


/***************函数功能：遥控**********/
void Get_RC(void)
{
  float LY, RX;
  if (Flag_Stop == 0)
  {
    LY = 128 - PWM_Channel_0; //计算偏差
    RX = 128 - PWM_Channel_1;
    if (LY > -Yuzhi && LY < Yuzhi)LY = 0; //小角度设为死区 防止抖动出现异常
    if (RX > -Yuzhi && RX < Yuzhi)RX = 0;
    Velocity = -LY / 4; //速度和摇杆的力度相关。
    Angle = RX / 2;
  }
}
/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM
**************************************************************************/
void Set_Pwm(int Motor_A, int Motor_B)
{
  if (Motor_A >= 0)     analogWrite(AIN2, Motor_A), digitalWrite(AIN1, LOW); //赋值给PWM寄存器
  else                 digitalWrite(AIN1, HIGH), analogWrite(AIN2, 255 + Motor_A); //赋值给PWM寄存器
  if (Motor_B >= 0)     digitalWrite(BIN2, LOW), analogWrite(BIN1, Motor_B); //赋值给PWM寄存器
  else                 analogWrite(BIN1, 255 + Motor_B), digitalWrite(BIN2, HIGH); //赋值给PWM寄存器
}
/**************************************************************************
函数功能：异常关闭电机
入口参数：电压
返回  值：1：异常  0：正常
/**************************************************************************/
unsigned char  Turn_Off()
{
  static byte Turn_Off_State = 0;
  if (Flag_Stop == 1 || Battery_Voltage < Votltage_Threshold) { //Flag_Stop置1或者电压太低，则关闭电机
    Turn_Off_State = 1;
  }
  else   Turn_Off_State = 0;
  return Turn_Off_State;
  return Turn_Off_State;
}


/**************************************************************************
函数功能：小车运动数学模型
入口参数：速度和转角
//**************************************************************************/
void Kinematic_Analysis(float velocity, float angle)
{
  if (angle <= 0) servo = SERVO_MID + angle * Servo_K_L;         //舵机转向
  else            servo = SERVO_MID + angle * Servo_K_R;

  servo = constrain(servo, SERVO_MID - SERVO_AMP, SERVO_MID + SERVO_AMP + 40);
  Angle_Real = angle * Angle_K;

  Motor_A = velocity * (1 + T * tan(Angle_Real * pi / 180) / 2 / L) * Speed_K;
  Motor_B = velocity * (1 - T * tan(Angle_Real * pi / 180) / 2 / L) * Speed_K; //后轮差速

  Motor_A = constrain(Motor_A, -255, 255);
  Motor_B = constrain(Motor_B, -255, 255);
}

/*********函数功能：5ms控制函数 核心代码  *******/
void control()
{
  static int Stop_State, Voltage_Temp; //临时变量
  static float Voltage_All; //电压采样相关变量
  static unsigned char Voltage_Count; //位置控制分频用的变量
  Get_RC();
  Kinematic_Analysis(Velocity, Angle);
  Voltage_Temp = analogRead(0);  //采集一下电池电压
  Voltage_Count++;       //平均值计数器
  Voltage_All += Voltage_Temp;   //多次采样累积
  if (Voltage_Count == 20) Battery_Voltage = Voltage_All * 0.5371 / 2, Voltage_All = 0, Voltage_Count = 0; //求平均值
  Stop_State = My_click();   //按键检查
  if (Stop_State == 0)Flag_Stop = !Flag_Stop;
  if (Turn_Off() == 0) {
    Set_Pwm(Motor_A, Motor_B); //不存在异常，使能电机转速
    myservo.write(servo);    //不存在异常，使能舵机转角
  }
  else {
    Set_Pwm(0, 0);            //存在异常，使能电机关闭
    myservo.write(SERVO_MID); //存在异常，使能舵机归正
  }

}
/***********函数功能：初始化  ************/
void setup()
{
  oled.ssd1306_init(SSD1306_SWITCHCAPVCC);
  oled.clear();   // clears the screen and buffer
  pinMode(AIN1, OUTPUT);          //电机控制引脚
  pinMode(AIN2, OUTPUT);          //电机控制引脚，
  pinMode(BIN1, OUTPUT);          //电机速度控制引脚
  pinMode(BIN2, OUTPUT);          //电机速度控制引脚
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(KEY, INPUT_PULLUP);              //按键引脚
  timer_Channel_0.setTimeout(5);
  timer_Channel_1.setTimeout(5);
  timer_OLED.setTimeout(40);

  myservo.attach(SERVO);           // 选择控制的舵机
  delay(10);                      //延时等待初始化完成
  myservo.write(SERVO_MID);
}
/******函数功能：主循环程序体*******/
void loop()
{
  if (timer_Channel_0.onRestart()) {
    PWM_Channel_0_Raw = pulseIn(A1, HIGH);
    if (PWM_Channel_0_Raw >= 500 && PWM_Channel_0_Raw <= 2500 ) {
      PWM_Channel_0 = constrain(PWM_Channel_0_Raw, Channel_Min, Channel_Max);
      PWM_Channel_0 = map(PWM_Channel_0, Channel_Min, Channel_Max, 0, 255);
      // Serial.print(PWM_Channel_0_Raw);
      // Serial.print('\t');
    }
    else if (PWM_Channel_0_Raw == 0) {
      PWM_Channel_0 = 128;// void for donkey calibrate --channel 0 on Raspberry Pi
    }
    else {
      Flag_Stop = 1;
    }
  }
  if (timer_Channel_1.onRestart()) {
    PWM_Channel_1_Raw = pulseIn(A2, HIGH);
    if (PWM_Channel_1_Raw >= 500 && PWM_Channel_1_Raw <= 2500) {
      PWM_Channel_1 = constrain(PWM_Channel_1_Raw, Channel_Min, Channel_Max);
      PWM_Channel_1 = map(PWM_Channel_1, Channel_Min, Channel_Max, 0, 255);
      // Serial.println(PWM_Channel_1_Raw);
    }
    else if (PWM_Channel_1_Raw == 0) {
      PWM_Channel_1 = 128;// void for donkey calibrate --channel 1 on Raspberry Pi
    }
    else {
      Flag_Stop = 1;
    }
  }

  control();
  if (timer_OLED.onRestart()) {
    OLED();
  }
}
