#include "robot.h"

IKparam IKParam;
// 左腿相关运动学基本参数
motionControlParams LeftMotionControlParams;
// 右腿相关运动学基本参数
motionControlParams RightMotionControlParams;
motorsparam motorsParam;

float roll_kp = 0.0020, roll_kd = -0.006;  //机器人自稳Kp、Kd值 - 增强高机身稳定性

float leftY = 140, rightY = 180;  //机器人Y轴方向腿部高度
float leftX = 60, rightX = 60;    //机器人X轴方向腿部高度
float x1 = leftX, x2 = rightX, Y1 = leftY, y2 = rightY;
float motorLeftFront, motorLeftRear, motorRightFront, motorRightRear;
int ZeparamremoteValue = 0;
float roll_EH;
float forwardBackward = 0;
float motor1_vel, motor2_vel;
int lastZeparamremoteValue = 0; // 上次输出的 ZeparamremoteValue
int EH = 0;
float Tartget_Roll_angle;
float roll, pitch, yaw, init_pitch; //陀螺仪xyz轴值
float gyroX, gyroY, gyroZ;//陀螺仪xyz轴加速度值
float target_roll = 0.0;
float steering = 0;
float remoteBalanceOffset = 0;
uint8_t origin_pos_flag = 1;
MIT devicesState[8];
float robot_kp = 3.0;
float jump_vlaue; //起跳高度
float balance_offset = 4;


// 用于获取电机初始位置函数
void get_origin_pos()
{
  if (origin_pos_flag == 1)
  {
    // 左腿关节电机初始位置
    motorsParam.motorleftsita0_origin = -devicesState[0].pos;
    motorsParam.motorleftsita1_origin = -devicesState[1].pos;

    // 右腿关节电机初始位置
    motorsParam.motorrightsita0_origin = -devicesState[3].pos;
    motorsParam.motorrightsita1_origin = -devicesState[2].pos;

    if (motorsParam.motorleftsita0_origin != 0 && motorsParam.motorleftsita1_origin != 0 && motorsParam.motorrightsita0_origin != 0 && motorsParam.motorrightsita1_origin != 0)
      origin_pos_flag = 0;
  }
}

// 将读取的PPM数据进行映射处理 转换为机器人控制参数
void mapPPMToRobotControl()
{
  if(Shake_shoulder == 0)
  {
    // 这里用于控制腿部高度变化控制 采用增量式PID是为了让腿部快速变化时，保持稳定
    float error = (1000 - (filteredPPMValues[0] - 70))/1.5; // 计算误差
    ZeparamremoteValue = -0.30 * error;      // 计算当前的值
    ZeparamremoteValue = constrainValue(ZeparamremoteValue, 0, 150);
    ZeparamremoteValue = ZeparamremoteValue + (ZeparamremoteValue - lastZeparamremoteValue) * 0.12; // 基于误差更新输出（增量控制）
    lastZeparamremoteValue = ZeparamremoteValue;
  }                                                    // 存储当前值作为下一次计算的参考
  // 用于roll轴自稳
  if(EH_rollflag == 1) //自稳开关
  {
    target_roll = target_roll +  roll_kp * (roll - 0);
    target_roll = constrainValue(target_roll, -60, 60);
  }
  else
  {
    target_roll = 0;
  }
  // 用于控制前进后退
  forwardBackward = mapJoystickValuevel(filteredPPMValues[1]);
  // 用于调节重心偏置
  remoteBalanceOffset = mapJoystickValueInt(filteredPPMValues[2]);
  
  
  // 用于控制转向
  steering = -0.03 * (-mapJoystickValuesteering(filteredPPMValues[5]) - gyroZ);
  steering = constrainValue(steering, -10, 10);
  //
  roll_EH = sin((target_roll * 3.14) / 180) * (LeftMotionControlParams.robotl) / 2;
  roll_EH = constrainValue(roll_EH, -80, 80);
}

void jump_control()
{
  static unsigned long startMillis = 0; // 用于记录起始时间
  static bool timingStarted = false;    // 标志是否开始计时
  if (jump_flag == 1)   //起跳flag
  {
    if (!timingStarted)  
    {
      // 开始计时
      startMillis = millis();
      timingStarted = true;
    }
    if (millis() - startMillis >= 70)
    { // 判断是否达到100ms
       Am_kp = 1.8;
       jump_vlaue = 0;
    }
    else
    {
       Am_kp = 2.5;
       jump_vlaue = 150;
    }
  }
  else
  {
    // 重置计时器和标志位
    Am_kp = 1.2;
    timingStarted = false;
    startMillis = 0;
  }
}

// 运动学逆解函数
void inverseKinematics()
{
  float alpha1, alpha2, beta1, beta2;

  // 右腿逆解运算
  float aRight = 2 * x2 * L1;
  float bRight = 2 * y2 * L1;
  float cRight = x2 * x2 + y2 * y2 + L1 * L1 - L2 * L2;
  float dRight = 2 * L4 * (x2 - L5);
  float eRight = 2 * L4 * y2;
  float fRight = ((x2 - L5) * (x2 - L5) + L4 * L4 + y2 * y2 - L3 * L3);

  IKParam.alphaRight = 2 * atan((bRight + sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
  IKParam.betaRight = 2 * atan((eRight - sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));

  alpha1 = 2 * atan((bRight + sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
  alpha2 = 2 * atan((bRight - sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
  beta1 = 2 * atan((eRight + sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));
  beta2 = 2 * atan((eRight - sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));

  alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * PI);
  alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * PI);

  if (alpha1 >= PI / 4)
    IKParam.alphaRight = alpha1;
  else
    IKParam.alphaRight = alpha2;
  if (beta1 >= 0 && beta1 <= PI / 4)
    IKParam.betaRight = beta1;
  else
    IKParam.betaRight = beta2;

  // 左腿逆解运算
  float aLeft = 2 * x1 * L1;
  float bLeft = 2 * Y1 * L1;
  float cLeft = x1 * x1 + Y1 * Y1 + L1 * L1 - L2 * L2;

  float dLeft = 2 * L4 * (x1 - L5);
  float eLeft = 2 * L4 * Y1;
  float fLeft = ((x1 - L5) * (x1 - L5) + L4 * L4 + Y1 * Y1 - L3 * L3);

  // alpha的计算
  alpha1 = 2 * atan((bLeft + sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
  alpha2 = 2 * atan((bLeft - sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
  beta1 = 2 * atan((eLeft + sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));
  beta2 = 2 * atan((eLeft - sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));

  // 角度解算范围限制
  alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * PI);
  alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * PI);

  if (alpha1 >= PI / 4)
    IKParam.alphaLeft = alpha1;
  else
    IKParam.alphaLeft = alpha2;
  if (beta1 >= 0 && beta1 <= PI / 4)
    IKParam.betaLeft = beta1;
  else
    IKParam.betaLeft = beta2;

  // 定义坐标原点alphaRight
  // 加1.57*8betaRight 
  motorRightRear = (2.33 + 1.57 * 8) - (IKParam.betaRight * 8);
  motorRightFront = (0.65 + 1.57 * 8) - (IKParam.alphaRight * 8);  // alphaLeft
  
  motorLeftRear = (1.55 + 1.57 * 8) - (IKParam.betaLeft * 8);
  motorLeftFront = (1.45 + 1.57 * 8) - (IKParam.alphaLeft * 8);
}


// 轮足机器人控制部分程序
// int aaa = 0;
float Shake_shoulder_vakue = 0;
void robot_control()
{
  if(Shake_shoulder == 1)//抖肩
  {
    ZeparamremoteValue = 50;
    // vel_kp = -0.55;//速度环
    // balance_kp = -0.17;//直立环Kp
    // balance_kd = 0.024;//直立环Kd
    // robot_kp = 3.5;
    // speed_limit = 4;
    Shake_shoulder_vakue = filteredPPMValues[4];
    if(Shake_shoulder_vakue< 1600 && Shake_shoulder_vakue > 1400)Shake_shoulder_vakue = 1500; 
    Shake_shoulder_vakue = (Shake_shoulder_vakue - 1500) / 20;
    Y1 = leftY + ZeparamremoteValue + EH_rollflag * roll_EH + jump_vlaue - Shake_shoulder_vakue;
    y2 = rightY + ZeparamremoteValue - EH_rollflag * roll_EH + jump_vlaue + Shake_shoulder_vakue;
  }
  else
  {
    Y1 = leftY + ZeparamremoteValue + EH_rollflag * roll_EH + jump_vlaue;
    y2 = rightY + ZeparamremoteValue - EH_rollflag * roll_EH + jump_vlaue;
  }
  x1 = leftX + -robot_kp * (-forwardBackward - (motor1_vel + (motor2_vel)) / 2) + -0.02 * gyroY ; //-0.02 * gyroY
  x2 = rightX + robot_kp * (-forwardBackward - (motor1_vel + (motor2_vel)) / 2) + -0.02 * gyroY ; //-0.02 * gyroY
  // 限幅Y轴幅度 避免超限导致逆解出问题
  Y1 = constrainValue(Y1, 130, 380);
  y2 = constrainValue(y2, 130, 380);
  // if (aaa++ > 300)
  // {
  //   aaa = 0;
  //   Serial.print(Y1);
  //   Serial.print("\t");
  //   Serial.print(y2);
  //   Serial.print("\t");
  //   Serial.println(ZeparamremoteValue);
  // }
}

float constrainValue(float value, float minValue, float maxValue)
{
  if (value > maxValue)
    return maxValue;
  if (value < minValue)
    return minValue;
  return value;
}

int mapJoystickValuerollzeparam(int inputValue)
{
  if (inputValue < 1000)
    inputValue = 1000;
  if (inputValue > 2000)
    inputValue = 2000;

  int output = ((inputValue - 1500) / 20);

  return output;
}

float mapJoystickValuevel(int inputValue)
{
  if (inputValue < 1000)
    inputValue = 1000;
  if (inputValue > 2000)
    inputValue = 2000;
  if (inputValue < 1600 && inputValue > 1400)
    inputValue = 1500;
  float mappedValue = (inputValue - 1500) / 100.0;
  return mappedValue;
}

// 摇杆轴数据的映射处理
float mapJoystickValueInt(int inputValue)
{
  if (inputValue < 1000)
    inputValue = 1000;
  if (inputValue > 2000)
    inputValue = 2000;
  float mappedValue = (inputValue - 1500) / 100.0;
  if (mappedValue > -0.7 && mappedValue < 0.7)
  {
    mappedValue = 0;
  }
  return mappedValue;
}
float mapJoystickValuesteering(int inputValue)
{
  if (inputValue < 1000)
    inputValue = 1000;
  if (inputValue > 2000)
    inputValue = 2000;
  if (inputValue < 1600 && inputValue > 1400)
    inputValue = 1500;
  float mappedValue = (inputValue - 1500) / 2.0;
  return mappedValue;
}
// 步态变化函数 用于前后移动模仿人滑动前行 通过遥控器控制
int trot(int inputValue)
{
  if (inputValue < 1000)
    inputValue = 1000;
  if (inputValue > 2000)
    inputValue = 2000;
  if (inputValue > 1400 && inputValue < 1550)
    inputValue = 1500;
  int output = ((inputValue - 1500) / 10);
  return output;
}