#include "main.h"

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))) // 限幅函数

MPU6050 mpu6050 = MPU6050(Wire);//实例化MPU6050
hw_timer_t *timer = NULL; // Timer for accurate pulse width measurement
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IMUTask(void *pvParameters);
void testdataprint(); //调试打印函数
void Open_thread_function();//启动线程


void setup()
{
  Wire.begin(1, 2, 400000UL);//初始化IIC
  mpu6050.begin(); //初始化MPU陀螺仪
  Open_thread_function();//启动线程
  Serial.begin(115200);//初始化调试串口
  CANInit(); // 初始化CAN
  ppm_init(); //遥控器读取中断初始化
  motorInit();
  delay(3000);
  CAN_Control();//使能关节电机
  delay(1000);
}

void loop()
{
  serialReceiveUserCommand();                                 // 串口数据输入处理 用于调试pid用
  // PIDValues pid = interpolatePID(ZeparamremoteValue);         //PID线性拟合函数
  wheel_control();                                            // 轮子 霍尔电机PID控制函数
  CAN_Control();                                         // CAN 关节电机控制函数
  remote_switch();                                            // 遥控开关
  jump_control();                                             // 机器人跳跃控制
  inverseKinematics();                                           // 运动学逆解
  robot_control();                                            // 机器人行为控制
  sendMotorTargets(up_start * wheel_motor1_target, up_start * wheel_motor2_target); // 发送控制轮毂电机的目标值
  storeFilteredPPMData();                                     // 对ppm数据进行滤波处理 避免数据大幅度跳动
  mapPPMToRobotControl();                                     // 处理遥控器数据 将其映射为机器人行为控制
  testdataprint();
}

//启动线程
void Open_thread_function() 
{
    // 陀螺仪读取任务进程
    xTaskCreatePinnedToCore(
        IMUTask,   // 任务函数
        "IMUTask", // 任务名称
        4096,      // 堆栈大小
        NULL,      // 传递的参数
        1,         // 任务优先级
        NULL,      // 任务句柄
        1          // 运行在核心 0
    );
}

// 陀螺仪数据读取
void IMUTask(void *pvParameters) 
{
  gyroY = mpu6050.getGyroY();  // 第一次读出来作为初值
  
  while (true) 
  {
    mpu6050.update();
    roll = mpu6050.getAngleX();
    pitch = mpu6050.getAngleY();
    //yaw = mpu6050.getAngleZ();
    gyroX = mpu6050.getGyroX();
    gyroZ = mpu6050.getGyroZ();
    // 这里是设置一定死区 避免数据的波动
    if (gyroZ > -3 && gyroZ < 6) gyroZ = 0;
    gyroY = lowPassFilter(mpu6050.getGyroY(), gyroY, 0.005);

  }
}