#include "ppm.h"

uint16_t ppmValues[NUM_CHANNELS] = {0} ;
int filteredPPMValues[NUM_CHANNELS] = {0};
uint32_t lastTime = 0;
uint8_t currentChannel = 0;
float alpha = 0.02;       // 设置滤波系数（值越小，平滑度越高）ime = 0;
int EH_rollflag;
int jump_flag = 0;
int up_start = 0;  //一键启动
int Shake_shoulder = 0 ;//抖肩

// 遥控器开关拨动 判断是否开启机器人跳动和开启roll自稳
void remote_switch()
{
  if( ppmValues[4] < 1300)  //遥控器左上波轮滑到最下面（L），机器人跳跃
  {
    Shake_shoulder = 0 ;//不抖肩
    if(ppmValues[6] > 1300 && EH_rollflag ==0)//没有开启自稳
    {
      jump_flag = 1;//机器人起跳
    }
    else{
      jump_flag = 0; //机器人不起跳
    }
  }
  else //遥控器左上波轮滑到最上面（H）,机器人抖肩
  {  
    jump_flag = 0;//机器人不起跳
    if(ppmValues[6] > 1300 && EH_rollflag ==0)//没有开启自稳
    {
      Shake_shoulder = 1 ;//机器人控制遥杆抖肩
    }
    else{
      Shake_shoulder = 0 ;//不抖肩
    }
  }

  if (ppmValues[7] > 1800)//机器人轮毂电机启动和机器人开启自稳定
  {
    EH_rollflag = 1;
    up_start = 1;
  }
  else if (ppmValues[7] > 1300)//机器人轮毂电机启动和机器人关闭自稳定
  {
    EH_rollflag = 0;
    up_start = 1;
  }
  else if (ppmValues[7] < 1200)//机器人关闭轮毂电机和机器人关闭自稳定
  {
    EH_rollflag = 0;
    up_start = 0;
  }
}

//遥控器PPM读取初始化
void ppm_init()
{
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), onPPMInterrupt, RISING); // 开启ppm读取中断
}

//遥控器低通滤波
float lowPassFilter(float currentValue, float previousValue, float alpha) 
{
  return alpha * currentValue + (1 - alpha) * previousValue;
}

//遥控器低通滤波接口
void storeFilteredPPMData()
{
  filteredPPMValues[0] = lowPassFilter(ppmValues[2], filteredPPMValues[0], alpha);//腿高
  filteredPPMValues[1] = lowPassFilter(ppmValues[1], filteredPPMValues[1], alpha);//前进后退
  filteredPPMValues[2] = lowPassFilter(ppmValues[5], filteredPPMValues[2], alpha);//平衡偏置
  // filteredPPMValues[3] = lowPassFilter(ppmValues[3], filteredPPMValues[3], alpha);
  filteredPPMValues[4] = lowPassFilter(ppmValues[3], filteredPPMValues[4], alpha);//左遥杆左右腿高
  filteredPPMValues[5] = lowPassFilter(ppmValues[0], filteredPPMValues[5], alpha);//左右转向
}

void IRAM_ATTR onPPMInterrupt()
{
  uint32_t now = micros();
  uint32_t duration = now - lastTime;
  lastTime = now;

  if (duration >= SYNC_GAP)
  {
    // Sync pulse detected, reset channel index
    currentChannel = 0;
  }
  else if (currentChannel < NUM_CHANNELS)
  {
    ppmValues[currentChannel] = duration;
    currentChannel++;
  }
}