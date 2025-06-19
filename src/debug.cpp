#include "debug.h"

#define DEBUG_TIME false // 打印时间间隔

#define DEBUG_DURATION 100 // 輸出頻率，建議為10，
#define DEBUG_REMOTOR false // 打印遥控器数据
#define DEBUG_CONTROL false // 打印控制数据
#define DEBUG_MPU false // 打印陀螺儀的調試信息
#define DEBUG_PID true // 打印电机数据

int cnt;
unsigned long __lastTime = 0;
unsigned long loopCount = 0;

// 测试数据打印函数 用于打印调试相关数据
void testdataprint()
{
  #if DEBUG_TIME == true
    loopCount++;
    unsigned long currentTime = micros();
    Serial.println(currentTime - __lastTime);
    __lastTime = currentTime;
  #endif

  if (cnt++ > DEBUG_DURATION)
  {
    cnt = 0;
    
    #if DEBUG_MPU == true
    //   Serial.print(gyroZ);
    //   Serial.print("\t");
    //   Serial.print(pitch);
    //   Serial.print("\t");
    //   Serial.print(gyroY);
    //   Serial.print("\t");
    //   Serial.println(ZeparamremoteValue);
    Serial.print(pitch);
    Serial.print(",");
    Serial.print(yaw);
    Serial.print(",");
    Serial.print(roll);
    Serial.print(",");
    Serial.print(gyroX);
    Serial.print(",");
    Serial.print(gyroY);
    Serial.print(",");
    Serial.println(gyroZ);
    #endif

    #if DEBUG_REMOTOR == true
      for (int i = 0; i < NUM_CHANNELS; i++)
      {
        Serial.print(filteredPPMValues[i]);
        if (i< NUM_CHANNELS-1) Serial.print(",");
        else Serial.println();
      }
    #endif

    #if DEBUG_CONTROL == true
        Serial.print("remoteBalanceOffset:");
        Serial.println(remoteBalanceOffset);
        Serial.print("ZeparamremoteValue:");
        Serial.println(ZeparamremoteValue);
        Serial.print("steering:");
        Serial.println(steering);
        Serial.print("forwardBackward:");
        Serial.println(forwardBackward);
    #endif

    #if DEBUG_PID == true
        float _gy = gyroY + 2.92;
        if (_gy > -0.02 && _gy < 0.02) _gy = 0;
      Serial.print(motor1_vel);
      Serial.print(",");
      Serial.print(wheel_motor1_target);
      Serial.print(",");
      Serial.print(_gy);
      Serial.print(",");
      Serial.print(motor2_vel);
      Serial.print(",");
      Serial.print(wheel_motor2_target);
      Serial.print(",");
      Serial.print(Y1);
      Serial.print(",");
      Serial.print(y2);
      Serial.print(",");
      Serial.print(filteredPPMValues[0]);
      Serial.print(",");
      Serial.println(ZeparamremoteValue);
    #endif
  }
}