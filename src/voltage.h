int low_voltage_flag = 1;
unsigned long vol_measure_time = 0;
void voltageInit()
{
  analogReference(INTERNAL);
}
void voltageMeasure() //测量电压
{
  if (millis() - vol_measure_time > 1000) //每1000毫秒测量一次
  {
    vol_measure_time = millis();
    double voltage = (analogRead(VOL_MEASURE_PIN) * 1.1 / 1024) * ((10 + 1.5) / 1.5); //读取电压值
    if (voltage > 7.8)                                                                //电池供电
    {
      if (low_voltage_flag == 1)
      {
        rgb.lightOff();
        digitalWrite(STBY_PIN,HIGH);
      }
      low_voltage_flag = 0; //满电压标志
    }
    else
    {
      if (voltage < 7.0) //The battery is low in power and needs to be charged.
      {
        motion_mode = STOP;
        digitalWrite(STBY_PIN,LOW);
      }
      //待机,停止,自动启动, 启动模式
      if (motion_mode == STANDBY || motion_mode == STOP || motion_mode == START)
      {
        rgb.flashRedColor();
      }
      low_voltage_flag = 1; //低电压标志
    }
  }
}
