int sign(float value)
{
  if(value == 0) return 0;
  return value / abs(value);
}

float adduction(float angle)
{
    while(angle > 180) angle -= 360;
    while(angle < -180) angle += 360;

    return angle;
}

void wait(int time_wait)
{
  if(time_wait == 0) {mpu.update(); return;}
  float timer_wait = millis();
  while(millis() - timer_wait < time_wait)
  {
    mpu.update();
  }
}