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

void wait(int time)
{
  if(time == 0) {mpu.update(); return;}
  float timer = millis();
  while(millis() - timer < time)
  {
    mpu.update();
  }
}