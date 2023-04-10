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

void wait(int time_wait, Distance_data* distance_data)
{
  if(time_wait == 0) {mpu.update(); return;}
  
  float timer_wait = millis();
  
  while(millis() - timer_wait < time_wait)
  {
    mpu.update();
  }
  
  distance_data->right_dist = get_distance(&sensor_r);
  distance_data->central_dist = get_distance(&sensor_u);
  distance_data->left_dist = get_distance(&sensor_l);
}