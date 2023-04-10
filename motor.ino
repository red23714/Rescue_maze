void motor_l(int value) 
{ 
  int sign_v = sign(value);
  if (abs(value) > 255) value = 255 * sign_v;
  value = sign_v*(255 - abs(value));
  
  if(value == 0 && sign_v == 0) 
  {
    digitalWrite(M1_1, 1);
    digitalWrite(M1_2, 1);
  }
  else
  {
    if(sign_v > 0)
    {
      digitalWrite(M1_1, 1);
      analogWrite(M1_2, abs(value));
    }
    else
    {
      digitalWrite(M1_2, 1);
      analogWrite(M1_1, abs(value));
    }
  }
}

void motor_r(int value) 
{ 
  int sign_v = sign(value);
  if (abs(value) > 255) value = 255 * sign_v;

  value = sign_v*(255 - abs(value));

  if(value == 0 && sign_v == 0) 
  {
    digitalWrite(M2_1, 1);
    digitalWrite(M2_2, 1);
  }
  else
  {
    if(sign_v > 0)
    {
      digitalWrite(M2_1, 1);
      analogWrite(M2_2, abs(value));
    }
    else
    {
      digitalWrite(M2_2, 1);
      analogWrite(M2_1, abs(value));
    }
  }
}

void motors(int value_l, int value_r) 
{
  motor_l(value_l);
  motor_r(value_r);
}

void motor_stop() 
{
  motor_l(0);
  motor_r(0);
}

bool mov_forward(Map_data* map_data, Distance_data* distance_data) 
{
#if DEBUG_HAND
  Serial.println("Move forward ");
  delay(10);
#else

#if DEBUG 
  Serial.println("Move forward ");
  delay(10);
#endif

  int u, err = 0, right_k, left_k;

  bool is_stop_moving = false;

  right_k = DISTANCE_WALL - distance_data->right_dist;
  left_k = DISTANCE_WALL - distance_data->left_dist;

  if(distance_data->right_dist > DISTANCE) right_k = 0;
  if(distance_data->left_dist > DISTANCE) left_k = 0;

  err = left_k - right_k;

  u = err * K_WALL;

  if (distance_data->central_dist < DISTANCE_WALL) is_stop_moving = true;
  else if ((countL + countR) / 2 >= CELL_SIZE_ENCODER) is_stop_moving = true;
  // else if(abs(CELL_SIZE - err_dis) < 30) is_stop_moving = true;

  motors(SPEED - u, SPEED + u);

  if(is_stop_moving)
  {
    countL = 0;
    countR = 0;
    add_by_angle(map_data);
  }

  return is_stop_moving;
#endif
}

bool rotate(float angle)
{
#if DEBUG 
  Serial.print("Rotate ");
  Serial.println(angle);
#endif
  
  float err = 0, u = 0, is_stop_rotate = false, delta, i;
  static float timer = millis(), timer_i = millis();
  static bool flag = true;
  if(flag)
  {
    mpu.yaw_first = 0;
    mpu.yaw_first = mpu.yaw();
    flag = false;
  }

  #if DEBUG_ENC
    Serial.print("countL = ");
    Serial.print(countL);
    Serial.print(" countR = ");
    Serial.println(countR);
  #endif

  delta = millis() - timer_i;

  err = adduction(angle - mpu.yaw());

  i += err * delta;
  if(abs(i) > 10) i = 10 * sign(i);

  u = err * K_ROT + i * K_WALL_I;

  timer_i = millis();

  // if(abs(u) < 50) u = 50 * sign(u);
  if(abs(u) > 180) u = 180 * sign(u);

  motors(-u, u);

  if(abs(err) > K_STOP_ROTATE) timer = millis();
  if(millis() - timer > 1000) is_stop_rotate = true;

  if(is_stop_rotate)
  {
    countR = 0;
    countL = 0;
    flag = true;
  }

  return is_stop_rotate;
}

int rot_right() 
{
  return rotate(90);
}

int rot_left() 
{
  return rotate(-90);
}
