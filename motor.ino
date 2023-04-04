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

bool mov_forward(int* map_angle) 
{
#if DEBUG_HAND
  Serial.println("Move forward ");
  delay(10);
#else

#if DEBUG 
  Serial.println("Move forward ");
  delay(10);
#endif

  int u, err = 0, u_dis, right_k, left_k;
  static int err_dis = 0;
  bool is_stop_moving = false;

  right_k = DISTANCE_WALL - get_distance(&sensor_r);
  left_k = DISTANCE_WALL - get_distance(&sensor_l);

  if(get_distance(&sensor_r) > DISTANCE) right_k = 0;
  if(get_distance(&sensor_l) > DISTANCE) left_k = 0;

  err = left_k - right_k;

  if(!vlFlag1 && !vlFlag2) vlFlag3 = 1;
  else vlFlag3 = 0;
  err_dis += (get_delta_distance_up() * vlFlag1 + get_delta_distance_back() * vlFlag2 + get_delta_encoder() * vlFlag3) / (vlFlag1 + vlFlag2 + vlFlag3);

  u = err * K_WALL;
  u_dis = (CELL_SIZE - err_dis) * K_DIS;

  if (get_distance(&sensor_u) < DISTANCE_WALL) is_stop_moving = true;
  // else 
  // {
  //   if(get_distance(&sensor_u) > 1000)
  //   {
  //     if(get_distance(&sensor_b) - distance_old > CELL_SIZE) //&& abs(err) < K_CALIBRATION
  //     {
  //       distance_old = get_distance(&sensor_b); 
  //       is_stop_moving = true;
  //     }
  //   }
  //   else
  //   {
  //     if(distance_old - get_distance(&sensor_u) > CELL_SIZE) //&& abs(err) < K_CALIBRATION
  //     {
  //       distance_old = get_distance(&sensor_u); 
  //       is_stop_moving = true;
  //     }
  //   }
  // }
  // // else if ((countL + countR) / 2 >= CELL_SIZE && get_distance(&sensor_u) >  DISTANCE_WALL + 30) is_stop_moving = true;

  motors(SPEED - u + u_dis, SPEED + u + u_dis);

  if(is_stop_moving)
  {
    countL = 0;
    countR = 0;
    add_by_angle(map_angle);
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
    yaw_first = 0;
    yaw_first = yaw();
    flag = false;
  }

  #if DEBUG_ENC
    Serial.print("countL = ");
    Serial.print(countL);
    Serial.print(" countR = ");
    Serial.println(countR);
  #endif

  delta = millis() - timer_i;

  err = adduction(angle - yaw());

  i += err * delta;
  if(abs(i) > 10) i = 10 * sign(i);

  u = err * ROT_K + i * K_WALL_I;

  timer_i = millis();

  // if(abs(u) < 50) u = 50 * sign(u);
  if(abs(u) > 180) u = 180 * sign(u);

  motors(-u, u);

  if(abs(err) > K_STOP_ROTATE) timer = millis();
  if(millis() - timer > 1000) is_stop_rotate = true;

  if(is_stop_rotate)
  {
    angle_err = yaw();
    countR = 0;
    countL = 0;
    flag = true;
    
    if(get_distance(&sensor_u) > 1000) distance_old = get_distance(&sensor_u);
    else distance_old = get_distance(&sensor_b);
  
    if(get_distance(&sensor_u) > DISTANCE)
    {
      current_state = MOVING;
      return false;
    }
  }

  return is_stop_rotate;
}

int rot_right(int* map_angle) 
{
  *map_angle = adduction(*map_angle + 90);
  return rotate(90);
}

int rot_left(int* map_angle) 
{
  *map_angle = adduction(*map_angle - 90);
  return rotate(-90);
}
