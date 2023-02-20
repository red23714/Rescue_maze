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

bool mov_forward() {
#if DEBUG_HAND
  Serial.println("Move forward ");
  delay(10);
#else

#if DEBUG 
  Serial.println("Move forward ");
  delay(10);
#endif

  int u, u_dis, err_dis = 0, err = 0, right_k, left_k, is_stop_moving = false;

  if (get_distance(&sensor_u) < DISTANCE_WALL) is_stop_moving = true;
  else if(distance_old - get_distance(&sensor_u) > CELL_DIST) {distance_old = get_distance(&sensor_u); is_stop_moving = true;}
  // else if ((countL + countR) / 2 >= CELL_SIZE && get_distance(&sensor_u) >  DISTANCE_WALL + 30) is_stop_moving = true;

  right_k = DISTANCE_WALL - get_distance(&sensor_r);
  left_k = DISTANCE_WALL - get_distance(&sensor_l);

  if(get_distance(&sensor_r) > DISTANCE) right_k = 0;
  if(get_distance(&sensor_l) > DISTANCE) left_k = 0;

  err = left_k - right_k;
  // err_dis = cell_count * CELL_SIZE - get_distance(&sensor_u);

  u = err * K_WALL;
  u_dis = err_dis * K_DIS;

  motors(SPEED - u, SPEED + u);

  if(is_stop_moving)
  {
    countL = 0;
    countR = 0;
    cell_count = 0;
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
  
  float err = 0, u = 0, is_stop_rotate = false;
  static float timer = millis();
  yaw_first = 0;
  yaw_first = yaw();

  #if DEBUG_ENC
    Serial.print("countL = ");
    Serial.print(countL);
    Serial.print(" countR = ");
    Serial.println(countR);
  #endif

  err = adduction(angle - yaw());
  u = err * ROT_K;

  if(abs(u) < 50) u = 50 * sign(u);
  if(abs(u) > 180) u = 180 * sign(u);

  motor_l(-u);
  motor_r(u);

  if(abs(err) > K_STOP_ROTATE) timer = millis();
  if(millis() - timer > 1000) is_stop_rotate = true;

  Serial.println(timer);

  distance_old = get_distance(&sensor_u);
  cell_count = round(distance_old / CELL_SIZE);

  if(is_stop_rotate)
  {
    angle_err = yaw();
    countR = 0;
    countL = 0;
    if(get_distance(&sensor_u) > DISTANCE)
    {
      mov_forward();
      current_state = MOVING;
      add_by_angle();
    }
  }

  return is_stop_rotate;
}

int rot_right() 
{
  map_angle = adduction(map_angle + 90);
  return rotate(90);
}

int rot_left() 
{
  map_angle = adduction(map_angle - 90);
  return rotate(-90);
}
