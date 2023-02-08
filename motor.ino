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

void mov_forward() {
#if DEBUG_HAND
  Serial.println("Move forward ");
  delay(10);
#else

#if DEBUG 
  Serial.println("Move forward ");
  delay(10);
#endif

  int u, err = 0, dir;

  //if(abs(yaw() - angle_err) < 50) rotate(yaw() - angle_err);

  while (get_distance(&sensor_u) >  DISTANCE_WALL) 
  {
    err = 0;
    err = get_distance(&sensor_r) - get_distance(&sensor_l);

    u = err * K_DIS;
    motors(SPEED - u, SPEED + u);
  } 
  
  motor_stop();
  // delay(1000);
  countL = 0;
  countR = 0;
#endif
}

void rotate(float angle)
{
#if DEBUG 
  Serial.print("Rotate ");
  Serial.println(angle);
#endif

  while(!mpu.update());
  
  float err = 0, u = 0, timer = millis();
  yaw_first = 0;
  yaw_first = yaw();

  while (true)
  {
    #if DEBUG_ENC
      Serial.print("countL = ");
      Serial.print(countL);
      Serial.print(" countR = ");
      Serial.println(countR);
    #endif
    
    while(!mpu.update());
    
    err = adduction(angle - yaw());
    u = err * ROT_K;
    

    if(abs(u) < 50) u = 50 * sign(u);
    if(abs(u) > 180) u = 180 * sign(u);
    
    motor_l(-u);
    motor_r(u);

    if(abs(err) > K_STOP_ROTATE) timer = millis();
    if(millis() - timer > 1000) break;

    // Serial.print(angle);
    // Serial.print(" ");
    // Serial.print(yaw());
    // Serial.print(" ");
    // Serial.print(u); 
    // Serial.print(" ");
    //Serial.println(err);
  }

  motor_stop();
  wait(1000);
  // delay(1000);
  angle_err = yaw();
  countR = 0;
  countL = 0;
}

void rot_right() 
{
  rotate(90);
}

void rot_left() 
{
  rotate(-90);
}
