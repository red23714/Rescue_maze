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

  if(get_distance(&sensor_r) < DISTANCE)
  {
    dir = 1;
  }

  if(get_distance(&sensor_l) < DISTANCE)
  {
    dir = 2;
  }

  while (get_distance(&sensor_u) >  DISTANCE) 
  {
    err = 0;
    if(dir == 1 && abs(get_distance(&sensor_r) - DISTANCE) < 100) err = get_distance(&sensor_r) - DISTANCE;
    if(dir == 2 && abs(get_distance(&sensor_l) - DISTANCE) < 100) err = get_distance(&sensor_l) - DISTANCE;

    u = err * K_DIS;
    motors(SPEED + u, SPEED - u);
    delay(10);
  } 
  
  motor_stop();
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
  float last_yaw = yaw();

  while (true)
  {
    #if DEBUG_ENC
      Serial.print("countL = ");
      Serial.print(countL);
      Serial.print(" countR = ");
      Serial.println(countR);
    #endif
    
    if(mpu.update())
    {
      err = adduction(angle - (yaw() - last_yaw));
      u = err * ROT_K;
    }

    if(abs(u) < 50) u = 50 * sign(u);
    
    motor_l(-u);
    motor_r(u);

    if(abs(err) > 3) timer = millis();
    if(millis() - timer > 500) break;

//    Serial.print(angle);
//    Serial.print(" ");
//    Serial.print(yaw());
//    Serial.print(" ");
//    Serial.print(u); 
//    Serial.print(" ");
//    Serial.println(err);
  }

  motor_stop();
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
