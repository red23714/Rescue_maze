void motor_l(int value) 
{
  if (abs(value) > 255) value = 255 * sign(value);

  if(value == 0) 
  {
    digitalWrite(M1_1, 1);
    digitalWrite(M1_2, 1);
  }
  else
  {
    if(value > 0)
    {
      digitalWrite(M1_1, 1);
      analogWrite(M1_2, value);
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
  if (abs(value) > 255) value = 255 * sign(value);

  if(value == 0) 
  {
    digitalWrite(M2_1, 1);
    digitalWrite(M2_2, 1);
  }
  else
  {
    if(value > 0)
    {
      digitalWrite(M2_1, 1);
      analogWrite(M2_2, value);
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

  motor_l(SPEED);
  motor_r(SPEED);
  while ((countL + countR) / 2 <= 1800 || get_distance(&sensor_u) >  DISTANCE) 
  {
    #if DEBUG_ENC
      Serial.print("countL = ");
      Serial.print(countL);
      Serial.print(" countR = ");
      Serial.println(countR);
    #endif
    Serial.print("");
    delay(10);
  } 
  
  motor_stop();
  countL = 0;
  countR = 0;
#endif
}

void mov_back() {
#if DEBUG_HAND
  Serial.println("Move back ");
  delay(10);
#else

  motor_l(-SPEED);
  motor_r(-SPEED);
  while ((countL + countR) / 2 <= 1800) 
  {
    #if DEBUG_ENC
      Serial.print("countL = ");
      Serial.print(countL);
      Serial.print(" countR = ");
      Serial.println(countR);
    #endif
    Serial.print("");
    delay(10);
  } 
  
  motor_stop();
  countL = 0;
  countR = 0;
#endif
}

void rot_right() 
{
#if DEBUG_HAND
  Serial.println("Rotate right ");
  delay(10);
#else

  motor_l(SPEED);
  motor_r(-SPEED);
  while(countL <= 1150 || (get_distance(&sensor_r) - get_distance(&sensor_l) < 10)) 
  {
    #if DEBUG_ENC
      Serial.print("countL = ");
      Serial.print(countL);
      Serial.print(" countR = ");
      Serial.println(countR);
    #endif
    Serial.print("");
    delay(10);
  } 
    
  motor_stop();
  countR = 0;
  countL = 0;
#endif
}

void rot_left() 
{
#if DEBUG_HAND
  Serial.println("Rotate left ");
#else

  motor_l(-SPEED);
  motor_r(SPEED);

  while (countR <= 1150 || (get_distance(&sensor_l) - get_distance(&sensor_r) < 10))
  {
    #if DEBUG_ENC
      Serial.print("countL = ");
      Serial.print(countL);
      Serial.print(" countR = ");
      Serial.println(countR);
    #endif
    Serial.print("");
    delay(10);
  }

  motor_stop();
  countR = 0;
  countL = 0;
#endif
}





