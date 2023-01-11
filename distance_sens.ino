void init_dis() 
{
  pinMode(XSHUT_pin1, OUTPUT);
  pinMode(XSHUT_pin2, OUTPUT);
  pinMode(XSHUT_pin3, OUTPUT);

  Wire.begin();
  delay(500);

  sensor_r.setAddress(sensor_r_newAddress);
  pinMode(XSHUT_pin1, INPUT);
  delay(10);

  sensor_u.setAddress(sensor_u_newAddress);
  pinMode(XSHUT_pin2, INPUT);
  delay(10);

  sensor_l.setAddress(sensor_l_newAddress);
  pinMode(XSHUT_pin3, INPUT);
  delay(10);

  sensor_r.init();
  sensor_u.init();
  sensor_l.init();

  delay(2000);

  sensor_r.setTimeout(500);
  sensor_u.setTimeout(500);
  sensor_l.setTimeout(500);

  sensor_r.startContinuous();
  sensor_u.startContinuous();
  sensor_l.startContinuous();
}

int get_distance(VL53L0X* sensor) 
{
  if (sensor->timeoutOccurred()) Serial.print(" TIMEOUT");
  return sensor->readRangeContinuousMillimeters();
}

void debug_dis() 
{
  int right_S = get_distance(&sensor_r);
  if (right_S > 700) right_S = 700;

  int central_S = get_distance(&sensor_u);
  if (central_S > 700) central_S = 700;

  int left_S = get_distance(&sensor_l); //sensor_r.readRangeContinuousMillimeters()
  if (left_S > 700) left_S = 700;

  Serial.print(" Right_R: ");
  Serial.print(right_S);
  Serial.print(" Central_R: ");
  Serial.println(central_S);
  Serial.print(" Left_R: ");
  Serial.println(left_S);
  delay(50);
}