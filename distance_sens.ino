void init_dis() 
{
  pinMode(XSHUT_pin1, OUTPUT);
  pinMode(XSHUT_pin2, OUTPUT);
  pinMode(XSHUT_pin3, OUTPUT);

  digitalWrite(XSHUT_pin1, 0);
  digitalWrite(XSHUT_pin2, 0);
  digitalWrite(XSHUT_pin3, 0);

  Wire.begin();
  delay(500);

  digitalWrite(XSHUT_pin1, 1);
  delay(100);
  sensor_r.setAddress(sensor_r_newAddress);
  delay(10);

  digitalWrite(XSHUT_pin2, 1);
  delay(100);
  sensor_u.setAddress(sensor_u_newAddress);
  delay(10);

  digitalWrite(XSHUT_pin3, 1);
  delay(100);
  sensor_l.setAddress(sensor_l_newAddress);
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

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor_r.setSignalRateLimit(0.1);
  sensor_u.setSignalRateLimit(0.1);
  sensor_l.setSignalRateLimit(0.1);
#endif
}

int get_distance(VL53L0X* sensor) 
{
  if (sensor->timeoutOccurred()) Serial.print(" TIMEOUT");
  int sensor_dis = sensor->readRangeContinuousMillimeters(); //-50

  if(sensor_dis == 8191) return -1;

  if(sensor_dis == 8190) sensor_dis = sensor->sensor_dis_old;
  else sensor->sensor_dis_old = sensor_dis;

  return sensor_dis;
}

void debug_dis() 
{
  int right_S = get_distance(&sensor_r);
  int central_S = get_distance(&sensor_u);
  int left_S = get_distance(&sensor_l); //sensor_r.readRangeContinuousMillimeters()

  Serial.print(" Right_R: ");
  Serial.print(right_S);
  Serial.print(" Central_R: ");
  Serial.println(central_S);
  Serial.print(" Left_R: ");
  Serial.println(left_S);
  delay(50);
}