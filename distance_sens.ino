void init_dis() 
{
  pinMode(XSHUT_pin1, OUTPUT);
  pinMode(XSHUT_pin2, OUTPUT);
  pinMode(XSHUT_pin3, OUTPUT);
  pinMode(XSHUT_pin4, OUTPUT);

  digitalWrite(XSHUT_pin1, 0);
  digitalWrite(XSHUT_pin2, 0);
  digitalWrite(XSHUT_pin3, 0);
  digitalWrite(XSHUT_pin4, 0);

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

  digitalWrite(XSHUT_pin4, 1);
  delay(100);
  sensor_b.setAddress(sensor_b_newAddress);
  delay(10);

  sensor_r.init();
  sensor_u.init();
  sensor_l.init();
  sensor_b.init();

  delay(2000);

  sensor_r.setTimeout(500);
  sensor_u.setTimeout(500);
  sensor_l.setTimeout(500);
  sensor_b.setTimeout(500);

  sensor_r.startContinuous();
  sensor_u.startContinuous();
  sensor_l.startContinuous();
  sensor_b.startContinuous();

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor_r.setSignalRateLimit(0.1);
  sensor_u.setSignalRateLimit(0.1);
  sensor_l.setSignalRateLimit(0.1);
  sensor_b.setSignalRateLimit(0.1);
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

int get_delta_distance_up()
{
  int dis_old = sensor_u.sensor_dis_old;
  int dist = get_distance(&sensor_u);

  if(dist == -1 || dist > 700 ||
     dis_old == -1 || dis_old > 700) vlFlag1 = 0;
  else vlFlag1 = 1;

  return dis_old - dist;
}

int get_delta_distance_back()
{
  int dis_old = sensor_b.sensor_dis_old;
  int dist = get_distance(&sensor_b);

  if(dist == -1 || dist > 700 ||
     dis_old == -1 || dis_old > 700) vlFlag2 = 0;
  else vlFlag2 = 1;

  return dist - dis_old;
}

void debug_dis() 
{
  int right_S = get_distance(&sensor_r);
  int central_S = get_distance(&sensor_u);
  int left_S = get_distance(&sensor_l); //sensor_r.readRangeContinuousMillimeters()
  int back_S = get_distance(&sensor_b);

  Serial.print(" Right_R: ");
  Serial.print(right_S);
  Serial.print(" Central_R: ");
  Serial.println(central_S);
  Serial.print(" Left_R: ");
  Serial.print(left_S);
  Serial.print(" Back_R: ");
  Serial.println(back_S);
  delay(50);
}