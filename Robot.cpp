#include "Robot.h"

//Инициализация всех датчиков
int Robot::init()
{
  pinMode(BUTTON_PIN, OUTPUT);
  
  init_dis();
  init_encoder();
  init_servo();

  mpu.init_gyro();
}

//Расчет движения робота
bool Robot::mov_forward() 
{
  int u, err = 0, right_err, left_err;

  bool is_stop_moving = false;

  right_err = DISTANCE_WALL - right_dist;
  left_err = DISTANCE_WALL - left_dist;

  if(right_dist > DISTANCE) right_err = 0;
  if(left_dist > DISTANCE) left_err = 0;

  err = left_err - right_err;

  u = err * K_WALL;

  if (central_dist < DISTANCE_WALL) is_stop_moving = true;
  else if ((countL + countR) / 2 >= CELL_SIZE_ENCODER) is_stop_moving = true;

  motors(SPEED - u, SPEED + u);

  if(is_stop_moving)
  {
    countL = 0;
    countR = 0;
    graph.add_by_angle(map_angle);
  }

  return is_stop_moving;
}

bool Robot::rotate(float angle)
{
  float err = 0, u = 0, is_stop_rotate = false, delta, i;
  static float timer = millis(), timer_i = millis();
  static bool flag = true;

  if(flag)
  {
    mpu.yaw_first = 0;
    mpu.yaw_first = mpu.yaw();
    flag = false;
  }

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
    countL = 0;
    countR = 0;
    flag = true;
  }

  return is_stop_rotate;
}

int Robot::rot_right() 
{
  return rotate(90);
}

int Robot::rot_left() 
{
  return rotate(-90);
}

//Изменение состаяний робота и state машина
void Robot::alg_right_hand() 
{
  if (right_dist > DISTANCE || right_dist == -1) 
  {
    current_state = ROTATION_RIGHT;

    if(left_dist > DISTANCE) graph.add_by_angle(map_angle, false);

    map_angle = adduction(map_angle - 90);
  } 
  else if (central_dist > DISTANCE || central_dist == -1) 
  {
    current_state = MOVING;

    if(left_dist > DISTANCE) graph.add_by_angle(map_angle, false);
  } 
  else  
  {
    current_state = ROTATION_LEFT;
    map_angle = adduction(map_angle + 90);
  }
}

void Robot::alg_left_hand() 
{
  
}

void Robot::state_machine()
{
  switch(current_state)
  {
    case WAIT: 
      alg_right_hand();
      break;
    case MOVING: 
      if(mov_forward()) current_state = WAIT;
      break;
    case ROTATION_RIGHT: 
      if(rot_right()) 
      {
        if(central_dist > DISTANCE || central_dist == -1) current_state = MOVING;
        else current_state = WAIT;
      }
      break;
    case ROTATION_LEFT: 
      if(rot_left()) 
      {
        if(central_dist > DISTANCE || central_dist == -1) current_state = MOVING;
        else current_state = WAIT;
      }
      break;
    case GIVING:
      giving();

      break;
  }

  wait(1);
}

//Выдача спаснабора и инициализация сервы
void Robot::init_servo()
{
  myservo.attach(9);
  myservo.write(91);
}

int Robot::giving()
{
  
}

//Возврат к точке на карте где робот не был
void Robot::return_to_point() 
{
  Serial.println("Lets goooo");
  node point = graph.get_not_discovered();
  Vec<enum moves> moves = graph.get_move(point, 0);

  for (int i = 0; i < moves.size(); i++) 
  {
    switch(moves[i])
    {
      case ROTATE_RIGHT:
        current_state = ROTATION_RIGHT;
        break;
      case ROTATE_LEFT:
        current_state = ROTATION_LEFT;
        break;
      case MOVE_FORWARD:
        current_state = MOVING;
        break;
      default:
        break;
    }
  }
}

//Обновление всех показаний с датчиков
void Robot::wait(int time_wait)
{
  float timer_wait = millis();
  
  do 
  {
    right_dist = get_distance(sensor_r);
    central_dist = get_distance(sensor_u);
    left_dist = get_distance(sensor_l);

    mpu.update();
  }
  while(millis() - timer_wait < time_wait);
}

//Работа с энкодерами
void Robot::init_encoder()
{
  attachInterrupt(0, encL, RISING);
  attachInterrupt(1, encR, RISING);
  instance_ = this;
}

void Robot::encL() 
{
  instance_->handleEncL();
}

void Robot::encR() 
{
  instance_->handleEncR();
}

void Robot::handleEncL() {
  countL++;
}

void Robot::handleEncR() {
  countR++;
}

Robot * Robot::instance_;

//Работа с датчиками расстояния
void Robot::init_dis() 
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

int Robot::get_distance(VL53L0X sensor) 
{
  if (sensor.timeoutOccurred()) Serial.print(" TIMEOUT");
  int sensor_dis = sensor.readRangeContinuousMillimeters(); //-50

  if(sensor_dis == 8191) return -1;

  if(sensor_dis == 8190) sensor_dis = sensor.sensor_dis_old;
  else sensor.sensor_dis_old = sensor_dis;

  return sensor_dis;
}

void Robot::debug_dis() 
{
  Serial.print(" Right_R: ");
  Serial.print(right_dist);
  Serial.print(" Central_R: ");
  Serial.println(central_dist);
  Serial.print(" Left_R: ");
  Serial.println(left_dist);
  delay(50);
}

//Работа с моторами
void Robot::motor_l(int value) 
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

void Robot::motor_r(int value) 
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

void Robot::motors(int value_l, int value_r) 
{
  motor_l(value_l);
  motor_r(value_r);
}

void Robot::motor_stop() 
{
  motor_l(0);
  motor_r(0);
}