#include "HardwareSerial.h"
#include "Robot.h"

// Инициализация всех датчиков с настройкой
void Robot::init(bool is_button, bool is_mpu, bool is_dis, bool is_enc, bool is_servo, bool is_color)
{
  if (is_button)
    pinMode(BUTTON_PIN, OUTPUT);

  if (is_mpu)
  {
    mpu.init_gyro();
    if (is_button)
      mpu.gyro_calibration(BUTTON_PIN);
    else
      mpu.gyro_calibration(-1);
  }

  if (is_dis)
    init_dis();
  if (is_enc)
    init_encoder();
  if (is_servo)
    init_servo();
  if (is_color)
    init_color();
}

// Машина состояний, где переключаются текущие действия робота
void Robot::state_machine()
{
  // Serial.println(current_state);
  switch (current_state)
  {
  case WAIT:
    alg_right_hand();
    break;
  case MOVING:
    if (mov_forward())
      current_state = WAIT;
    break;
  case ROTATION_RIGHT:
    if (rot_right())
    {
      if (central_dist > DISTANCE || central_dist == -1)
        current_state = MOVING;
      else
        current_state = WAIT;
    }
    break;
  case ROTATION_LEFT:
    if (rot_left())
    {
      if (central_dist > DISTANCE || central_dist == -1)
        current_state = MOVING;
      else
        current_state = WAIT;
    }
    break;
  case GIVING:
    giving();

    break;
  }
}

// Вывод показания датчиков расстояния
void Robot::print_dis()
{
  Serial.print(" Right_R: ");
  Serial.print(right_dist);
  Serial.print(" Central_R: ");
  Serial.print(central_dist);
  Serial.print(" Left_R: ");
  Serial.println(left_dist);
}

// Вывод показания энкодеров
void Robot::print_enc()
{
  Serial.print("countL: ");
  Serial.print(countL);
  Serial.print(" ");
  Serial.print("countR: ");
  Serial.println(countR);
}

// Вывод карты построенной роботом
void Robot::print_map()
{
  graph.print_graph();
}

// Вывод с какой стороны обнаружен спаснабор и кол-во, которое нужно выдать
void Robot::print_save()
{
  Serial.print("Side: ");
  Serial.print(side);
  Serial.print(" ");
  Serial.print("Count: ");
  Serial.println(count_save);
}

// Вывод показания гироскопа
void Robot::print_gyro()
{
  mpu.print_roll_pitch_yaw();
}

// Движение вперед с выравниванием по боковым датчикам
bool Robot::mov_forward()
{
  int u, err = 0, right_err, left_err;

  bool is_stop_moving = false;

  right_err = DISTANCE_WALL - right_dist;
  left_err = DISTANCE_WALL - left_dist;

  if (right_dist > DISTANCE)
    right_err = 0;
  if (left_dist > DISTANCE)
    left_err = 0;

  err = left_err - right_err;

  u = err * K_WALL;

  if (central_dist < DISTANCE_WALL && central_dist != -1 && central_dist != 0)
    is_stop_moving = true;
  // else if ((countL + countR) / 2 >= CELL_SIZE_ENCODER) is_stop_moving = true;

  motors(SPEED + u, SPEED - u);

  if (is_stop_moving)
  {
    countL = 0;
    countR = 0;
    graph.add_by_angle(map_angle);
  }

  return is_stop_moving;
}

// Функция поворота с помощью гироскопа
bool Robot::rotate(float angle)
{
  float err = 0, u = 0, is_stop_rotate = false, delta, i;
  static float timer = millis(), timer_i = millis();
  static bool flag = true;

  if (flag)
  {
    mpu.yaw_first = 0;
    mpu.yaw_first = mpu.yaw();
    flag = false;
  }

  delta = millis() - timer_i;

  err = adduction(angle - mpu.yaw());

  i += err * delta;
  if (abs(i) > 10)
    i = 10 * sign(i);

  u = err * K_ROT + i * K_WALL_I;

  timer_i = millis();

  // if(abs(u) < 50) u = 50 * sign(u);
  if (abs(u) > 180)
    u = 180 * sign(u);

  motors(u, -u);

  if (abs(err) > K_STOP_ROTATE)
    timer = millis();
  if (millis() - timer > 1000)
    is_stop_rotate = true;

  if (is_stop_rotate)
  {
    countL = 0;
    countR = 0;
    flag = true;
  }

  return is_stop_rotate;
}

// Поворот на 90 градусов
int Robot::rot_right()
{
  return rotate(90);
}

// Поворот на -90 градусов
int Robot::rot_left()
{
  return rotate(-90);
}

// Обновление всех показаний с датчиков
void Robot::wait(int time_wait)
{
  float timer_wait = millis();

  do
  {
    right_dist = get_distance(sensor_r);
    central_dist = get_distance(sensor_u);
    left_dist = get_distance(sensor_l);

    if (Serial1.available())
    {
      side = 1;
      count_save = Serial1.read();
    }
    if (Serial2.available())
    {
      side = 2;
      count_save = Serial2.read();
    }

    mpu.update();
  } while (millis() - timer_wait < time_wait);
}

// Алгоритм правой ркуи для прохождения лабиринта
void Robot::alg_right_hand()
{
  if (right_dist > DISTANCE || right_dist == -1 || right_dist == 0)
  {
    current_state = ROTATION_RIGHT;

    if (left_dist > DISTANCE)
      graph.add_by_angle(map_angle, false);

    map_angle = adduction(map_angle - 90);
  }
  else if (central_dist > DISTANCE || central_dist == -1 || central_dist == 0)
  {
    current_state = MOVING;

    if (left_dist > DISTANCE)
      graph.add_by_angle(map_angle, false);
  }
  else
  {
    current_state = ROTATION_LEFT;
    map_angle = adduction(map_angle + 90);
  }

  Serial.println(current_state);
}

// Алгоритм левой руки
void Robot::alg_left_hand()
{
  if (left_dist > DISTANCE || left_dist == -1 || left_dist == 0)
  {
    current_state = ROTATION_LEFT;

    if (right_dist > DISTANCE)
      graph.add_by_angle(map_angle, false);

    map_angle = adduction(map_angle + 90);
  }
  else if (central_dist > DISTANCE || central_dist == -1 || central_dist == 0)
  {
    current_state = MOVING;

    if (right_dist > DISTANCE)
      graph.add_by_angle(map_angle, false);
  }
  else
  {
    current_state = ROTATION_RIGHT;
    map_angle = adduction(map_angle - 90);
  }

  Serial.println(current_state);
}

// Возврат к точке на карте где робот не был
void Robot::return_to_point()
{
  Serial.println("Lets goooo");
  node point = graph.get_not_discovered();
  Vec<enum moves> moves = graph.get_move(point, 0);

  for (int i = 0; i < moves.size(); i++)
  {
    switch (moves[i])
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

// Инициализация сервы
void Robot::init_servo()
{
  myservo.attach(SERVO_PIN);
  myservo.write(91);
}

// Выдача спаснабора
int Robot::giving()
{
  if (side == 1)
  {
    for (int i = 91; i < 180; i++)
    {
      myservo.write(i);
      delay(1);
    }

    for (int i = 180; i < 91; i--)
    {
      myservo.write(i);
      delay(1);
    }
  }
  else if (side == 2)
  {
    for (int i = 91; i < 0; i--)
    {
      myservo.write(i);
      delay(1);
    }

    for (int i = 0; i < 91; i++)
    {
      myservo.write(i);
      delay(1);
    }
  }
}

// Инициализация датчиков расстояния
void Robot::init_dis()
{
  pinMode(XSHUT_pin_r, OUTPUT);
  pinMode(XSHUT_pin_u, OUTPUT);
  pinMode(XSHUT_pin_l, OUTPUT);

  digitalWrite(XSHUT_pin_r, 0);
  digitalWrite(XSHUT_pin_u, 0);
  digitalWrite(XSHUT_pin_l, 0);

  Wire.begin();
  delay(500);

  digitalWrite(XSHUT_pin_r, 1);
  delay(100);
  sensor_r.setAddress(sensor_r_newAddress);
  delay(10);

  digitalWrite(XSHUT_pin_u, 1);
  delay(100);
  sensor_u.setAddress(sensor_u_newAddress);
  delay(10);

  digitalWrite(XSHUT_pin_l, 1);
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

// Функция для получения расстояния с датчиков
int Robot::get_distance(VL53L0X sensor)
{
  if (sensor.timeoutOccurred())
    Serial.print(" TIMEOUT");
  int sensor_dis = sensor.readRangeContinuousMillimeters(); //-50

  if (sensor_dis == 8191)
    return -1;

  if (sensor_dis == 8190)
    sensor_dis = sensor.sensor_dis_old;
  else
    sensor.sensor_dis_old = sensor_dis;

  return sensor_dis;
}

// Инициализация датчика цвета
void Robot::init_color()
{
  // сконфигурировать пины
  pinMode(color_S0, 1);
  pinMode(color_S1, 1);
  pinMode(color_S2, 1);
  pinMode(color_S3, 1);
  pinMode(color_OUT, 0);

  // масштабирование 20%
  digitalWrite(color_S0, 1);
  digitalWrite(color_S1, 0);
}
// Получение значения цвета на котором стоит робот
color Robot::get_color()
{
  int R = 0;
  int G = 0;
  int B = 0;

  // установить R фильтр
  digitalWrite(color_S2, 0);
  digitalWrite(color_S3, 0);

  // Получение частоты на выходе
  R = pulseIn(color_OUT, 0);

  // установить G фильтр
  digitalWrite(color_S2, 1);
  digitalWrite(color_S3, 1);

  // Получение частоты на выходе
  G = pulseIn(color_OUT, 0);

  // установить B фильтр
  digitalWrite(color_S2, 0);
  digitalWrite(color_S3, 1);

  // Получение частоты на выходе
  B = pulseIn(color_OUT, 0);

  if (in_range(R, RED_BLUE, COLOR_SPREAD) && in_range(G, GREEN_BLUE, COLOR_SPREAD) &&
      in_range(B, BLUE_BLUE, COLOR_SPREAD))
    return BLUE;

  if (in_range(R, RED_BLACK, COLOR_SPREAD) && in_range(G, GREEN_BLACK, COLOR_SPREAD) &&
      in_range(B, BLUE_BLACK, COLOR_SPREAD))
    return BLACK;

  return WHITE;
}

// Инициализация энкодеров
void Robot::init_encoder()
{
  attachInterrupt(3, encL, RISING);
  attachInterrupt(2, encR, RISING);
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

void Robot::handleEncL()
{
  countL++;
}

void Robot::handleEncR()
{
  countR++;
}

Robot *Robot::instance_;

// Подать значение на левый мотор
void Robot::motor_l(int value)
{
  int sign_v = sign(value);
  if (abs(value) > 255)
    value = 255 * sign_v;
  value = sign_v * (255 - abs(value));

  if (value == 0 && sign_v == 0)
  {
    digitalWrite(M1_1, 1);
    digitalWrite(M1_2, 1);
  }
  else
  {
    if (sign_v > 0)
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

// Подать значение на правый мотор
void Robot::motor_r(int value)
{
  int sign_v = -1 * sign(value);
  if (abs(value) > 255)
    value = 255 * sign_v;

  value = sign_v * (255 - abs(value));

  if (value == 0 && sign_v == 0)
  {
    digitalWrite(M2_1, 1);
    digitalWrite(M2_2, 1);
  }
  else
  {
    if (sign_v > 0)
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

// Подать значения на оба мотора (левый, правый)
void Robot::motors(int value_l, int value_r)
{
  if (DEBUG)
  {
    Serial.println(value_l);
    Serial.println(value_r);
  }
  else
  {
    motor_l(value_l);
    motor_r(value_r);
  }
}

// Остановить оба мотора
void Robot::motor_stop()
{
  if (DEBUG)
  {
    Serial.println("Left stop");
    Serial.println("Right stop");
  }
  else
  {
    motor_l(0);
    motor_r(0);
  }
}