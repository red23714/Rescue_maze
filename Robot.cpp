#include "Arduino.h"
#include "HardwareSerial.h"
#include "Robot.h"

// Инициализация всех датчиков с настройкой
void Robot::init(bool is_button, bool is_dis, bool is_enc, bool is_servo, bool is_color)
{
  mpu.set_yaw_first(mpu.get_yaw());
  mpu.set_pitch_first(mpu.get_pitch());

  if (is_button) pinMode(BUTTON_PIN, INPUT_PULLUP);

  if (is_dis)
  {
    sensor_r.init_dis();
    sensor_u.init_dis();
    sensor_l.init_dis();
  }
  
  if (is_enc) init_encoder();
  if (is_servo) init_servo();
  if (is_color) color_sens.init_color();
}

// Машина состояний, где переключаются текущие действия робота
void Robot::state_machine()
{
  switch (current_state)
  {
  case WAIT:
    if(is_giving) 
    {
      current_state = state::GIVING;
    }
    else 
    {
      alg_right_hand();
    }

    break;
  case state::MOVING:
    if (mov_forward()) current_state = state::WAIT;
    break;
  case state::ROTATION_RIGHT:
    if (rot_right())
    {
      if ((central_dist > DISTANCE || central_dist == -1) && !is_giving) current_state = state::MOVING;
      else current_state = state::WAIT;
    }
    break;
  case state::ROTATION_LEFT:
    if (rot_left())
    {
      if ((central_dist > DISTANCE || central_dist == -1) && !is_giving) current_state = state::MOVING;
      else current_state = state::WAIT;
    }
    break;
  case state::GIVING:
    motor_stop();
    giving(camera_l.get_side(), camera_l.get_letter());
    break;
  case state::STANDING:
    motor_stop();
    wait(5000);
    is_stand = true;
    current_state = state::MOVING;
    break;
  case state::RETURN:
    return_to_point();
    break;
  }
}

// Вывод показаний датчиков расстояния
void Robot::print_dis()
{
  Serial.print(" Right_R: ");
  Serial.print(right_dist);
  Serial.print(" Central_R: ");
  Serial.print(central_dist);
  Serial.print(" Left_R: ");
  Serial.println(left_dist);
}

// Вывод показаний энкодеров
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
  camera_l.print_camera();
}

// Вывод показаний гироскопа
void Robot::print_gyro()
{
  mpu.print_gyro();
}

// Вывод показаний датчика цвета
void Robot::print_color()
{
  color_sens.print_color();
}

void Robot::print_current_state()
{
  Serial.print("Current state: ");
  switch (current_state)
  {
  case state::WAIT:
    Serial.println("WAIT");
    break;

  case state::MOVING:
    Serial.println("MOVING");
    break;

  case state::ROTATION_LEFT:
    Serial.println("ROTATION_LEFT");
    break;

  case state::ROTATION_RIGHT:
    Serial.println("ROTATION_RIGHT");
    break;

  case state::GIVING:
    Serial.println("GIVING");
    break;

  case state::RETURN:
    Serial.println("RETURN");
    break;

  case state::STANDING:
    Serial.println("STANDING");
    break;
  
  default:
    break;
  }
}

// Движение вперед с выравниванием по боковым датчикам
bool Robot::mov_forward()
{
  int u, err = 0, right_err, left_err;

  bool is_stop_moving = false;

  right_err = DISTANCE_WALL - right_dist;
  left_err = DISTANCE_WALL - left_dist;

  if (right_dist > DISTANCE) right_err = 0;
  if (left_dist > DISTANCE) left_err = 0;

  err = left_err - right_err;

  u = err * K_WALL;

  if (central_dist < DISTANCE_WALL_CENTER && central_dist != -1 && central_dist != 0) is_stop_moving = true;
  else if ((countL + countR) / 2 >= CELL_SIZE_ENCODER) is_stop_moving = true;

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
    mpu.set_yaw_first(mpu.get_yaw());
    flag = false;
  }

  delta = millis() - timer_i;

  err = adduction(angle - mpu.get_yaw());

  i += err * delta;
  if (abs(i) > 10) i = 10 * sign(i);

  u = err * K_ROT + i * K_WALL_I;

  timer_i = millis();

  // if(abs(u) < 50) u = 50 * sign(u);
  if (abs(u) > 180) u = 180 * sign(u);

  motors(u, -u);

  if (abs(err) > K_STOP_ROTATE) timer = millis();
  if (millis() - timer > 1000) is_stop_rotate = true;

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
    for (int i = 0; i < sizeof(periph) / sizeof(periph[0]); i++)
    {
      periph[i]->update();
    }

    right_dist = sensor_r.get_sensor_dis();
    central_dist = sensor_u.get_sensor_dis();
    left_dist = sensor_l.get_sensor_dis();

    color color = color_sens.get_color();
    if(color == color::BLUE && !is_stand) current_state = state::STANDING; 
    if(color == color::WHITE) is_stand = false;

    letter l = camera_l.get_letter();
    if (!is_giving && l != letter::N && 
        (graph_length_old != graph.get_graph_length() || map_angle_old != map_angle))
    {
      graph_length_old = graph.get_graph_length();
      map_angle_old = map_angle;
      is_giving = true;
      camera_l.set_is_update(false);
    }

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
  myservo.write(START_POS_SERVO);
}

// Выдача спаснабора
int Robot::giving(int side_in, letter count_save)
{
  int l = 0;
  switch (count_save)
  {
  case letter::S:
    l = 2;
    break;

  case letter::H:
    l = 1;
    break;

  default:
    print_save();
    break;
  }

  for (int n = 0; n < l; n++)
  {
    if (side_in == 1)
    {
      for (int i = START_POS_SERVO; i < 180; i++)
      {
        myservo.write(i);
        delay(15);
      }

      for (int i = 180; i > START_POS_SERVO; i--)
      {
        myservo.write(i);
        delay(15);
      }
    }
    else if (side_in == 2)
    {
      for (int i = START_POS_SERVO; i > 0; i--)
      {
        myservo.write(i);
        delay(15);
      }

      for (int i = 0; i < START_POS_SERVO; i++)
      {
        myservo.write(i);
        delay(15);
      }
    }
  }
  current_state = state::WAIT;
  is_giving = false;
  camera_l.set_is_update(true);
}

// Инициализация энкодеров
void Robot::init_encoder()
{
  attachInterrupt(1, encL, RISING);
  attachInterrupt(0, encR, RISING);
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

// Остановить оба мотора (левый, правый)
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