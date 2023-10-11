//Порты моторов
#define M1_1 7
#define M1_2 9
#define M2_1 5 // 13
#define M2_2 4 // 10

//Порт серво-мотора
#define SERVO_PIN 11

//Адреса датчиков расстояния
#define sensor_r_newAddress 42
#define sensor_u_newAddress 43
#define sensor_l_newAddress 44

//Порт кнопки
#define BUTTON_PIN 32

//Порты XSHUT для датчиков расстояния
#define XSHUT_pin_r 22
#define XSHUT_pin_u 25
#define XSHUT_pin_l 23

#define START_POS_SERVO 80

//Флаг который позволяет датчикам расстояния видеть большее расстояния, уменьшается точность
// #define LONG_RANGE

//Расстояние, которое должно быть между боковым датчиком расстояния и стеной 120
#define DISTANCE_WALL 120 // 120
#define DISTANCE_WALL_CENTER 80 // 90
//Расстояние для проверки находиться ли сбоку стена 170
#define DISTANCE 170

//Порты датчика цвета
#define color_S0 4
#define color_S1 5
#define color_S2 6
#define color_S3 7
#define color_OUT 8

//Значения RGB для синего
#define RED_BLUE 0
#define GREEN_BLUE 0
#define BLUE_BLUE 0
//Значения RGB для черного
#define RED_BLACK 0
#define GREEN_BLACK 0
#define BLUE_BLACK 0
//Погрешность при котором идеальные значения могут не совпадать с реальными параметрами цвета
#define COLOR_SPREAD 20

//Средняя скорость робота
#define SPEED 100

//Кол-во оборотов, которое нужно совершить мотору, чтобы проехать одну ячейку
#define CELL_SIZE_ENCODER 2600 // 1700

//Коэффициент резкости поворота при езде вдоль стены 1.5
#define K_WALL 1.5
//Коэффициент интегральной составляющей регулятора при повороте 5
#define K_WALL_I 5
//Коэффициент 20
#define K_CALIBRATION 20
//Коэффициент пропорциональной составляющей регулятора при повороте 10
#define K_ROT 10
//Минимальное значение ошибки, выше которого робот перестает поворачиваться 3
#define K_STOP_ROTATE 3