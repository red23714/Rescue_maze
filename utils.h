#ifndef UTILS_H
#define UTILS_H

//Ограничивает значения исходного угла в диапозоне -180; 180
inline float adduction(float angle)
{
    while(angle > 180) angle -= 360;
    while(angle < -180) angle += 360;

    return angle;
}

//Возвращает знак исходного числа или ноль
inline int sign(float value)
{
  if(value == 0) return 0;
  if(value < 0) return -1;
  if(value > 0) return 1; 
}

inline bool in_range(float value, float edge, float spread)
{
  if(value < edge - spread || value > edge + spread) return false;
  else return true;
}

#endif