#ifndef UTILS_H
#define UTILS_H

inline float adduction(float angle)
{
    while(angle > 180) angle -= 360;
    while(angle < -180) angle += 360;

    return angle;
}

inline int sign(float value)
{
  if(value == 0) return 0;
  if(value < 0) return -1;
  if(value > 0) return 1; 
}

#endif