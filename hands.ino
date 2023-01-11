void right_hand() 
{
  if (get_distance(&sensor_r) > DISTANCE) 
  {
    rot_right();
  } 
  else if (get_distance(&sensor_u) > DISTANCE) 
  {
    mov_forward();
  } 
  else  
  {
    rot_left();
  } 
}

void left_hand() 
{
  if (get_distance(&sensor_l) > DISTANCE) 
  {
    rot_left();
  } 
  else if (get_distance(&sensor_u) > DISTANCE) 
  {
    mov_forward();
  } 
  else
  {
    rot_right();
  } 
}