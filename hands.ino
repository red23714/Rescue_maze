void right_hand() 
{
  if (get_distance(&sensor_r) > DISTANCE) 
  {
    current_state = ROTATION_RIGHT;
    if(get_distance(&sensor_u) > DISTANCE)
    {
      current_state = MOVING;
    }
  } 
  else if (get_distance(&sensor_u) > DISTANCE) 
  {
    current_state = MOVING;
  } 
  else  
  {
    current_state = ROTATION_LEFT;
    if(get_distance(&sensor_u) > DISTANCE)
    {
      current_state = MOVING;
    }
  }
}

void left_hand() 
{
  if (get_distance(&sensor_l) > DISTANCE) 
  {
    current_state = ROTATION_LEFT;
  } 
  else if (get_distance(&sensor_u) > DISTANCE) 
  {
    current_state = MOVING;
  } 
  else
  {
    current_state = ROTATION_RIGHT;
  } 
}
