void alg_right_hand(int* map_angle) 
{
  if (get_distance(&sensor_r) > DISTANCE || get_distance(&sensor_r) == -1) 
  {
    current_state = ROTATION_RIGHT;

    if(get_distance(&sensor_l) > DISTANCE) add_by_angle(map_angle, false);

    *map_angle = adduction(*map_angle - 90);
  } 
  else if (get_distance(&sensor_u) > DISTANCE || get_distance(&sensor_u) == -1) 
  {
    current_state = MOVING;

    if(get_distance(&sensor_l) > DISTANCE) add_by_angle(map_angle, false);
  } 
  else  
  {
    current_state = ROTATION_LEFT;
    *map_angle = adduction(*map_angle + 90);
  }
}

void alg_left_hand() 
{
  if (get_distance(&sensor_l) > DISTANCE || get_distance(&sensor_l) == -1) 
  {
    current_state = ROTATION_LEFT;
  } 
  else if (get_distance(&sensor_u) > DISTANCE || get_distance(&sensor_u) == -1) 
  {
    current_state = MOVING;
  } 
  else
  {
    current_state = ROTATION_RIGHT;
  } 
}
