void alg_right_hand(Map_data* map_data, Distance_data* distance_data, state* current_state) 
{
  if (distance_data->right_dist > DISTANCE || distance_data->right_dist == -1) 
  {
    *current_state = ROTATION_RIGHT;

    if(distance_data->left_dist > DISTANCE) add_by_angle(map_data, false);

    map_data->map_angle = adduction(map_data->map_angle - 90);
  } 
  else if (distance_data->central_dist > DISTANCE || distance_data->central_dist == -1) 
  {
    *current_state = MOVING;

    if(distance_data->left_dist > DISTANCE) add_by_angle(map_data, false);
  } 
  else  
  {
    *current_state = ROTATION_LEFT;
    map_data->map_angle = adduction(map_data->map_angle + 90);
  }
}

void alg_left_hand(state* current_state) 
{
  if (get_distance(&sensor_l) > DISTANCE || get_distance(&sensor_l) == -1) 
  {
    *current_state = ROTATION_LEFT;
  } 
  else if (get_distance(&sensor_u) > DISTANCE || get_distance(&sensor_u) == -1) 
  {
    *current_state = MOVING;
  } 
  else
  {
    *current_state = ROTATION_RIGHT;
  } 
}
