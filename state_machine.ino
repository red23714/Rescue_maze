void state_machine(Map_data* map_data, Package_data* package_data, Distance_data* distance_data, state* current_state)
{
  switch(*current_state)
  {
    case WAIT: 
      alg_right_hand(map_data, distance_data, current_state);

      motor_stop();
      wait(100, distance_data);
      break;
    case MOVING: 
      if(mov_forward(map_data, distance_data)) *current_state = WAIT;
      break;
    case ROTATION_RIGHT: 
      if(rot_right()) 
      {
        if(distance_data->central_dist > DISTANCE || distance_data->central_dist == -1) *current_state = MOVING;
        else *current_state = WAIT;
      }
      break;
    case ROTATION_LEFT: 
      if(rot_left()) 
      {
        if(distance_data->central_dist > DISTANCE || distance_data->central_dist == -1) *current_state = MOVING;
        else *current_state = WAIT;
      }
      break;
    case GIVING:
      giving(package_data);

      break;
  }
}


