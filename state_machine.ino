void state_machine(int* map_angle)
{
  switch(current_state)
  {
    case(WAIT): 
      alg_right_hand(map_angle);

      motor_stop();
      wait(1000);
      break;
    case(MOVING): 
      if(mov_forward(map_angle)) current_state = WAIT;
      break;
    case(ROTATION_RIGHT): 
      if(rot_right()) 
      {
        if(get_distance(&sensor_u) > DISTANCE || get_distance(&sensor_u) == -1) current_state = MOVING;
        else current_state = WAIT;
      }
      break;
    case(ROTATION_LEFT): 
      if(rot_left()) 
      {
        if(get_distance(&sensor_u) > DISTANCE || get_distance(&sensor_u) == -1) current_state = MOVING;
        else current_state = WAIT;
      }
      break;
  }
}


