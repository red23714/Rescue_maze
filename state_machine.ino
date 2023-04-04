void state_machine(int* map_angle)
{
  switch(current_state)
  {
    case(WAIT): 
      alg_right_hand();
      motor_stop();
      wait(1000);
      break;
    case(MOVING): 
      if(mov_forward(map_angle)) current_state = WAIT;
      break;
    case(ROTATION_RIGHT): 
      if(rot_right(map_angle)) current_state = WAIT;
      break;
    case(ROTATION_LEFT): 
      if(rot_left(map_angle)) current_state = WAIT;
      break;
  }
}