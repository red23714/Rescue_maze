void state_machine()
{
  switch(current_state)
  {
    case(WAIT): 
      alg_right_hand();
      break;
    case(MOVING): 
      if(mov_forward()) current_state = WAIT;
      break;
    case(ROTATION_RIGHT): 
      if(rot_right()) current_state = WAIT;
      break;
    case(ROTATION_LEFT): 
      if(rot_left()) current_state = WAIT;
      break;
  }
}