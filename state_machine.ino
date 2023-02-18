void state_machine()
{
  if(current_state == WAIT) right_hand();
  else if(current_state == MOVING) mov_forward();
  else if(current_state == ROTATION_RIGHT) 
  {
    rot_right();
    if(get_distance(&sensor_u) > DISTANCE)
    {
      current_state = MOVING;
    }
  }
  else if(current_state == ROTATION_LEFT) 
  {
    rot_left();
    if(get_distance(&sensor_u) > DISTANCE)
    {
      current_state = MOVING;
    }
  }

  wait(1);
}