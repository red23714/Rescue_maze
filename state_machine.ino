void state_machine()
{
  if(current_state == WAIT) right_hand();
  else if(current_state == MOVING) mov_forward();
  else if(current_state == ROTATION_RIGHT) rot_right();
  else if(current_state == ROTATION_LEFT) rot_left();

  wait(0);
}