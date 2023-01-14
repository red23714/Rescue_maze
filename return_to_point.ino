void return_to_point() 
{
  node point = graph.get_not_discovered();
  Vec<enum moves> moves = graph.get_move(point, 0);

  for (int i = 0; i < moves.size(); i++) 
  {
    switch(moves[i])
    {
      case ROTATE_RIGHT:
        rot_right();
        break;
      case ROTATE_LEFT:
        rot_left();
        break;
      case MOVE_FORWARD:
        mov_forward();
        break;
      default:
        break;
    }
  }
}