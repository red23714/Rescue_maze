void return_to_point() 
{
  node point = graph.get_not_been();
  Vec<byte> moves = graph.get_move(point);

  for (int i = 0; i < moves.size(); i++) 
  {
    switch (moves[i]) 
    {
      case 0:
        rot_right();
        mov_forward();
        break;
      case 1:
        rot_left();
        mov_forward();
        break;
      case 2:
        mov_forward();
        break;
      case 3:
        mov_back();
        break;
    }
  }
}