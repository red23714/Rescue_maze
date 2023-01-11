void add_by_angle() 
{
  switch (angle) 
  {
    case 0:
      y--;
      break;
    case 90:
      x--;
      break;
    case 180:
      y++;
      break;
    case 270:
      x++;
      break;
  }

  graph.add_node(x, y);
}