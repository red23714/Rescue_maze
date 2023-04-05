void add_by_angle(int* map_angle, bool discovered = true)
{
  int x_local, y_local;
  x_local = x;
  y_local = y;

  if(!discovered)
  {
    switch (*map_angle) 
    {
      case 0:
        x_local--;
        break;
      case -90:
        y_local++;
        break;
      case 90:
        y_local--;
        break;
      case 180:
        x_local++;
        break;
      case -180:
        x_local++;
        break;
    }

    graph.add_node(x_local, y_local, discovered);
  }
  else 
  {
    switch (*map_angle) 
    {
      case 0:
        y++;
        break;
      case -90:
        x++;
        break;
      case 90:
        x--;
        break;
      case 180:
        y--;
        break;
      case -180:
        y--;
        break;
    }

    graph.add_node(x, y);
  }

  graph.print_graph();

  // Serial.println(*map_angle);
}

void return_to_point(int* map_angle) 
{
  Serial.println("Lets goooo");
  node point = graph.get_not_discovered();
  Vec<enum moves> moves = graph.get_move(point, 0);



  for (int i = 0; i < moves.size(); i++) 
  {
    switch(moves[i])
    {
      case ROTATE_RIGHT:
        // while(!rot_right()) wait(1);
        Serial.println("ROTATE_RIGHT");
        break;
      case ROTATE_LEFT:
        // while(!rot_left()) wait(1);
        Serial.println("ROTATE_LEFT");
        break;
      case MOVE_FORWARD:
        // while(!mov_forward(map_angle)) wait(1);
        Serial.println("MOVE_FORWAARD");
        break;
      default:
        break;
    }
    wait(1000);
  }
}