void add_by_angle(Map_data* map_data, bool discovered = true)
{
  int x_local, y_local;
  x_local = map_data->x;
  y_local = map_data->y;

  if(!discovered)
  {
    switch (map_data->map_angle) 
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
    switch (map_data->map_angle) 
    {
      case 0:
        map_data->y++;
        break;
      case -90:
        map_data->x++;
        break;
      case 90:
        map_data->x--;
        break;
      case 180:
        map_data->y--;
        break;
      case -180:
        map_data->y--;
        break;
    }

    graph.add_node(map_data->x, map_data->y);
  }

  // graph.print_graph();

  // Serial.println(*map_angle);
}

void return_to_point(Map_data map_data) 
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
  }
}