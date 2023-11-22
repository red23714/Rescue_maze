#include "Graph.h"

Graph graph;

void setup()
{  
  Serial.begin(115200);

  // robot.init();
  graph.add_by_angle(0);
  graph.add_by_angle(90);
  graph.add_by_angle(180);
  graph.add_by_angle(90);
  graph.add_by_angle(90, false);
  graph.add_by_angle(90);
  graph.add_by_angle(-90);
  graph.add_by_angle(-90);
  graph.add_by_angle(0);
  graph.add_by_angle(-90);
  graph.add_by_angle(-180);

  node point = graph.get_not_discovered();
  Vec<state> path = graph.get_move(point, 0);

  for (int i = 0; i < path.size(); i++)
  {
    Serial.println(path[i]);
  }

  Serial.println("End");
}

void loop()
{
  // robot.print_dis();
  // robot.print_gyro();
  // robot.print_save();
  // robot.print_enc();
  // robot.print_color();
  // robot.print_current_state();
  // robot.print_map();

  // robot.wait(1);

  // robot.state_machine();
}
