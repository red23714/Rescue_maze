#include "Graph.h"

Graph::Graph(){
    node start_node;
    start_node.number = 0;
    start_node.x = 0;
    start_node.y = 0;
    start_node.discovered = true;

    graph.push_back(start_node);

    current_node = 0;
}

//По исходному углу вычисляет с какими координатами нужно добавить вершину в граф
void Graph::add_by_angle(int map_angle, bool discovered = true)
{
  int x_local, y_local;
  x_local = graph[current_node].x;
  y_local = graph[current_node].y;

  if(!discovered)
  {
    switch (map_angle) 
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

    add_node(x_local, y_local, false);
  }
  else 
  {
    switch (map_angle) 
    {
      case 0:
        y_local++;
        break;
      case -90:
        x_local++;
        break;
      case 90:
        x_local--;
        break;
      case 180:
        y_local--;
        break;
      case -180:
        y_local--;
        break;
    }

    add_node(x_local, y_local);
  }
}

//Добавляет в граф вершину с заданными координатами и флагом, который показывает не является ли точка развилкой
void Graph::add_node(int x, int y, bool is_current)
{  
    int index;
    index = get_node(x, y);

    if(index == -1)
    {
        node temp_node;

        temp_node.number = graph.size();
        temp_node.x = x;
        temp_node.y = y;

        graph.push_back(temp_node);

        make_connection(temp_node.number, graph[current_node].number);
        // graph[graph.size()].add_connection(&graph[current_node]);
        // graph[current_node].add_connection(&graph[graph.size()]);

        if(is_current) 
        {
            graph[graph.size() - 1].discovered = true;
            current_node = graph.size() - 1;
        }
    }
    else 
    {
        if(graph_connection[graph[index].number][graph[current_node].number] == false)
        {
            make_connection(graph[index].number, graph[current_node].number);
        }

        if(is_current) 
        {
            if(!graph[index].discovered) graph[index].discovered = true;
            current_node = index;
        }
    }
}

//Помечает соединения между вершинами графа в таблице смежности
void Graph::make_connection(int n1, int n2)
{
    graph_connection[n1][n2] = true;
    graph_connection[n2][n1] = true;
}

//По координатам двух вершин возвращает направление в котором находится вторая вершина
enum direction Graph::get_move_dir(int x1, int y1, int x2, int y2, int angle)
{
    int xs = x2 - x1;
    int ys = y2 - y1;

    int xr = xs * cos(angle * M_PI / 180) + ys * sin(angle * M_PI / 180);
    int yr = -xs * sin(angle * M_PI / 180) + ys * cos(angle * M_PI / 180);
    
    if(xr == 0 && yr == 0) return NONE;

    if(yr == 0)
    {
        if(xr > 0) return RIGHT;
        else return LEFT;
    }
    else
    {
        if(yr > 0) return FORWARD;
        else return BACKWARD;
    }
}

//Ограничивает значения исходного угла в диапозоне -360; 360
int Graph::adduction(int angle)
{
    while (angle > 180) angle -= 180;
    while (angle < -180) angle += 180;

    return angle;
}

//Создает набор движений, которые необходимо выполнить, чтобы добраться из текущий вершины в заданную
Vec<state> Graph::get_move(node to, int angle)
{
    Vec<state> moves;
    enum direction dir;
    Vec<node> path;
    int x = graph[current_node].x;
    int y = graph[current_node].y;

    path = find_path(to.number);

    for (int i = 0; i < path.size(); i++)
    {
        dir = get_move_dir(x, y, path[i].x, path[i].y, angle);

        switch(dir)
        {
            case FORWARD:
                moves.push_back(state::MOVING);
                break;
            case RIGHT:
                angle = adduction(angle - 90);
                moves.push_back(state::ROTATION_RIGHT);
                moves.push_back(state::MOVING);
                break;
            case LEFT:
                angle = adduction(angle + 90);
                moves.push_back(state::ROTATION_LEFT);
                moves.push_back(state::MOVING);
                break;
            case BACKWARD:
                angle = adduction(angle + 180);
                moves.push_back(state::ROTATION_RIGHT);
                moves.push_back(state::ROTATION_RIGHT);
                moves.push_back(state::MOVING);
                break;
            default:
                moves.push_back(state::WAIT);
                break;
        }

        x = path[i].x;
        y = path[i].y;
    }
    return moves;
}

//Проверяет существует ли вершина и проезжал ли робот эту клетку
bool Graph::is_discovered(int x, int y)
{
    int number = get_node(x, y);
    if (number != -1) return graph[number].discovered;
    else return false;
}

//Алгоритм Деикстры, который находит кратчайший путь до заданной вершины
Vec<node> Graph::find_path(int end)
{
    int start = current_node;
    int n = graph.size();
    int maximum = 1000;

    // Массив для хранения расстояний от начальной точки
    int *dist = new int[n];

    // Массив для отслеживания посещенных вершин
    bool *visited = new bool[n];

    // Инициализация расстояния от начальной точки до самой себя
    for (int i = 0; i < n; ++i)
    {
        dist[i] = maximum;
        visited[i] = false;
    }
    dist[start] = 0;

    // Цикл по всем вершинам графа
    for (int count = 0; count < n - 1; ++count)
    {
        // Находим вершину с минимальным расстоянием, которую еще не посетили
        int minDist = maximum, minIndex;
        for (int v = 0; v < n; ++v)
        {
            if (!visited[v] && dist[v] <= minDist)
            {
                minDist = dist[v];
                minIndex = v;
            }
        }

        // Помечаем выбранную вершину как посещенную
        visited[minIndex] = true;

        // Обновляем расстояния до соседних вершин через выбранную вершину
        for (int v = 0; v < n; ++v)
        {
            if (!visited[v] && graph_connection[minIndex][v] && dist[minIndex] != maximum && dist[minIndex] + graph_connection[minIndex][v] < dist[v])
            {
                dist[v] = dist[minIndex] + graph_connection[minIndex][v];
            }
        }
    }

    Vec<int> path;

    // Выводим кратчайший путь от начальной точки к конечной
    int current = end;
    while (current != start)
    {
        for (int v = 0; v < n; ++v)
        {
            if (graph_connection[v][current] == 1 && dist[current] == dist[v] + 1)
            {
                path.push_back(v);
                Serial.println(v);
                current = v;
                break;
            }
        }
    }

    Vec<node> exit;
    for (int i = path.size() - 2; i >= 0; i--)
    {
        exit.push_back(graph[path[i]]);
    }
    

    delete[] dist;
    delete[] visited;

    return exit;
}

// std::ostream& operator <<(std::ostream& out, Graph graph_in)
// {
//     for (int i = 0; i < graph_in.graph.size(); i++)
//     {
//         for (int j = 0; j < graph_in.graph.size(); j++)
//         {
//             out << graph_in.graph_connection[i][j];
//         }
//         out << std::endl;
//     }

//     return out;
// }


//Вывод графа в консоль
int Graph::print_graph()
{
    for (int i = 0; i < graph.size(); i++)
    {
        graph[i].print_node();
    }
    Serial.println("------------");
}

//Проверяет существует ли вершина
bool Graph::get_node_exist(int x, int y)
{
    return get_node(x, y) != -1;
}

//Возвращает вершину из графа по заданным координатам
int Graph::get_node(int x, int y)
{
	// int c = 0;
	// int l = -1;
	// int r = graph.size();
	// while(r - l > 1)
	// {
	// 	c = (l + r)/2;
	// 	if(graph[c].x >= x && graph[c].y >= y)
	// 	{
	// 		r = c;
	// 	}
	// 	else
	// 	{
	// 		l = c;
	// 	}
	// }

	// if(graph[r].x == x && graph[r].y == y) return r;
    // else return -1;

    for (int i = 0; i < graph.size(); i++)
    {
        if(graph[i].x == x && graph[i].y == y) return i;
    }
    
    return -1;
}

void Graph::set_to_checkpoint()
{
    for(int i = graph.size() - 1; i >= 0; i--)
    {
        if(graph[i].type == cell_type::CHECKPOINT || i == 0) 
        {
            current_node = i;
            break;
        }
    }
}

//Находит ближайшую вершину, которая не была пройдена
node Graph::get_not_discovered()
{
    int x = graph[current_node].x, y = graph[current_node].y;
    int x_c = 100, y_c = 100, index = 0;
    int x_vec_1, y_vec_1, x_vec_2, y_vec_2, len_1, len_2;
    for(int i = 0; i < graph.size(); i++)
    {
        x_vec_1 = abs(x - x_c);
        y_vec_1 = abs(y - y_c);
        x_vec_2 = abs(x - graph[i].x);
        y_vec_2 = abs(y - graph[i].y);
        len_1 = sqrt(x_vec_1*x_vec_1 + y_vec_1*y_vec_1);
        len_2 = sqrt(x_vec_2*x_vec_2 + y_vec_2*y_vec_2);

        if(graph[i].discovered == false && len_1 > len_2)
        {
            x_c = graph[i].x;
            y_c = graph[i].y;
            index = i;
        }
    }

    return graph[index];
}

//Возращает граф в виде массива вершин
Vec<node> Graph::get_graph()
{
    return graph;
}

//Возвращает текущую вершину в которой мы находимся
node Graph::get_current_node()
{
    return graph[current_node];
}

int Graph::get_graph_length()
{
    return graph.size();
}

bool Graph::is_start_node()
{
    return current_node == 0;
}

void Graph::set_current_node(cell_type type = cell_type::USUAL, letter letter_cell = letter::N)
{
    graph[current_node].type = type;
    graph[current_node].letter_cell = letter_cell;
}
