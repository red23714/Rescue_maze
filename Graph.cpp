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

    add_node(x_local, y_local, discovered);
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
    if(angle > 180) angle -= 180;
    if(angle < -180) angle += 180;

    return angle;
}

//Создает набор движений, которые необходимо выполнить, чтобы добраться из текущий вершины в заданную
Vec<enum moves> Graph::get_move(node to, int angle)
{
    Vec<enum moves> moves;
    enum direction dir;
    Vec<node> path;
    int x = graph[current_node].x;
    int y = graph[current_node].y;

    path = find_path(to);

    for (int i = 0; i < path.size(); i++)
    {
        // std::cout << x << ' ' << y << ' ' << angle << " ";
        dir = get_move_dir(x, y, path[i].x, path[i].y, angle);
        // std::cout << dir << "\n";

        switch(dir)
        {
            case FORWARD:
                moves.push_back(MOVE_FORWARD);
                break;
            case RIGHT:
                angle = adduction(angle - 90);
                moves.push_back(ROTATE_RIGHT);
                moves.push_back(MOVE_FORWARD);
                break;
            case LEFT:
                angle = adduction(angle + 90);
                moves.push_back(ROTATE_LEFT);
                moves.push_back(MOVE_FORWARD);
                break;
            case BACKWARD:
                angle = adduction(angle + 180);
                moves.push_back(ROTATE_RIGHT);
                moves.push_back(ROTATE_RIGHT);
                moves.push_back(MOVE_FORWARD);
                break;
            default:
                moves.push_back(NO_MOVE);
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
    if(number != -1) return graph[number].discovered;
    else return false;
}

//Алгоритм Деикстры, который находит кратчайший путь до заданной вершины
Vec<node> Graph::find_path(node to)
{
    int st = graph[current_node].number;
    int fn = to.number;

    Vec<node> exit;

    if(fn != -1)
    {
        Vec<Vec<int>> g;

        for (int i = 0; i < graph.size(); i++)
        {
            g.push_back(Vec<int>());
            for(int j = 0; j < graph.size(); j++)
            {
                if(graph_connection[i][j])
                {
                    g[i].push_back(j);
                }
            }
        }

        Queue q;
        q.push(st);
        Vec<bool> used(graph.size());
        Vec<int> p(graph.size());
        used[st] = true;
        p[st] = -1;
        while (!q.empty()) 
        {
            int v = q.front();
            q.pop();
            for (size_t i = 0; i < g[v].size(); ++i) 
            {
                int to = g[v][i];
                if (!used[to]) 
                {
                    used[to] = true;
                    q.push (to);
                    p[to] = v;
                }
            }
        }

        Vec<int> path;
        int past = fn;
        for (int i = 0; i < graph.size(); i++)
        {
            if(past != -1) 
            {
                path.push_back(past);
                past = p[past];
            }
        }

        // std::reverse(path.begin(), path.end());

        int n = path.size();
        for (int i = 0; i < n; i++)
        {
            exit.push_back(graph[path[n - i - 1]]);
        }
    }

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
    if(get_node(x, y) != -1) return true;
    else return false;
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

//Находит ближайшую вершину, которая не была пройдена
node Graph::get_not_discovered()
{
    int x = graph[current_node].x, y = graph[current_node].y;
    int x_c = 500, y_c = 500, index = 0;
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
