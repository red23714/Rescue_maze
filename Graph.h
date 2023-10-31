#pragma once

#include "Vec.h"
#include "Queue.h"
#include <math.h>
#include "Node.h"
#include "Direction.h"
#include "Moves.h"

class Graph
{
public:
    Graph();

    void add_by_angle(int, bool = true);
    void add_node(int, int, bool = true);
    // void make_current_node();
    
    node get_not_discovered();
    bool is_discovered(int, int);
    bool get_node_exist(int, int);
    Vec<node> get_graph();
    node get_current_node();
    int get_graph_length();

    void set_current_node(cell_type = cell_type::USUAL, letter = letter::N);

    int print_graph();

    Vec<enum moves> get_move(node to, int);
    Vec<node> find_path(node);
    // 0 - right
    // 1 - left
    // 2 - forward
    // 3 - back

    // friend std::ostream& operator <<(std::ostream& out, Graph graph_in);

private:
    Vec<node> graph;
    int count_of_nodes = 0;
    bool graph_connection[25][25]; 
    int current_node;

    int get_node(int, int);

    void make_connection(int, int);

    int adduction(int);

    enum direction get_move_dir(int, int, int, int, int);
};
