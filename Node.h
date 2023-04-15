enum cell_type{
    USUAL = 0,
    WATER,
    HOLE,
    CHECKPOINT
};

struct node
{
    int number;

    int x;
    int y;
    bool discovered = false;
    enum cell_type type = USUAL;

    bool operator ==(node a)
    {
        return this->number == a.number && this->x == a.x && this->y == a.y;
    }

    // bool add_connection(node *node)
    // {
    //     if(n >= 3) return 0;
    //     if(abs(node->x-x) > 1 || abs(node->y-y) > 1) return 0;
       
    //     nodes[n] = node;
    //     std::cout << nodes[n] << "\n";
    //     n++;
    //     return 1;
    // }

    int print_node()
    {
        Serial.print("number: ");
        Serial.print(number);
        Serial.print(" ");
        Serial.print("x: ");
        Serial.print(x);
        Serial.print(" ");
        Serial.print("y: ");
        Serial.print(y);
        Serial.print(" ");
        Serial.print("discovered: ");
        Serial.println(discovered);
    }
    
private: 
    node *nodes[4];
    int n = 0;
};

// inline std::ostream& operator <<(std::ostream& out, node node_out)
// {
//     out << "number:" << node_out.number << std::endl << "x:" << node_out.x << " " <<
//         " " << "y:" << node_out.y << std::endl << "discovered:" << node_out.discovered << std::endl;
    
//     return out;
// }