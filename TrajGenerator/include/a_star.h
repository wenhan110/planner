#ifndef PP_ASTAR_H
#define PP_ASTAR_H
#include "PlannerMap.h"
#include <queue>

class Node{
public:
int x_, y_, flag_;
float cost_, g_cost_;
Node* parent_;
Node(int x,int y): x_(x), y_(y){
    flag_ = 0;
    parent_ = NULL;
    cost_ = INFINITY;
    g_cost_ = INFINITY;
}

};

class Cell{
public:
Cell(){}
int row_, col_;
Cell(int row, int col):row_(row), col_(col){}

};


class AStar
{
public:
AStar(PlannerMap *plannermap, uint8_t inaccessible_cost, float heuristic_factor);
virtual ~AStar(){}

enum SearchState{
    NOT_HANDLED,
    OPEN,
    CLOSED
};

void SetMapPtr(PlannerMap* plannermap){
    static_layer_ = plannermap->GetStaticLayer();
    sta_inflation_layer_ = plannermap->GetStaInflationLayer();
}

void Reset();

bool Search(Cell start, Cell goal);

void Search(uint32_t start_row, uint32_t start_col, uint32_t goal_row, uint32_t goal_col);

void VisualizePath();

std::vector<Cell> GetGlobalPath(){
    return path_;
}   

StaticLayer* static_layer_;
InflationLayer* sta_inflation_layer_;


private:
void GetNeighbours(const Node& node, std::vector<Node>& neighbours);
void GetNeighbours(const int &current_index, std::vector<int> &neighbours_index_list) const;
void GetPath();
float CalcHeuristic(Node* node, Node* next);
void CalcMoveCost(const int &current_index, const int &neighbour_index, int &move_cost) const ;
void GetManhattanDistance(const int &index_1, const int & index_2, int &manhattan_distance) const ;

std::vector<int> g_score_;
static std::vector<int> f_score_;
std::vector<int> parent_;
// Matrix g_score_;
// static Matrix f_score_;
// Matrix parent_;

std::vector<AStar::SearchState> state_;
std::vector<Cell> path_;
Matrix cost_;

struct cmp{
    bool operator() (const int &index_left, const int &index_right){
        return AStar::f_score_.at(index_left) > AStar::f_score_.at(index_right);
    }
};

Node* node_goal_;
Node* node_start_;


float heuristic_factor_;
uint8_t inaccessible_cost_;
double weight_start_, weight_goal_, weight_obs_;
uint32_t cell_row_size_, cell_col_size_;
double resolution_;


};

// std::vector<int> AStar::f_score_;





#endif // PP_ASTAR_H