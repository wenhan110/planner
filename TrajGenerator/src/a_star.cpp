#include "a_star.h"


std::vector<int> AStar::f_score_;

AStar::AStar(PlannerMap *plannermap, uint8_t inaccessible_cost, float heuristic_factor):
            inaccessible_cost_(inaccessible_cost), 
            heuristic_factor_(heuristic_factor){
    cell_row_size_ = plannermap->GetStaticLayer()->GetCellSizeRow();
    cell_col_size_ = plannermap->GetStaticLayer()->GetCellSizeCol();
    cost_.resize(cell_row_size_ , cell_col_size_);
    resolution_ = plannermap->GetStaticLayer()->GetResolution();
    SetMapPtr(plannermap);
}


// AStar::~AStar(){
    
// }

void AStar::Reset(){
    g_score_.clear();
    f_score_.clear();
    parent_.clear();
    state_.clear();
    cell_col_size_ = static_layer_->GetCellSizeCol();
    cell_row_size_ = static_layer_->GetCellSizeRow();
    cost_.resize(cell_row_size_ , cell_col_size_);
    cost_ = sta_inflation_layer_->GetCostmap();
    g_score_.resize(cell_row_size_ * cell_col_size_, std::numeric_limits<int>::max());
    f_score_.resize(cell_row_size_ * cell_col_size_, std::numeric_limits<int>::max());
    parent_.resize(cell_row_size_ * cell_col_size_, std::numeric_limits<int>::max());
    state_.resize(cell_row_size_ * cell_col_size_, SearchState::NOT_HANDLED);
}


// bool AStar::Search(Cell start, Cell goal){
//     node_goal_->x_ = goal.x_;
//     node_goal_->y_ = goal.y_;
//     struct cmp{
//         bool operator()(const Node* left, Node* right){
//             return left->cost_ > right->cost_;  //from big value to small value
//         }
//     };
//     std::priority_queue <Node*, std::vector<Node*>, cmp > open_list;
    

// }

void AStar::Search(uint32_t start_row, uint32_t start_col, uint32_t goal_row, uint32_t goal_col){
    // g_score_.clear();
    // f_score_.clear();
    // parent_.clear();
    // state_.clear();
    // cell_col_size_ = static_layer_->GetCellSizeCol();
    // cell_row_size_ = static_layer_->GetCellSizeRow();
    // cost_.resize(cell_row_size_ * cell_col_size_);
    // cost_ = sta_inflation_layer_->GetCostmap();
    // g_score_.resize(cell_row_size_ * cell_col_size_, std::numeric_limits<int>::max());
    // f_score_.resize(cell_row_size_ * cell_col_size_, std::numeric_limits<int>::max());
    // parent_.resize(cell_row_size_ * cell_col_size_, std::numeric_limits<int>::max());
    // state_.resize(cell_row_size_ * cell_col_size_, SearchState::NOT_HANDLED);
    Reset();
    std::priority_queue<int, std::vector<int>, cmp> open_list;
    g_score_.at(start_row * cell_col_size_ + start_col) = 0;
    open_list.push(start_row * cell_col_size_ + start_col);

    std::vector<int> neighbours_index_list;

    int current_index, move_cost, h_score, count = 0;

    while(!open_list.empty()){
        current_index = open_list.top();
        open_list.pop();
        state_.at(current_index) = SearchState::CLOSED;
        if(current_index == goal_row * cell_col_size_ + goal_col){
            cout<<"Search takes "<<count<<" steps"<<endl;
            break;
        }

        GetNeighbours(current_index, neighbours_index_list);

        for(auto neighbour_index : neighbours_index_list){
            if(cost_(neighbour_index / cell_col_size_, neighbour_index % cell_col_size_) >= inaccessible_cost_ || state_.at(neighbour_index) == SearchState::CLOSED){
                continue;
            }
            CalcMoveCost(current_index, neighbour_index, move_cost);

            if(g_score_.at(neighbour_index) > g_score_.at(current_index) + move_cost){
                g_score_.at(neighbour_index) = g_score_.at(current_index) + move_cost;
                parent_.at(neighbour_index) = current_index;
                h_score = heuristic_factor_ * 10 * (abs((int)(neighbour_index / cell_col_size_ - goal_row)) + abs((int)(neighbour_index % cell_col_size_ - goal_col)));
                f_score_.at(neighbour_index) = g_score_.at(neighbour_index) + h_score;
                if(state_.at(neighbour_index) == SearchState::NOT_HANDLED){
                    GetManhattanDistance(neighbour_index, goal_row * cell_col_size_ + goal_col, h_score);
                    f_score_.at(neighbour_index) = g_score_.at(neighbour_index) + h_score;
                    state_.at(neighbour_index) = SearchState::OPEN;
                    open_list.push(neighbour_index);
                }
            }
        }
        count++;
    }
    path_.clear();
    uint32_t iter_index = current_index;
    uint32_t iter_row = iter_index / cell_col_size_;
    uint32_t iter_col = iter_index % cell_col_size_;
    Cell iter_cell(iter_row, iter_col);
    path_.push_back(iter_cell);
    while(iter_index != start_row * cell_col_size_ + start_col){
        iter_index = parent_.at(iter_index);
        iter_row = iter_index / cell_col_size_;
        iter_col = iter_index % cell_col_size_;
        Cell iter_cell(iter_row, iter_col);
        path_.push_back(iter_cell);
    }
    std::reverse(path_.begin(), path_.end());
    cout<<"Path size: "<<path_.size()<<endl;
}

void AStar::VisualizePath(){
    for(uint32_t i = 0; i < path_.size(); i++){
        uint32_t row = path_[i].row_;
        uint32_t col = path_[i].col_;
        sta_inflation_layer_->SetCost(row, col, i);
    }
}



void AStar::CalcMoveCost(const int &current_index, const int &neighbour_index, int &move_cost) const {
    if(current_index / cell_col_size_ == neighbour_index / cell_col_size_ || current_index % cell_col_size_ == neighbour_index % cell_col_size_){
        move_cost = 10;
    }
    else if(abs(neighbour_index - current_index) == cell_col_size_ - 1 || abs(neighbour_index - current_index) == cell_col_size_ + 1){
        move_cost = 14;
    }
    else{
        cout<<"Error: AStar::CalcMoveCost()"<<endl;
    }
}

void AStar::GetManhattanDistance(const int &index_1, const int & index_2, int &manhattan_distance) const {
    manhattan_distance = heuristic_factor_ * 10 * abs((int)(index_1 / cell_col_size_ - index_2 / cell_col_size_)) + abs((int)(index_1 % cell_col_size_ - index_2 % cell_col_size_));
}


void AStar::GetNeighbours(const int &current_index, std::vector<int> &neighbours_index) const{
    neighbours_index.clear();
    //up
    if(current_index - cell_col_size_ >= 0){
        neighbours_index.push_back(current_index - cell_col_size_);
    }
    //left_up
    if(current_index - cell_col_size_ - 1 >= 0 && current_index % cell_col_size_ != 0){
        neighbours_index.push_back(current_index - cell_col_size_ - 1);
    }
    // left
    if(current_index - 1 >= 0 && current_index % cell_col_size_ != 0){
        neighbours_index.push_back(current_index - 1);
    }
    // left_down
    if(current_index + cell_col_size_ - 1 < cell_row_size_ * cell_col_size_ && current_index % cell_col_size_ != 0){
        neighbours_index.push_back(current_index + cell_col_size_ - 1);
    }
    //down
    if(current_index + cell_col_size_ < cell_row_size_ * cell_col_size_){
        neighbours_index.push_back(current_index + cell_col_size_);
    }
    //right_down
    if(current_index + cell_col_size_ + 1 < cell_row_size_ * cell_col_size_ && (current_index + 1) % cell_col_size_ != 0){
        neighbours_index.push_back(current_index + cell_col_size_ + 1);
    }
    //right 
    if(current_index + 1 < cell_row_size_ * cell_col_size_ && (current_index + 1) % cell_col_size_ != 0){
        neighbours_index.push_back(current_index + 1);
    }
    //right_up
    if(current_index - cell_col_size_ + 1 >= 0 && (current_index + 1) % cell_col_size_ != 0){
        neighbours_index.push_back(current_index - cell_col_size_ + 1);
    }
}