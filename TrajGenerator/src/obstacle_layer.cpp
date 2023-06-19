#include "obstacle_layer.h"

ObstacleLayer::ObstacleLayer(StaticLayer* static_layer, const Pose2dT& cur_pose) : static_layer_(static_layer) {
  Initialize(cur_pose);
}

void ObstacleLayer::Initialize(const Pose2dT& pose) {
  resolution_ = OBSTACLE_MAP_RESOLUTION;
  size_row_ = OBSTACLE_MAP_ROWS;
  size_col_ = OBSTACLE_MAP_COLS;
  origin_x_ = pose.x - size_col_ / 2 * resolution_;
  origin_y_ = pose.y + size_row_ / 2 * resolution_;
  data_.resize(size_row_, size_col_);
  label_.resize(size_row_, size_col_);
  fold_ = static_layer_->GetResolution() / resolution_;
  cur_pose_ = pose;
  last_pose_ = pose;
  double upper_left_x = pose.x - size_col_ / 2 * resolution_;
  double upper_left_y = pose.y + size_row_ / 2 * resolution_;
  double lower_right_x = pose.x + size_col_ / 2 * resolution_;
  double lower_right_y = pose.y - size_row_ / 2 * resolution_;
  LoadStaticInfo(upper_left_x, upper_left_y, lower_right_x, lower_right_y);
}

void ObstacleLayer::Reset(const Pose2dT& pose){
  double upper_left_x = pose.x - size_col_ / 2 * resolution_;
  double upper_left_y = pose.y + size_row_ / 2 * resolution_;
  double lower_right_x = pose.x + size_col_ / 2 * resolution_;
  double lower_right_y = pose.y - size_row_ / 2 * resolution_;
  LoadStaticInfo(upper_left_x, upper_left_y, lower_right_x, lower_right_y);
}

uint8_t ObstacleLayer::Getfold() { return fold_; }

// Input: Position of the bound in obstacle_layer. Get the index in static_layer and set the cost.
// void ObstacleLayer::LoadStaticInfo(double upper_left_x, double upper_left_y, double lower_right_x, double
// lower_right_y){
//     int start_cell_row_obs = static_cast<int>((-upper_left_y + origin_y_) / resolution_);
//     int start_cell_col_obs = static_cast<int>((upper_left_x - origin_x_) / resolution_);
//     int end_cell_col_obs = static_cast<int>((lower_right_x - origin_x_) / resolution_);
//     int end_cell_row_obs = static_cast<int>((-lower_right_y + origin_y_) / resolution_);

//     // int start_cell_col_sta = static_cast<int>((upper_left_x - static_layer_->GetOriginX()) /
//     static_layer_->GetResolution());
//     // int start_cell_row_sta = static_cast<int>((-upper_left_y + static_layer_->GetOriginY()) /
//     static_layer_->GetResolution());
//     // int end_cell_col_sta = static_cast<int>((lower_right_x - static_layer_->GetOriginX()) /
//     static_layer_->GetResolution());
//     // int end_cell_row_sta = static_cast<int>((-lower_right_y + static_layer_->GetOriginY()) /
//     static_layer_->GetResolution()); int start_cell_col_sta = static_cast<int>((upper_left_x -
//     static_layer_->GetOriginX()) / 0.1); int start_cell_row_sta = static_cast<int>((-upper_left_y +
//     static_layer_->GetOriginY()) / 0.1); int end_cell_col_sta = static_cast<int>((lower_right_x -
//     static_layer_->GetOriginX()) / 0.1); int end_cell_row_sta = static_cast<int>((-lower_right_y +
//     static_layer_->GetOriginY()) / 0.1); int size_row_sta = static_layer_->GetCellSizeRow(); int size_col_sta =
//     static_layer_->GetCellSizeCol(); for(int i = start_cell_row_obs; i < end_cell_row_obs; i++){
//         for(int j = start_cell_col_obs; j < end_cell_col_obs; j++){
//             int static_i = (i - start_cell_col_obs) / fold_ + start_cell_row_sta;
//             int static_j = (j - start_cell_row_obs) / fold_ + start_cell_col_sta;
//             if(static_i >= size_row_sta || static_j >= size_col_sta || static_i < 0 || static_j < 0){
//                 static_layer_->needResizeGlobalMap_ = true;
//                 data_(i, j) = 0;
//                 label_(i, j) = 0;
//             }
//             else{
//                 data_(i, j) = static_layer_->GetData(static_i, static_j);
//                 label_(i, j) = static_layer_->GetLabel(static_i, static_j);
//             }
//         }
//     }
//     uint8_t resize_direction_;

//     if(start_cell_row_sta < 0 && end_cell_col_sta >= size_col_sta){  // Right Up
//         resize_direction_ = RESIZING_RIGHTUP;
//     }
//     else if(start_cell_row_sta < 0 && start_cell_col_sta < 0){  // Left Up
//         resize_direction_ = RESIZING_LEFTUP;
//     }
//     else if(end_cell_row_sta >= size_row_sta && start_cell_col_sta < 0){  // Left Down
//         resize_direction_ = RESIZING_LEFTDOWN;
//     }
//     else if(end_cell_row_sta >= size_row_sta && end_cell_col_sta >= size_col_sta){  // Right Down
//         resize_direction_ = RESIZING_RIGHTDOWN;
//     }
//     else if(start_cell_row_sta < 0 && start_cell_col_sta >= 0 && end_cell_col_sta < size_col_sta){  // Up
//         resize_direction_ = RESIZING_UP;
//     }
//     else if(start_cell_col_sta < 0 && start_cell_row_sta >= 0 && end_cell_row_sta < size_col_sta){  // Left
//         resize_direction_ = RESIZING_LEFT;
//     }
//     else if(end_cell_row_sta >= size_row_sta && start_cell_col_sta >= 0 && end_cell_col_sta < size_col_sta){  // Down
//         resize_direction_ = RESIZING_DOWN;
//     }
//     else if(end_cell_col_sta >= size_col_sta && start_cell_row_sta >= 0 && end_cell_row_sta < size_row_sta){  //
//     Right
//         resize_direction_ = RESIZING_RIGHT;
//     }
//     else{
//         static_layer_->needResizeGlobalMap_ = false;
//     }

//     static_layer_->ResizeMap(resize_direction_);
// }

void ObstacleLayer::LoadStaticInfo(
    double upper_left_x, double upper_left_y, double lower_right_x, double lower_right_y) {
  int start_cell_row_obs = round((-upper_left_y + origin_y_) / resolution_);  // 0
  int start_cell_col_obs = round((upper_left_x - origin_x_) / resolution_);   // 0
  int end_cell_col_obs = round((lower_right_x - origin_x_) / resolution_);    // 50
  int end_cell_row_obs = round((-lower_right_y + origin_y_) / resolution_);   // 50
  double dltx = upper_left_x - static_layer_->GetOriginX();
  double dlty = -upper_left_y + static_layer_->GetOriginY();
  double resolution = static_layer_->GetResolution();

  int start_cell_col_sta = round((dltx / resolution));                                          // 25
  int start_cell_row_sta = round((dlty / resolution));                                          // 25s
  int end_cell_col_sta = round(((lower_right_x - static_layer_->GetOriginX()) / resolution));   // 35
  int end_cell_row_sta = round(((-lower_right_y + static_layer_->GetOriginY()) / resolution));  // 35
  int size_row_sta = static_layer_->GetCellSizeRow();
  int size_col_sta = static_layer_->GetCellSizeCol();

  for (int i = start_cell_row_obs; i < end_cell_row_obs; i++) {    // 0~49
    for (int j = start_cell_col_obs; j < end_cell_col_obs; j++) {  // 0~49
      int static_i = (i - start_cell_row_obs) / fold_ + start_cell_row_sta;
      int static_j = (j - start_cell_col_obs) / fold_ + start_cell_col_sta;
      // if(i == 9){
      //     cout<<"ERROR_CHECK: static index is :" << static_i << "  " << static_j << endl;
      // }

      if (static_i >= size_row_sta || static_j >= size_col_sta || static_i < 0 || static_j < 0) {
        static_layer_->needResizeGlobalMap_ = true;
        data_(i, j) = 0;
        label_(i, j) = 0;
      } else {
        data_(i, j) = static_layer_->GetData(static_i, static_j);
        label_(i, j) = static_layer_->GetLabel(static_i, static_j);
      }
    }
  }
  uint8_t resize_direction_;

  if (start_cell_row_sta < 0 && end_cell_col_sta >= size_col_sta) {  // Right Up
    resize_direction_ = RESIZING_RIGHTUP;
  } else if (start_cell_row_sta < 0 && start_cell_col_sta < 0) {  // Left Up
    resize_direction_ = RESIZING_LEFTUP;
  } else if (end_cell_row_sta >= size_row_sta && start_cell_col_sta < 0) {  // Left Down
    resize_direction_ = RESIZING_LEFTDOWN;
  } else if (end_cell_row_sta >= size_row_sta && end_cell_col_sta >= size_col_sta) {  // Right Down
    resize_direction_ = RESIZING_RIGHTDOWN;
  } else if (start_cell_row_sta < 0 && start_cell_col_sta >= 0 && end_cell_col_sta < size_col_sta) {  // Up
    resize_direction_ = RESIZING_UP;
  } else if (start_cell_col_sta < 0 && start_cell_row_sta >= 0 && end_cell_row_sta < size_col_sta) {  // Left
    resize_direction_ = RESIZING_LEFT;
  } else if (end_cell_row_sta >= size_row_sta && start_cell_col_sta >= 0 && end_cell_col_sta < size_col_sta) {  // Down
    resize_direction_ = RESIZING_DOWN;
  } else if (end_cell_col_sta >= size_col_sta && start_cell_row_sta >= 0 && end_cell_row_sta < size_row_sta) {  // Right
    resize_direction_ = RESIZING_RIGHT;
  } else {
    static_layer_->needResizeGlobalMap_ = false;
  }

  static_layer_->ResizeMap(resize_direction_);
}

void ObstacleLayer::UpdateObstacle(const Pose2dT& pose) {
  // UpdateObservation(cloud);
  double robot_x = pose.x;
  double robot_y = pose.y;
  double new_origin_x = robot_x - data_.cols() / 2 * resolution_;
  double new_origin_y = robot_y + data_.rows() / 2 * resolution_;
    //   for(int i = 45; i < 50; i++){
    //     for(int j = 10; j < 15; j++){
    //         data_(i, j) = 40;
    //         label_(i, j) = 40;
    //     }
    // }
    
  UpdateOrigin(new_origin_x, new_origin_y);
  // if (cloud_.size <= 0) {
  //   printf("No cloud data!\n");
  // } else {
  //   for (int i = 0; i < cloud_.size; i++) {
  //     Point2dT pt(cloud_.points[i].x, cloud_.points[i].y);
  //     pt = pose * pt;  // get the world coordinate of cloud
  //     int cell_col = static_cast<uint32_t>((pt.x - origin_x_) / resolution_);
  //     int cell_row = static_cast<uint32_t>((-pt.y + origin_y_) / resolution_);
  //     if (cell_row >= 0 && cell_row < data_.rows() && cell_col >= 0 && cell_col < data_.cols()) {
  //       if (cloud_.points[i].z < 0.4) {
  //         data_(cell_row, cell_col) -= 25;
  //         if (data_(cell_row, cell_col) < 0) {
  //           data_(cell_row, cell_col) = 0;
  //         }
  //       } else if (cloud_.points[i].z > 0.6) {
  //         data_(cell_row, cell_col) += 25;
  //         if (data_(cell_row, cell_col) > 255) {
  //           data_(cell_row, cell_col) = 255;
  //         }
  //       }
  //     }
  //   }
  // }
}

// void ObstacleLayer::UpdateOrigin(double new_origin_x, double new_origin_y){
//     int dlt_row, dlt_col;
//     dlt_row = int((new_origin_y - origin_y_) / resolution_);
//     dlt_col = int((new_origin_x - origin_x_) / resolution_);
//     double new_grid_ox, new_grid_oy;
//     new_grid_ox = origin_x_ + dlt_col * resolution_;
//     new_grid_oy = origin_y_ + dlt_row * resolution_;
//     // copy region
//     int start_cell_row_old, start_cell_col_old, end_cell_row_old, end_cell_col_old;
//     int start_cell_row_new, start_cell_col_new, end_cell_row_new, end_cell_col_new;
//     int size_row = size_row_;
//     int size_col = size_col_;
//     start_cell_col_old = std::min(std::max(dlt_col, 0), size_col);
//     start_cell_row_old = std::min(std::max(-dlt_row, 0), size_row);
//     end_cell_col_old = std::min(std::max(dlt_col + size_col, 0), size_col);
//     end_cell_row_old = std::min(std::max(-dlt_row + size_row, 0), size_row);
//     start_cell_col_new = std::min(std::max(-dlt_col, 0), size_col);
//     start_cell_row_new = std::min(std::max(dlt_row, 0), size_row);
//     end_cell_col_new = std::min(std::max(-dlt_col + size_col, 0), size_col);
//     end_cell_row_new = std::min(std::max(dlt_row + size_row, 0), size_row);
//     unsigned int cell_size_row = end_cell_row_old - start_cell_row_old;
//     unsigned int cell_size_col = end_cell_col_old - start_cell_col_old;

//     Matrix new_data_(size_row, size_col);
//     Matrix new_label_(size_row, size_col);

//     new_data_.block(start_cell_row_new, start_cell_col_new, cell_size_row, cell_size_col) =
//     data_.block(start_cell_row_old, start_cell_col_old, cell_size_row, cell_size_col);
//     new_label_.block(start_cell_row_new, start_cell_col_new, cell_size_row, cell_size_col) =
//     label_.block(start_cell_row_old, start_cell_col_old, cell_size_row, cell_size_col);

//     Point2dT write_old_bottom_1, write_old_upper_1, write_old_bottom_2, write_old_upper_2;
//     Point2dT load_static_bottom_1, load_static_upper_1, load_static_bottom_2, load_static_upper_2;
//     if(rolling_direction_ == 1){
//         write_old_bottom_1.setPoint(origin_x_,origin_y_);
//         write_old_upper_1.setPoint(new_grid_ox, size_row_ * resolution_ + origin_y_);
//         write_old_bottom_2.setPoint(new_grid_ox, origin_y_);
//         write_old_upper_2.setPoint(origin_x_ + size_col_ * resolution_, new_grid_oy);
//         load_static_bottom_1.setPoint(new_grid_ox, size_row_ * resolution_ + origin_y_);
//         load_static_upper_1.setPoint(origin_x_ + size_col_ * resolution_, new_grid_oy + size_row_ * resolution_);
//         load_static_bottom_2.setPoint(origin_x_ + size_col_ * resolution_, new_grid_oy);
//         load_static_upper_2.setPoint(new_grid_ox + size_col_ * resolution_, new_grid_oy + size_row_ * resolution_);
//     }
//     else if(rolling_direction_ == 2){
//         write_old_bottom_1.setPoint(origin_x_, origin_y_);
//         write_old_upper_1.setPoint(origin_x_ + size_col_ * resolution_, new_grid_oy);
//         write_old_bottom_2.setPoint(new_grid_ox + size_col_ * resolution_, new_grid_oy);
//         write_old_upper_2.setPoint(origin_x_ + size_col_ * resolution_, origin_y_ + size_row_ * resolution_);
//         load_static_bottom_1.setPoint(new_grid_ox, new_grid_oy);
//         load_static_upper_1.setPoint(origin_x_, new_grid_oy + size_row_ * resolution_);
//         load_static_bottom_2.setPoint(origin_x_, origin_y_ + size_row_ * resolution_);
//         load_static_upper_2.setPoint(new_grid_ox + size_col_ * resolution_, new_grid_oy + size_row_ * resolution_);
//     }
//     else if(rolling_direction_ == 3){
//         write_old_bottom_1.setPoint(origin_x_, new_grid_oy + size_row_ * resolution_);
//         write_old_upper_1.setPoint(origin_x_ + size_col_ * resolution_, origin_y_ + size_row_ * resolution_);
//         write_old_bottom_2.setPoint(new_grid_ox + size_col_ * resolution_, origin_y_);
//         write_old_upper_2.setPoint(origin_x_ + size_col_ * resolution_, new_grid_oy + size_row_ * resolution_);
//         load_static_bottom_1.setPoint(new_grid_ox, new_grid_oy);
//         load_static_upper_1.setPoint(origin_x_, new_grid_oy + size_row_ * resolution_);
//         load_static_bottom_2.setPoint(origin_x_, new_grid_oy);
//         load_static_upper_2.setPoint(new_grid_ox + size_col_ * resolution_, origin_y_);
//     }
//     else{
//         write_old_bottom_1.setPoint(origin_x_, origin_y_);
//         write_old_upper_1.setPoint(new_grid_ox, origin_y_ + size_row_ * resolution_);
//         write_old_bottom_2.setPoint(new_grid_ox, new_grid_oy + size_row_ * resolution_);
//         write_old_upper_2.setPoint(origin_x_ + size_col_ * resolution_, origin_y_ + size_row_ * resolution_);
//         load_static_bottom_1.setPoint(new_grid_ox, new_grid_oy);
//         load_static_upper_1.setPoint(new_grid_ox + size_col_ * resolution_, origin_y_);
//         load_static_bottom_2.setPoint(origin_x_ + size_col_ * resolution_, origin_y_);
//         load_static_upper_2.setPoint(new_grid_ox + size_col_ * resolution_, new_grid_oy + size_row_ * resolution_);
//     }

//     //write to static layer
//     UpdateStaticLayer(write_old_bottom_1.x, write_old_bottom_1.y, write_old_upper_1.x, write_old_upper_1.y);
//     UpdateStaticLayer(write_old_bottom_2.x, write_old_bottom_2.y, write_old_upper_2.x, write_old_upper_2.y);

//     data_ = new_data_;
//     label_ = new_label_;
//     origin_x_ = new_grid_ox;
//     origin_y_ = new_grid_oy;

//     //load static information after update the origin

//     LoadStaticInfo(load_static_bottom_1.x, load_static_bottom_1.y, load_static_upper_1.x, load_static_upper_1.y);
//     LoadStaticInfo(load_static_bottom_2.x, load_static_bottom_2.y, load_static_upper_2.x, load_static_upper_2.y);

// }

void ObstacleLayer::UpdateOrigin(double new_origin_x, double new_origin_y) {
  // double dlt_distance = (new_origin_x - origin_x_))
  int dlt_row, dlt_col;
  dlt_row = int((new_origin_y - origin_y_) / resolution_);
  dlt_col = int((new_origin_x - origin_x_) / resolution_);
  double new_grid_ox, new_grid_oy;
  new_grid_ox = origin_x_ + dlt_col * resolution_;
  new_grid_oy = origin_y_ + dlt_row * resolution_;

  // copy region
  int start_cell_row_old, start_cell_col_old, end_cell_row_old, end_cell_col_old;
  int start_cell_row_new, start_cell_col_new, end_cell_row_new, end_cell_col_new;
  int size_row = size_row_;
  int size_col = size_col_;
  start_cell_col_old = std::min(std::max(dlt_col, 0), size_col);
  start_cell_row_old = std::min(std::max(-dlt_row, 0), size_row);
  end_cell_col_old = std::min(std::max(dlt_col + size_col, 0), size_col);
  end_cell_row_old = std::min(std::max(-dlt_row + size_row, 0), size_row);
  start_cell_col_new = std::min(std::max(-dlt_col, 0), size_col);
  start_cell_row_new = std::min(std::max(dlt_row, 0), size_row);
  end_cell_col_new = std::min(std::max(-dlt_col + size_col, 0), size_col);
  end_cell_row_new = std::min(std::max(dlt_row + size_row, 0), size_row);

  unsigned int cell_size_row = end_cell_row_old - start_cell_row_old;
  unsigned int cell_size_col = end_cell_col_old - start_cell_col_old;
  Matrix new_data_(size_row, size_col);
  Matrix new_label_(size_row, size_col);
  new_data_.setZero();
  new_label_.setZero();

  //       10              0                    40             40                           0                   10 40 40
  new_data_.block(start_cell_row_new, start_cell_col_new, cell_size_row, cell_size_col) =
      data_.block(start_cell_row_old, start_cell_col_old, cell_size_row, cell_size_col);
  new_label_.block(start_cell_row_new, start_cell_col_new, cell_size_row, cell_size_col) =
      label_.block(start_cell_row_old, start_cell_col_old, cell_size_row, cell_size_col);
  // cout<<"new_data_: "<<endl<<new_data_<<endl;

  Point2dT write_old_upperleft1, write_old_rightdown1, write_old_upperleft2, write_old_rightdown2;
  Point2dT load_new_upperleft1, load_new_rightdown1, load_new_upperleft2, load_new_rightdown2;

  if (rolling_direction_ == 1) { 
    write_old_upperleft1.setPoint(origin_x_, origin_y_);
    write_old_rightdown1.setPoint(new_grid_ox, origin_y_ - size_row_ * resolution_);
    write_old_upperleft2.setPoint(new_grid_ox, new_grid_oy - size_row_ * resolution_);
    write_old_rightdown2.setPoint(origin_x_ + size_col_ * resolution_, origin_y_ - size_row_ * resolution_);

    load_new_upperleft1.setPoint(new_grid_ox, new_grid_oy);
    load_new_rightdown1.setPoint(new_grid_ox + size_col_ * resolution_, origin_y_);
    load_new_upperleft2.setPoint(origin_x_ + size_col_ * resolution_, origin_y_);
    load_new_rightdown2.setPoint(new_grid_ox + size_col_ * resolution_, new_grid_oy - size_row_ * resolution_);
  } else if (rolling_direction_ == 2) {
    write_old_upperleft1.setPoint(origin_x_, origin_y_ - size_row_ * resolution_);
    write_old_rightdown1.setPoint(new_grid_ox + size_col_ * resolution_, origin_y_ - size_row_ * resolution_);
    write_old_upperleft2.setPoint(new_grid_ox + size_col_ * resolution_, origin_y_);
    write_old_rightdown2.setPoint(origin_x_ + size_col_ * resolution_, origin_y_ - size_row_ * resolution_);

    load_new_upperleft1.setPoint(new_grid_ox, new_grid_oy);
    load_new_rightdown1.setPoint(origin_x_, new_grid_oy - size_row_ * resolution_);
    load_new_upperleft2.setPoint(origin_x_, new_grid_oy);
    load_new_rightdown2.setPoint(new_grid_ox + size_col_ * resolution_, origin_y_);
  } else if (rolling_direction_ == 3) {
    write_old_upperleft1.setPoint(origin_x_, origin_y_);
    write_old_rightdown1.setPoint(new_grid_ox + size_col_ * resolution_, new_grid_oy);
    write_old_upperleft2.setPoint(new_grid_ox + size_col_ * resolution_, origin_y_);
    write_old_rightdown2.setPoint(origin_x_ + size_col_ * resolution_, origin_y_ - size_row_ * resolution_);

    load_new_upperleft1.setPoint(new_grid_ox, new_grid_oy);
    load_new_rightdown1.setPoint(origin_x_, new_grid_oy - size_row_ * resolution_);
    load_new_upperleft2.setPoint(origin_x_, origin_y_ - size_row_ * resolution_);
    load_new_rightdown2.setPoint(new_grid_ox + size_col_ * resolution_, new_grid_oy - size_row_ * resolution_);
  } else {
    write_old_upperleft1.setPoint(origin_x_, origin_y_);
    write_old_rightdown1.setPoint(new_grid_ox, origin_y_ - size_row_ * resolution_);
    write_old_upperleft2.setPoint(new_grid_ox, origin_y_);
    write_old_rightdown2.setPoint(origin_x_ + size_col_ * resolution_, new_grid_oy);

    load_new_upperleft1.setPoint(new_grid_ox, origin_y_ - size_row_ * resolution_);
    load_new_rightdown1.setPoint(origin_x_ + size_col_ * resolution_, new_grid_oy - size_row_ * resolution_);
    load_new_upperleft2.setPoint(origin_x_ + size_col_ * resolution_, new_grid_oy);
    load_new_rightdown2.setPoint(new_grid_ox - size_col_ * resolution_, new_grid_oy - size_row_ * resolution_);
  }

  // write to static layer
  UpdateStaticLayer(write_old_upperleft1.x, write_old_upperleft1.y, write_old_rightdown1.x, write_old_rightdown1.y);
  UpdateStaticLayer(write_old_upperleft2.x, write_old_upperleft2.y, write_old_rightdown2.x, write_old_rightdown2.y);

  data_ = new_data_;
  label_ = new_label_;
  origin_x_ = new_grid_ox;
  origin_y_ = new_grid_oy;

  LoadStaticInfo(load_new_upperleft1.x, load_new_upperleft1.y, load_new_rightdown1.x, load_new_rightdown1.y);
  LoadStaticInfo(load_new_upperleft2.x, load_new_upperleft2.y, load_new_rightdown2.x, load_new_rightdown2.y);
}

///////////////TOTTOTOTOTOTOTTODODODODDODODODODOD
// void ObstacleLayer::UpdateStaticLayer(double upper_left_x, double upper_left_y, double lower_right_x, double
// lower_right_y){
//     // uint32_t start_cell_col = static_cast<uint32_t>((upper_left_x - static_layer_->GetOriginX()) /
//     static_layer_->GetResolution()) + 1;
//     // uint32_t start_cell_row = static_cast<uint32_t>((-upper_left_y + static_layer_->GetOriginY()) /
//     static_layer_->GetResolution()) + 1;
//     // uint32_t end_cell_col = static_cast<uint32_t>((lower_right_x - static_layer_->GetOriginX()) /
//     static_layer_->GetResolution());
//     // uint32_t end_cell_row = static_cast<uint32_t>((-lower_right_y + static_layer_->GetOriginY()) /
//     static_layer_->GetResolution()); uint32_t start_cell_col = static_cast<uint32_t>((upper_left_x -
//     static_layer_->GetOriginX()) / 0.1) + 1; uint32_t start_cell_row = static_cast<uint32_t>((-upper_left_y +
//     static_layer_->GetOriginY()) / 0.1) + 1; uint32_t end_cell_col = static_cast<uint32_t>((lower_right_x -
//     static_layer_->GetOriginX()) / 0.1); uint32_t end_cell_row = static_cast<uint32_t>((-lower_right_y +
//     static_layer_->GetOriginY()) / 0.1); for(uint32_t i = start_cell_row; i < end_cell_row; i++){
//         for(uint32_t j = start_cell_col; j < end_cell_col; j++){
//             uint8_t costIn1Cell = 0;
//             double obs_lower_x = static_layer_->GetOriginX() + i * static_layer_->GetResolution();
//             double obs_lower_y = static_layer_->GetOriginY() + j * static_layer_->GetResolution();
//             uint32_t start_cell_obs_col = static_cast<uint32_t>((obs_lower_x - origin_x_) / resolution_);
//             uint32_t start_cell_obs_row = static_cast<uint32_t>((-obs_lower_y + origin_y_) / resolution_);
//             for(uint32_t k = start_cell_obs_row; k < start_cell_obs_row + fold_; k++){
//                 for(uint32_t l = start_cell_obs_col; l < start_cell_obs_col + fold_; l++){
//                     costIn1Cell += data_(k, l);    //how to update the data: replace or add?
//                 }
//             }
//             uint8_t obs_cost_average = costIn1Cell / (fold_ * fold_);
//             static_layer_->SetCost(i, j, obs_cost_average);

//         }
//     }
//     static_layer_->GetTime();
// }

void ObstacleLayer::UpdateStaticLayer(
    double upper_left_x, double upper_left_y, double lower_right_x, double lower_right_y) {
  double resolution = static_layer_->GetResolution();
  uint32_t start_cell_col = round((upper_left_x - static_layer_->GetOriginX()) / resolution);
  uint32_t start_cell_row = round((-upper_left_y + static_layer_->GetOriginY()) / resolution);
  uint32_t end_cell_col = round((lower_right_x - static_layer_->GetOriginX()) / resolution);
  uint32_t end_cell_row = round((-lower_right_y + static_layer_->GetOriginY()) / resolution);
  for (uint32_t i = start_cell_row; i < end_cell_row; i++) {
    for (uint32_t j = start_cell_col; j < end_cell_col; j++) {
      int costIn1Cell = 0;
      double obs_lower_x = static_layer_->GetOriginX() + j * static_layer_->GetResolution();
      double obs_lower_y = static_layer_->GetOriginY() - i * static_layer_->GetResolution();
      uint32_t start_cell_obs_col = round((obs_lower_x - origin_x_) / resolution_);
      uint32_t start_cell_obs_row = round((-obs_lower_y + origin_y_) / resolution_);
      for (uint32_t k = start_cell_obs_row; k < start_cell_obs_row + fold_; k++) {
        for (uint32_t l = start_cell_obs_col; l < start_cell_obs_col + fold_; l++) {
          costIn1Cell += data_(k, l);  // how to update the data: replace or add?
        }
      }
      int obs_cost_average = costIn1Cell / (fold_ * fold_);

      static_layer_->SetCost(i, j, obs_cost_average);
    }
  }
  static_layer_->GetTime();
}

void ObstacleLayer::SetRollingDirection(uint8_t direction) { rolling_direction_ = direction; }
