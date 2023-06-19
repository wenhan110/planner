#include "static_layer.h"

StaticLayer::StaticLayer(bool first_mapping) : last_update_utc_time_(0) {
  first_mapping_flag_ = first_mapping;
  Initialize();
}

StaticLayer::~StaticLayer() {
  for (auto global_map_file : global_map_file_list_) {
    fclose(global_map_file);
  }
}

void StaticLayer::Initialize() {
  if (first_mapping_flag_) {
    map_struct_.data_.resize(STATIC_MAP_ROWS, STATIC_MAP_COLS);
    map_struct_.label_.resize(STATIC_MAP_ROWS, STATIC_MAP_COLS);
    map_struct_.resolution_ = STATIC_MAP_RESOLUTION;
    map_struct_.origin_x_ = -STATIC_MAP_COLS * STATIC_MAP_RESOLUTION / 2.0;
    map_struct_.origin_y_ = STATIC_MAP_ROWS * STATIC_MAP_RESOLUTION / 2.0;
    map_struct_.last_update_utc_time_ = 0;
    // global_map_file_list_.push_back(cur_global_map);
  }
  else {
    FILE* cur_global_map = fopen("global_map.bin", "rb+");
    // std::cout<<"open success"<<std::endl;
    ReadFromBinary(cur_global_map, map_struct_);
    // cout<<"read success"<<endl;
    // fread(&map_struct_, sizeof(MapStruct), 1, cur_global_map);

    // global_map_file_list_.push_back(cur_global_map);
  }
  data_ = map_struct_.data_;
  label_ = map_struct_.label_;
  resolution_ = map_struct_.resolution_;
  size_row_ = data_.rows();
  size_col_ = data_.cols();
  origin_x_ = map_struct_.origin_x_;
  origin_y_ = map_struct_.origin_y_;
  last_update_utc_time_ = map_struct_.last_update_utc_time_;
}

// void StaticLayer::WriteGlobalMap(FILE* global_map_file) {
//   global_map_.data_ = data_;
//   global_map_.label_ = label_;
//   global_map_.resolution_ = resolution_;
//   global_map_.last_update_utc_time_ = last_update_utc_time_;
//   fwrite(&global_map_, sizeof(GlobalMap), 1, global_map_file);
//   // global_map_file_list_.push_back(global_map_file);
// }


void StaticLayer::WriteGlobalMap(FILE *global_map_file) {
    map_struct_.data_ = data_;
    map_struct_.label_ = label_;
    map_struct_.resolution_ = resolution_;
    // cout<<"resolution_: "<<resolution_<<endl;
    map_struct_.last_update_utc_time_ = last_update_utc_time_;
    map_struct_.origin_x_ = origin_x_;
    map_struct_.origin_y_ = origin_y_;
    // cout<<"size of global_map: "<<sizeof(GlobalMap)<<endl;
    Write2Binary(global_map_file, map_struct_);
}



void StaticLayer::ResizeMap(uint8_t direction) {
  if (!needResizeGlobalMap_)
    return;
  else {
    if (direction == RESIZING_RIGHTUP) {
      Matrix new_data_(data_.rows() + STATIC_MAP_ROWS, data_.cols() + STATIC_MAP_COLS);
      Matrix new_label_(label_.rows() + STATIC_MAP_ROWS, label_.cols() + STATIC_MAP_COLS);
      new_data_.block(STATIC_MAP_ROWS, 0, data_.rows(), data_.cols()) = data_;
      new_label_.block(STATIC_MAP_COLS, 0, label_.rows(), label_.cols()) = label_;
      data_ = new_data_;
      label_ = new_label_;
      origin_y_ = origin_y_ + STATIC_MAP_ROWS * resolution_;
    } else if (direction == RESIZING_LEFTUP) {
      Matrix new_data_(data_.rows() + STATIC_MAP_ROWS, data_.cols() + STATIC_MAP_COLS);
      Matrix new_label_(label_.rows() + STATIC_MAP_ROWS, label_.cols() + STATIC_MAP_COLS);
      new_data_.block(STATIC_MAP_ROWS, STATIC_MAP_COLS, data_.rows(), data_.cols()) = data_;
      new_label_.block(STATIC_MAP_ROWS, STATIC_MAP_COLS, label_.rows(), label_.cols()) = label_;
      data_ = new_data_;
      label_ = new_label_;
      origin_x_ = origin_x_ - STATIC_MAP_COLS * resolution_;
      origin_y_ = origin_y_ + STATIC_MAP_ROWS * resolution_;
    } else if (direction == RESIZING_LEFTDOWN) {
      Matrix new_data_(data_.rows() + STATIC_MAP_ROWS, data_.cols() + STATIC_MAP_COLS);
      Matrix new_label_(label_.rows() + STATIC_MAP_ROWS, label_.cols() + STATIC_MAP_COLS);
      new_data_.block(0, STATIC_MAP_COLS, data_.rows(), data_.cols()) = data_;
      new_label_.block(0, STATIC_MAP_COLS, label_.rows(), label_.cols()) = label_;
      data_ = new_data_;
      label_ = new_label_;
      origin_x_ = origin_x_ - STATIC_MAP_COLS * resolution_;
    } else if (direction == RESIZING_RIGHTDOWN) {
      Matrix new_data_(data_.rows() + STATIC_MAP_ROWS, data_.cols() + STATIC_MAP_COLS);
      Matrix new_label_(label_.rows() + STATIC_MAP_ROWS, label_.cols() + STATIC_MAP_COLS);
      new_data_.block(0, 0, data_.rows(), data_.cols()) = data_;
      new_label_.block(0, 0, label_.rows(), label_.cols()) = label_;
      data_ = new_data_;
      label_ = new_label_;
    } else if (direction == RESIZING_UP) {
      Matrix new_data_(data_.rows() + STATIC_MAP_ROWS, data_.cols());
      Matrix new_label_(label_.rows() + STATIC_MAP_ROWS, label_.cols());
      new_data_.block(STATIC_MAP_ROWS, 0, data_.rows(), data_.cols()) = data_;
      new_label_.block(STATIC_MAP_ROWS, 0, label_.rows(), label_.cols()) = label_;
      data_ = new_data_;
      label_ = new_label_;
      origin_y_ = origin_y_ + STATIC_MAP_ROWS * resolution_;
    } else if (direction == RESIZING_DOWN) {
      Matrix new_data_(data_.rows() + STATIC_MAP_ROWS, data_.cols());
      Matrix new_label_(label_.rows() + STATIC_MAP_ROWS, label_.cols());
      new_data_.block(0, 0, data_.rows(), data_.cols()) = data_;
      new_label_.block(0, 0, label_.rows(), label_.cols()) = label_;
      data_ = new_data_;
      label_ = new_label_;
    } else if (direction == RESIZING_LEFT) {
      Matrix new_data_(data_.rows(), data_.cols() + STATIC_MAP_COLS);
      Matrix new_label_(label_.rows(), label_.cols() + STATIC_MAP_COLS);
      new_data_.block(0, STATIC_MAP_COLS, data_.rows(), data_.cols()) = data_;
      new_label_.block(0, STATIC_MAP_COLS, label_.rows(), label_.cols()) = label_;
      data_ = new_data_;
      label_ = new_label_;
      origin_x_ = origin_x_ - STATIC_MAP_COLS * resolution_;
    } else if (direction == RESIZING_RIGHT) {
      Matrix new_data_(data_.rows(), data_.cols() + STATIC_MAP_COLS);
      Matrix new_label_(label_.rows(), label_.cols() + STATIC_MAP_COLS);
      new_data_.block(0, 0, data_.rows(), data_.cols()) = data_;
      new_label_.block(0, 0, label_.rows(), label_.cols()) = label_;
      data_ = new_data_;
      label_ = new_label_;
    } else {
      printf("Resize map error!\n");
      exit(1);
    }
    size_row_ = data_.rows();
    size_col_ = data_.cols();

  }
}

void StaticLayer::GetTime() {
  time_t now = time(0);
  last_update_utc_time_ = now;
}