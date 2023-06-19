#include "CostmapBase.h"

CostmapBase::CostmapBase() : size_row_(0), size_col_(0), resolution_(0), origin_x_(0), origin_y_(0) {
  data_.resize(size_row_, size_col_);
  label_.resize(size_row_, size_col_);
}


void CostmapBase::Write2Binary(FILE* file, const MapStruct &map){
    // FILE* file = fopen(filename, "wb+");
    int data_rows = map.data_.rows();
    int data_cols = map.data_.cols();
    int label_rows = map.label_.rows();
    int label_cols = map.label_.cols();
    float res_ = map.resolution_;
    // cout<<"size of res_: "<<sizeof(res_)<<endl;
    double origin_x_ = map.origin_x_;
    double origin_y_ = map.origin_y_;
    // cout<<"origin_y: "<<origin_y_<<endl;
    Time time_ = map.last_update_utc_time_;
    fwrite(&data_rows, sizeof(int), 1, file);
    fwrite(&data_cols, sizeof(int), 1, file);
    fwrite(&label_rows, sizeof(int), 1, file);
    fwrite(&label_cols, sizeof(int), 1, file);
    fwrite(&time_, sizeof(Time), 1, file);

    // cout<<"map.resolution_: "<<res_<<endl;
    fwrite(&origin_x_, sizeof(double), 1, file);
    fwrite(&origin_y_, sizeof(double), 1, file);
    fwrite(&res_, sizeof(float), 1, file);
    fwrite(map.data_.data(), sizeof(int), data_rows*data_cols, file);
    fwrite(map.label_.data(), sizeof(int), label_rows*label_cols, file);
    fclose(file);
}

void CostmapBase::ReadFromBinary(FILE* file, MapStruct &map){
    // FILE* file = fopen(filename, "rb");
    int data_rows, data_cols, label_rows, label_cols;
    fread(&data_rows, sizeof(int), 1, file);
    fread(&data_cols, sizeof(int), 1, file);
    fread(&label_rows, sizeof(int), 1, file);
    fread(&label_cols, sizeof(int), 1, file);
    fread(&map.last_update_utc_time_, sizeof(Time), 1, file);

    // cout<<"map.resolution_: "<<map.resolution_<<endl;
    fread(&map.origin_x_, sizeof(double), 1, file);
    fread(&map.origin_y_, sizeof(double), 1, file);
    fread(&map.resolution_, sizeof(float), 1, file);
    map.data_.resize(data_rows, data_cols);
    map.label_.resize(label_rows, label_cols);
    fread(map.data_.data(), sizeof(int), data_rows*data_cols, file);
    fread(map.label_.data(), sizeof(int), label_rows*label_cols, file);
    fclose(file);
}