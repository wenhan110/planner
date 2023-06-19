#ifndef COSTMAP_STATIC_LAYER_H
#define COSTMAP_STATIC_LAYER_H
#include <time.h>

#include <list>
#include <vector>
#include "CostmapBase.h"

// struct GlobalMap {
//   Matrix data_;
//   Matrix label_;
//   float resolution_;
//   Time last_update_utc_time_;
//   double origin_x_;
//   double origin_y_;
// };

class StaticLayer : public CostmapBase {
 public:
  StaticLayer(bool first_mapping);
  ~StaticLayer();

  bool needResizeGlobalMap_ = false;

  void Initialize();

  void WriteGlobalMap(FILE *global_map_file);

  void ResizeMap(uint8_t direction);

  void GetTime();

 private:
//   GlobalMap global_map_;

  Time last_update_utc_time_;

  std::list<FILE *> global_map_file_list_;
};

#endif  // COSTMAP_STATIC_LAYER_H
