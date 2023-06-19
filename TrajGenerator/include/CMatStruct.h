//
// Created by bacon on 22-9-2.
//

#ifndef SRC_CPOSE2D_H
#define SRC_CPOSE2D_H
#include "CConfig.h"
#include "stdint.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float lin_spd;
  float ang_spd;

} CTwist2D;

typedef struct {
  float x;
  float y;
  float th;
} CPose2D;

typedef struct {
  float x;
  float y;
} CPoint2D;

typedef struct {
  float x;
  float y;
  float z;
} CPoint3D;

typedef struct {
  CPoint3D points[CLOUD_MAX_SIZE];
  uint32_t size;
  uint8_t isGlobalCoord;
} CCloud;

typedef struct {
  float kP, kI, kD, dt;
  float set_val, actual_val, last_val;
  float cur_err, last_err;
  float integration;
  float out_val;
  float max_out, min_out;
} CPid;

typedef struct {
  float ang_min;
  float ang_max;
  float ang_inc;
  float ranges[MAX_SCAN_POINTS];
  uint32_t rangesN;
  float intensities[MAX_SCAN_POINTS];
  uint32_t intensitiesN;
} CScan;

typedef struct {
  int width, height;
  float resolution;
  CPose2D origin;
  uint8_t data[LOCAL_MAP_MAX_SIZE];
} CLocalMapT;

typedef struct {
  CTwist2D speed;
  CPose2D simple_goal;
  CLocalMapT prob_map;
  CLocalMapT cost_map;
} COutput;

typedef struct {
  double init_time;
  double time;
  CPose2D pose;
  CScan scan;
  CCloud cloud;
  CLocalMapT cloud_map;
  CLocalMapT local_map;
} CInput;

// typedef enum {
//    FWS_NONE                 = 0,
//    FWS_WORKING              = 1,
//    FWS_RETURNED_ORIGIN_LINE = 1,
//    FWS_REACHED_NEXT_LINE    = 2,
//}FollowWallStateT;

typedef struct {
  float init_line_y;
  float cur_line_y;
  float cur_dir;
  float next_line;
  int8_t follow_hand;  // 1 means follow left, -1 means follow right
  //    FollowWallStateT follow_wall_state;
} CLineFollow;

typedef struct {
  int8_t bmp_back;
} TmpCnts;

typedef struct {
  CPose2D init_pose;
  CLineFollow line_follow;
  TmpCnts cnts;
  float tmp_edge_y;
  CPid spin_pid;
} CStatus;

typedef struct {
  int8_t clean_towards_y_plus;
  float max_linear_speed;
  float max_spin_speed;
  float spin_pid_params[3];
  float max_block_width;
  float sys_freq;
  float acc_time_s;  //加速到最大速度所需要时间
} CConfigT;

extern CConfigT configs;
extern CInput input;
extern COutput output;
extern CStatus state;

#ifdef __cplusplus
};
#endif
#endif  // SRC_CPOSE2D_H
