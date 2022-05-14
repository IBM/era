#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

// TODO: This is a simple fix for now; later move this struct definition to a global.h file so main.c and occgrid.c can just include that
typedef struct lidar_inputs_struct {
        float odometry[3];
        int data_size;
        char data[200002];
}
lidar_inputs_t;


#ifndef OCCGRID_H
#define OCCGRID_H

#ifdef INT_TIME
/* This is OCC-GRID Internal Timing information (gathering resources) */
extern uint64_t ocgr_c2g_total_sec;
extern uint64_t ocgr_c2g_total_usec;

extern uint64_t ocgr_c2g_initCM_sec;
extern uint64_t ocgr_c2g_initCM_usec;

extern uint64_t ocgr_c2g_updOrig_sec;
extern uint64_t ocgr_c2g_updOrig_usec;

extern uint64_t ocgr_c2g_updBnds_sec;
extern uint64_t ocgr_c2g_updBnds_usec;


extern uint64_t ocgr_upBd_total_sec;
extern uint64_t ocgr_upBd_total_usec;

extern uint64_t ocgr_upBd_rayFSp_sec;
extern uint64_t ocgr_upBd_rayFSp_usec;

extern uint64_t ocgr_upBd_regObst_sec;
extern uint64_t ocgr_upBd_regObst_usec;


/** No need to do this here -- provides no more info than exterior measure, really
    extern uint64_t ocgr_ryFS_total_sec;
    extern uint64_t ocgr_ryFS_total_usec;
    
    extern uint64_t ocgr_ryFS_rtLine_sec;
    extern uint64_t ocgr_ryFS_rtLine_usec;
**/
#endif

//Define Costmap values
//  Note: Order is significant -- combineGrids takes a "MAX" value from the two maps
#define CMV_NO_INFORMATION      0
#define CMV_FREE_SPACE          1
#define CMV_LETHAL_OBSTACLE   255

// Moved to the CMakeLists.txt or .config file.
#define GRID_MAP_X_DIM     100
#define GRID_MAP_Y_DIM     100
#define GRID_MAP_RESLTN    2.0
#define RAYTR_RANGE        100

#define COST_MAP_X_DIM     (GRID_MAP_X_DIM/(int)GRID_MAP_RESLTN)
#define COST_MAP_Y_DIM     (GRID_MAP_Y_DIM/(int)GRID_MAP_RESLTN)
#define COST_MAP_ENTRIES   (COST_MAP_X_DIM * COST_MAP_Y_DIM)

//Define public structures
typedef struct geometry_msgs_Point {
    double x;
    double y;
    double z;
} Point;

typedef struct MapLocation {
    unsigned int x;
    unsigned int y;
} MapLocation;

typedef struct Costmap2D_struct {
  double   av_x;
  double   av_y;
  double   av_z;
  //double   av_w;
  double   cell_size;
  unsigned int x_dim;
  unsigned int y_dim;
  unsigned char default_value;
  unsigned char costmap[COST_MAP_ENTRIES];
} Costmap2D;

typedef struct Observation {
  bool  rolling_window;
  Point master_origin;
  float master_resolution;
  double max_obstacle_height;
  double min_obstacle_height;
  double raytrace_range;
  //MapLocation map_coordinates;
  Costmap2D master_costmap;
} Observation;

//Define global variables
extern Observation master_observation;
extern bool rotating_window;

//Define functions
void fuseIntoLocal(Costmap2D* localMap, Costmap2D* updateMap);

void combineGrids(unsigned char* grid1, unsigned char* grid2, double robot_x1, double robot_y1,
		  double robot_x2, double robot_y2, unsigned int cell_size_x, unsigned int cell_size_y, double resolution/*, char def_val*/);

void cloudToOccgrid(Observation * obs_ptr, size_t obs_ptr_sz,
                lidar_inputs_t* lidar_inputs, size_t lidar_inputs_sz /*=sizeof(*lidar_inputs*/,
                double* robot_yaw, size_t robot_yaw_sz /*=sizeof(double)*/,
                bool* rolling_window, size_t rolling_window_sz /*=sizeof(bool)*/,
                double* min_obstacle_height, size_t min_obstable_height_sz /*=sizeof(double)*/,
                double* max_obstacle_height, size_t max_obstable_height_sz /*=sizeof(double)*/,
                double* raytrace_range, size_t raytrace_range_sz /*=sizeof(double)*/,
                unsigned int* x_dim, size_t x_dim_sz /*=sizeof(unsigned int)*/,
                unsigned int* y_dim, size_t y_dim_sz /*=sizeof(unsigned int)*/,
                unsigned int* resolution, size_t resolution_sz /*=sizeof(unsigned int)*/ /*, unsigned char default_value*/);

void printMap();

void init_occgrid_state(void);

void print_ascii_costmap(FILE* fptr, Costmap2D* cmap);

#endif // OCCGRID_H
