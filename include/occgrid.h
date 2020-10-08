#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#ifndef OCCGRID_H_
#define OCCGRID_H_

//Define Costmap values
//  Note: Order is significant -- combineGrids takes a "MAX" value from the two maps
#define CMV_NO_INFORMATION      0
#define CMV_FREE_SPACE          1
#define CMV_LETHAL_OBSTACLE   255

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
    bool  rolling_window_;
    Point master_origin;
    float master_resolution;
    double max_obstacle_height_;
    double min_obstacle_height_;
    double raytrace_range_;
    MapLocation map_coordinates;
    Costmap2D master_costmap;
} Observation;

//Define global variables
extern Observation master_observation;
extern bool rotating_window;

//Define functions
void combineGrids(unsigned char* grid1, unsigned char* grid2, double robot_x1, double robot_y1,
                            double robot_x2, double robot_y2, unsigned int cell_size_x, unsigned int cell_size_y, double resolution, char def_val);

unsigned char* cloudToOccgrid(float* data, unsigned int data_size, double robot_x, double robot_y, double robot_z, double robot_yaw, bool rolling_window,
                              double min_obstacle_height, double max_obstacle_height, double raytrace_range, unsigned int size_x,
                              unsigned int size_y, double resolution, unsigned char default_value);

void updateMap(float* data, unsigned int data_size, double robot_x, double robot_y, double robot_z, double robot_yaw);

void updateOrigin(double new_origin_x, double new_origin_y);

void copyMapRegion(unsigned char* source_map, unsigned int sm_lower_left_x, unsigned int sm_lower_left_y,
                       unsigned int sm_size_x, unsigned int dm_lower_left_x,
                       unsigned int dm_lower_left_y, unsigned int dm_size_x, unsigned int region_size_x,
                       unsigned int region_size_y);

void updateBounds(float* data, unsigned int data_size, double robot_x, double robot_y, double robot_z, double robot_yaw, double min_x, double min_y, double max_x, double max_y);

void raytraceFreespace(float* data, unsigned int data_size, double min_x, double min_y, double max_x, double max_y, double robot_x, double robot_y, double robot_z, double robot_yaw);

bool worldToMap(double wx, double wy, double robot_x, double robot_y);

unsigned int cellDistance(double world_dist);

void raytraceLine(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1, unsigned int max_length); //TODO: Default max argument for max_length is UNIT_MAX

void bresenham2D(unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
                        int offset_b, unsigned int offset, unsigned int max_length);

void markCell(unsigned char value, unsigned int offset);

void updateRaytraceBounds(double ox, double oy, double wx, double wy, double range,
                          double min_x, double min_y, double max_x, double max_y);

void touch(double x, double y, double min_x, double min_y, double max_x, double max_y);

unsigned int getIndex(unsigned int x, unsigned int y);

void printMap();

void addStaticObstacle(unsigned char* obstacle_type);

void initCostmap(Observation* obsvtn,
		 bool rolling_window,
		 double min_obstacle_height, double max_obstacle_height, double raytrace_range,
		 unsigned int size_x, unsigned int size_y, double resolution,
		 unsigned char default_value,
		 double robot_x, double robot_y, double robot_z);

void init_occgrid_state(void);
void print_ascii_costmap(Costmap2D* cmap);

#endif // OCCGRID_H_
