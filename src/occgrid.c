#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/time.h>

#include "globals.h"
#include "occgrid.h"

#ifdef INT_TIME
/* This is OCC-GRID Internal Timing information (gathering resources) */
struct timeval /*ocgr_c2g_total_stop,*/ ocgr_c2g_total_start;
uint64_t ocgr_c2g_total_sec  = 0LL;
uint64_t ocgr_c2g_total_usec = 0LL;

struct timeval ocgr_c2g_initCM_stop; /*, ocgr_c2g_initCM_start;*/
uint64_t ocgr_c2g_initCM_sec  = 0LL;
uint64_t ocgr_c2g_initCM_usec = 0LL;

struct timeval ocgr_c2g_updOrig_stop; /*, ocgr_c2g_updOrig_start;*/
uint64_t ocgr_c2g_updOrig_sec  = 0LL;
uint64_t ocgr_c2g_updOrig_usec = 0LL;

struct timeval ocgr_c2g_updBnds_stop; /*, ocgr_c2g_updBnds_start;*/
uint64_t ocgr_c2g_updBnds_sec  = 0LL;
uint64_t ocgr_c2g_updBnds_usec = 0LL;


struct timeval ocgr_upBd_total_stop, ocgr_upBd_total_start;
uint64_t ocgr_upBd_total_sec  = 0LL;
uint64_t ocgr_upBd_total_usec = 0LL;

struct timeval ocgr_upBd_rayFSp_stop; /*, ocgr_upBd_rayFSp_start;*/
uint64_t ocgr_upBd_rayFSp_sec  = 0LL;
uint64_t ocgr_upBd_rayFSp_usec = 0LL;

/*struct timeval ocgr_upBd_regObst_stop, ocgr_upBd_regObst_start;*/
uint64_t ocgr_upBd_regObst_sec  = 0LL;
uint64_t ocgr_upBd_regObst_usec = 0LL;


/** No need to do this here -- provides no more info than exterior measure, really
    struct timeval ocgr_ryFS_total_stop, ocgr_ryFS_total_start;
    uint64_t ocgr_ryFS_total_sec  = 0LL;
    uint64_t ocgr_ryFS_total_usec = 0LL;
    
    struct timeval ocgr_ryFS_rtLine_stop, ocgr_ryFS_rtLine_start;
    uint64_t ocgr_ryFS_rtLine_sec  = 0LL;
    uint64_t ocgr_ryFS_rtLine_usec = 0LL;
**/
#endif

/** FORWARD DECLARATIONS **/
//void touch(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y);
void updateBounds(Observation* obs_ptr, float* data, unsigned int data_size, double robot_x, double robot_y, double robot_z, double robot_yaw,
		  double* min_x, double* min_y, double* max_x, double* max_y);
void raytraceFreespace(Observation* obs_ptr, float* data, unsigned int data_size,
		       double* min_x, double* min_y, double* max_x, double* max_y,
		       double robot_x, double robot_y, double robot_z, double robot_yaw);
/*void updateRaytraceBounds(double ox, double oy, double wx, double wy, double range,
  double* min_x, double* min_y, double* max_x, double* max_y);*/
//void updateMap(Observation* obs_ptr, float* data, unsigned int data_size, double robot_x, double robot_y, double robot_z, double robot_yaw);

void updateOrigin(Observation* obs_ptr, double new_origin_x, double new_origin_y);

void copyMapRegion(Observation* obs_ptr, unsigned char* source_map, unsigned int sm_lower_left_x, unsigned int sm_lower_left_y,
		   unsigned int sm_size_x, unsigned int dm_lower_left_x,
		   unsigned int dm_lower_left_y, unsigned int dm_size_x, unsigned int region_size_x,
		   unsigned int region_size_y);

//bool worldToMapCell(Observation* obs_ptr, double wx, double wy, double robot_x, double robot_y, int* nx, int* ny));

//unsigned int cellDistance(Observation* obs_ptr, double world_dist);

void raytraceLine(Observation* obs_ptr, unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1, unsigned int max_length); //TODO: Default max argument for max_length is UNIT_MAX

//void bresenham2D(Observation* obs_ptr, unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a, int offset_b, unsigned int offset, unsigned int max_length);

//void markCell(Observation* obs_ptr, unsigned char value, unsigned int offset);

//unsigned int getIndex(Observation* obs_ptr, unsigned int x, unsigned int y);

void addStaticObstacle(unsigned char* obstacle_type);

void initCostmap(Observation* obs_ptr,
		 bool rolling_window,
		 double min_obstacle_height, double max_obstacle_height, double raytrace_range,
		 unsigned int size_x, unsigned int size_y, double resolution,
		 /*unsigned char default_value,*/
		 double robot_x, double robot_y, double robot_z);


//Define global variables
//Observation master_observation;
//char data[199992];
bool rotating_window;

/*************** HELPER FUNCTIONS ******************/

static inline double hypot_dist(double x, double y) {
  return sqrt(x * x + y * y);
}

// Adding MACRO versions; a bit more dangerous (if used improperly) but could be faster.
#define MMIN(x, y) (((x) > (y)) ? (y)  : (x))
#define MMAX(x, y) (((x) > (y)) ? (x)  : (y))
#define MSIGN(x)   (((x) > 0) ? 1 : -1);

/*static inline int max(int num1, int num2) {
  return (num1 > num2) ? num1 : num2;
  }*/

/*static inline int min(int num1, int num2) {
  return (num1 < num2) ? num1 : num2;
  }*/

/*static inline int sign (int x) {
  return x > 0 ? 1.0 : -1.0;
}*/

/*static inline void touch(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y) {
  *min_x = MMIN(x, *min_x);
  *min_y = MMIN(y, *min_y);
  *max_x = MMAX(x, *max_x);
  *max_y = MMAX(y, *max_y);
  }*/

static inline unsigned int cellDistance(Observation* obs_ptr, double world_dist) {
  double cells_dist = MMAX(0.0, ceil(world_dist/obs_ptr->master_resolution));
  return (unsigned int) cells_dist;
}

static inline unsigned int getIndex(Observation* obs_ptr, unsigned int i, unsigned int j) {
  //printf("x_dim * j + i = %d * %d + %d = %d", obs_ptr->master_costmap.x_dim, j, i, obs_ptr->master_costmap.x_dim * j + i);
  return (obs_ptr->master_costmap.x_dim / obs_ptr->master_resolution) * j + i;
}

static inline void markCell(Observation* obs_ptr, unsigned char value, unsigned int offset) {
  //printf("OFFSET -> %d\n", offset);
  CHECK(if (offset >= COST_MAP_ENTRIES) {
      printf("ERROR : markCell : offset is too large at %d vs %d\n", offset, COST_MAP_ENTRIES);
    });
  obs_ptr->master_costmap.costmap[offset] = value;
}


//Calculate world coordinates relative to origin (and not robot's odometry)
static inline bool worldToMapCell(Observation* obs_ptr, double owx, double owy, double robot_x, double robot_y, int* nwx, int* nwy) {
  double owx_rel_origin = owx + robot_x;
  double owy_rel_origin = owy + robot_y;
  //printf("World To Map (Relative to Origin) = (%d, %d)\n", (int)((owx_rel_origin - obs_ptr->master_origin.x) / obs_ptr->master_resolution), (int)((owy_rel_origin - obs_ptr->master_origin.y) / obs_ptr->master_resolution));
  if ((owx_rel_origin < obs_ptr->master_origin.x) || (owy_rel_origin < obs_ptr->master_origin.y)) {
    DBGOUT(printf("Coordinates Out Of Bounds .... (owx, owy) = (%f, %f); (ox, oy) = (%f, %f)\n", owx, owy, obs_ptr->master_origin.x, obs_ptr->master_origin.y));
    return false;
  }

  *nwx = (int)((owx_rel_origin - obs_ptr->master_origin.x) / obs_ptr->master_resolution);
  *nwy = (int)((owy_rel_origin - obs_ptr->master_origin.y) / obs_ptr->master_resolution);

  //printf("World To Map (owx, owy) = (%f, %f) -> (mx, my) = (%d, %d)\n\n", owx, owy, obs_ptr->map_coordinates.x, obs_ptr->map_coordinates.y);

  if ((*nwx < obs_ptr->master_costmap.x_dim) && (*nwy < obs_ptr->master_costmap.y_dim)) {
    return true;
  }
  return false;
}

/* void printMap() {
  printf("map: \n");
  for (int i = 0; i < obs_ptr->master_costmap.y_dim / obs_ptr->master_resolution; i++) {
  for (int j = 0; j < obs_ptr->master_costmap.x_dim / obs_ptr->master_resolution; j++) {
  int index = i * obs_ptr->master_costmap.y_dim / obs_ptr->master_resolution + j;
  printf("%4d", obs_ptr->master_costmap.costmap[index]);
  }
  printf("\n\n");
  }
  } */

/** This is not used?  Should be cleaned up, anyway:
    Move from a string input to an enum type for obstaclt_type?
    Do a proper strcmp -- I think this will ALWAYS FAIL (as it compares pointer addresses)
**/
#if(0)
void addStaticObstacle(Observation* obs_ptr, unsigned char* obstacle_type) {
  int cell_x_dim = obs_ptr->master_costmap.x_dim / obs_ptr->master_resolution;
  int cell_y_dim = obs_ptr->master_costmap.y_dim / obs_ptr->master_resolution;
  if (obstacle_type == "border") {
    printf("Add Static Obstacle: Wall Border \n");
    for (int i = 0; i < cell_x_dim; i++) {
      for (int j = 0; j < cell_y_dim; j++) {
	int index = cell_x_dim * j + i;
	CHECK(if (index >= COST_MAP_ENTRIES) {
	    printf("ERROR : addStaticObstacle index too large at %d vs %d\n", index, COST_MAP_ENTRIES);
	  });
	if (i == (int) obs_ptr->master_origin.x || i == cell_x_dim - 1) {
	  obs_ptr->master_costmap.costmap[index] = CMV_LETHAL_OBSTACLE;
	} else if (j == (int) obs_ptr->master_origin.y || j == cell_y_dim - 1 ) {
	  obs_ptr->master_costmap.costmap[index] = CMV_LETHAL_OBSTACLE;
	}
      }
    }
  }
}
#endif

void initCostmap(Observation* obs_ptr,
		 bool rolling_window, double min_obstacle_height, double max_obstacle_height, double raytrace_range, unsigned int x_dim,
                 unsigned int y_dim, double resolution, /*unsigned char default_value,*/ double robot_x, double robot_y, double robot_z) {
  DBGOUT(printf("Initialize Master Costmap\n"));

  obs_ptr->rolling_window = rolling_window; //TODO:
  obs_ptr->min_obstacle_height = min_obstacle_height; //TODO:
  obs_ptr->max_obstacle_height = max_obstacle_height; //TODO:
  obs_ptr->raytrace_range = raytrace_range; //TODO:

  obs_ptr->master_costmap.cell_size = resolution;
  obs_ptr->master_costmap.x_dim = x_dim;
  obs_ptr->master_costmap.y_dim = y_dim;
  obs_ptr->master_costmap.default_value = CMV_NO_INFORMATION; // default_value;

  obs_ptr->master_costmap.av_x = robot_x;
  obs_ptr->master_costmap.av_y = robot_y;
  obs_ptr->master_costmap.av_z = robot_z;

  obs_ptr->master_resolution = resolution;

  obs_ptr->master_origin.x = robot_x - (x_dim - 1) / 2;
  obs_ptr->master_origin.y = robot_y - (y_dim - 1) / 2;
  obs_ptr->master_origin.z = robot_z;
  DBGOUT(printf("Master Origin -> <%f, %f, %f>\n", obs_ptr->master_origin.x, obs_ptr->master_origin.y, obs_ptr->master_origin.z));

  CHECK(int chkMaxIdx = obs_ptr->master_costmap.x_dim * obs_ptr->master_costmap.y_dim / (obs_ptr->master_resolution * obs_ptr->master_resolution);
	if (chkMaxIdx > COST_MAP_ENTRIES) {
	  printf("ERROR : initCostMap : Max index is too large at %d vs %d\n", chkMaxIdx, COST_MAP_ENTRIES);
	});
  for (int i = 0; i < obs_ptr->master_costmap.x_dim * obs_ptr->master_costmap.y_dim / (obs_ptr->master_resolution * obs_ptr->master_resolution); i++) {
    obs_ptr->master_costmap.costmap[i] = CMV_NO_INFORMATION; // obs_ptr->master_costmap.default_value;
  }

  DBGOUT(printf("Initialize Master Costmap ... DONE\n\n"));
}

/******************* FUNCTIONS *********************/

void fuseIntoLocal(Costmap2D* theLocal, Costmap2D* theInput)
{
  DBGOUT(printf("Entered fuseIntoLocal...\n"));
  //localMap is the persistent map fo local knowledge; inputMap is the new information to enter into the localMap
  unsigned char* localMap = theLocal->costmap;
  unsigned char* inputMap = theInput->costmap;
  // These are the "position" of the (center) of the map
  int local_x_pos = (int)((float)theLocal->av_x / theLocal->cell_size);
  int local_y_pos = (int)((float)theLocal->av_y / theLocal->cell_size);
  int input_x_pos = (int)((float)theInput->av_x / theInput->cell_size);
  int input_y_pos = (int)((float)theInput->av_y / theInput->cell_size);
  // These are the dimension (raw input) of the map
  int local_x_dim = theLocal->x_dim;
  int local_y_dim = theLocal->y_dim;
  int input_x_dim = theInput->x_dim;
  int input_y_dim = theInput->y_dim;
  // Thess are the number of costmap/gridmap "cells" (dimensionally)
  int local_x_cells = (local_x_dim / theLocal->cell_size);
  int local_y_cells = (local_y_dim / theLocal->cell_size);
  int input_x_cells = (input_x_dim / theInput->cell_size);
  int input_y_cells = (input_y_dim / theInput->cell_size);
  //int resolution = theLocal->cell_size;
  /* DBGOUT(printf("in fuseIntoLocal: x1 = %d x2 = %d  y1 = %d y2 = %d\n", input_x, local_x, input_y, local_y); */
  /* 	 printf("      num_cells : w_x = %u  w_y = %u   u_x = %u  u_y = %u\n", local_x_cells, local_y_cells, input_x_cells, input_y_cells)); */

  DBGOUT(printf(" Local in-data: X-pos %d y_pos %d x_cells %d y_cells %d\n", local_x_pos, local_y_pos, local_x_cells, local_y_cells);
	 printf(" Input in-data: X-pos %d y_pos %d x_cells %d y_cells %d\n", input_x_pos, input_y_pos, input_x_cells, input_y_cells));
  // Express the local/input maps in terms of "Global Cell Coordinates"
  int input_0_x = input_x_pos - input_x_cells/2;
  int input_0_y = input_y_pos - input_x_cells/2;
  int input_N_x = input_x_pos + input_x_cells/2 - 1;
  int input_N_y = input_y_pos + input_x_cells/2 - 1;
  DBGOUT(printf("Input: 0x Nx = %d  %d   :   0y Ny = %d  %d\n", input_0_x, input_N_x, input_0_y, input_N_y));

  int local_0_x = local_x_pos - local_x_cells/2;
  int local_0_y = local_y_pos - local_x_cells/2;
  int local_N_x = local_x_pos + local_x_cells/2 - 1;
  int local_N_y = local_y_pos + local_x_cells/2 - 1;
  DBGOUT(printf("Local: 0x Nx = %d  %d   :   0y Ny = %d  %d\n", local_0_x, local_N_x, local_0_y, local_N_y));

  // Now determine the (localized) X and Y cell dimensions that overlap
  //  We compute the min and max X and Y as for a for loop: for (ix = x_min; ix < x_max; ix++) 
  int input_x_min = MMAX(0, (local_0_x - input_0_x));
  DBGOUT(printf("   input_x_min = MMAX(0, (%d - %d) = %d) = %d\n", local_0_x, input_0_x, (local_0_x - input_0_x), input_x_min));
  int input_x_max = input_x_cells - MMAX(0, (input_N_x - local_N_x));
  DBGOUT(printf("   input_x_max = %d - MMAX(0, (%d - %d) = %d) = %d\n", input_x_cells, input_0_x, local_0_x, (input_N_x - local_N_x), input_x_max));
  int input_y_min = MMAX(0, (local_0_y - input_0_y));
  DBGOUT(printf("   input_y_min = MMAX(0, (%d - %d) = %d) = %d\n", local_0_y, input_0_y, (local_0_y - input_0_y), input_y_min));
  int input_y_max = input_y_cells - MMAX(0, (input_N_y - local_N_y));
  DBGOUT(printf("   input_y_max = %d - MMAX(0, (%d - %d) = %d) = %d\n", input_y_cells, input_0_y, local_0_y, (input_N_y - local_N_y), input_y_max));
  DBGOUT(printf("Input: x_min %d  x_max %d  y_min %d  y_max %d\n", input_x_min, input_x_max, input_y_min, input_y_max));

  int local_x_min = MMAX(0, (input_0_x - local_0_x));
  int local_x_max = local_x_cells - MMAX(0, (local_N_x - input_N_x));
  int local_y_min = MMAX(0, (local_0_y, input_0_y));
  int local_y_max = local_y_cells - MMAX(0, (local_N_y - input_N_y));
  DBGOUT(printf("Local: x_min %d  x_max %d  y_min %d  y_max %d\n", local_x_min, local_x_max, local_y_min, local_y_max));

  //Iterate through grids and assign corresponding max value
  int iiy = input_y_min;
  for (int wiy = local_y_min; wiy < local_y_max; wiy++) {
    int wbase = wiy*local_x_cells;
    int ibase = iiy*input_x_cells;
    int iix = input_x_min;
    for (int wix = local_x_min; wix < local_x_max; wix++) {
      int local_idx = wbase + wix;
      int input_idx = ibase + iix;
      CHECK(if ((local_idx < 0) || (local_idx >= COST_MAP_ENTRIES)) {
      	  printf("ERROR : fuseIntoLocal local_idx outside bounds at %d vs 0 .. %d\n", local_idx, COST_MAP_ENTRIES);
      	});
      CHECK(if ((input_idx < 0) || (input_idx >= COST_MAP_ENTRIES)) {
      	  printf("ERROR : fuseIntoLocal input_idx outside bounds at %d vs 0 .. %d\n", input_idx, COST_MAP_ENTRIES);
      	});
      //DBGOUT(printf("  Setting localMap[%d] = MMAX(%u, %u = inputMap[%d])\n", local_idx, localMap[local_idx], inputMap[input_idx], input_idx));
      localMap[local_idx] = MMAX(localMap[local_idx], inputMap[input_idx]);
      iix++;
    }
    iiy++;
  }
  return;
}

/* The combineGrids function takes two input map grids, grid1 and grid2, 
   and "fuses" (or combines) the information from both into grid2
   (overwriting some or all af that grid's contents).
*/
void combineGrids(unsigned char* grid1, unsigned char* grid2,
		  double robot_x1, double robot_y1,
		  double robot_x2, double robot_y2,
		  unsigned int x_dim, unsigned int y_dim, double resolution
		  /*,char def_val*/ )
{
  DBGOUT(printf("in combineGrids: x1 = %.1f x2 = %.1f  y1 = %.1f y2 = %.1f\n", robot_x1, robot_x2, robot_y1, robot_y2);
	 printf("     dimensions: x = %u  y = %u\n", x_dim, y_dim));
  //grid1 is previous map, grid2 is current map

  //Calculate the old origin of the map
  double origin_x = robot_x1 - (x_dim - 1) / 2;
  double origin_y = robot_y1 - (y_dim - 1) / 2;
	   
  //Calculate the new origin of the map
  double new_origin_x = robot_x2 - (x_dim - 1) / 2.0;
  double new_origin_y = robot_y2 - (y_dim - 1) / 2.0;

  //Calculate the number of cells between the old and new origin
  int cell_ox = ((new_origin_x - origin_x) / resolution);
  int cell_oy = ((new_origin_y - origin_y) / resolution);
  DBGOUT(printf("cell_ox = (%.1f - %.1f) / %.1f = %.1f = %d\n", new_origin_x, origin_x, resolution, ((new_origin_x - origin_x) / resolution), cell_ox);
	 printf("cell_oy = (%.1f - %.1f) / %.1f = %.1f = %d\n", new_origin_y, origin_y, resolution, ((new_origin_y - origin_y) / resolution), cell_oy));
  // Determine the dimensions (x, y) in terms of Grid cells
  int cell_x_dim = (int)(x_dim / resolution);
  int cell_y_dim = (int)(y_dim / resolution);
    
  //Determine the lower left cells of the origin
  int g1_lower_left_x = MMIN(MMAX(cell_ox, 0), cell_x_dim);
  int g1_lower_left_y = MMIN(MMAX(cell_oy, 0), cell_y_dim);
  int g2_lower_left_x = g1_lower_left_x - cell_ox;
  int g2_lower_left_y = g1_lower_left_y - cell_oy;

  //Calculate the indexes of which to start 'copying' over
  int g1_index = (g1_lower_left_y * cell_x_dim + g1_lower_left_x);
  int g2_index = (g2_lower_left_y * cell_x_dim + g2_lower_left_x);

  //The size of the overlapping region
  int region_x_dim = cell_x_dim - cell_ox;
  int region_y_dim = cell_y_dim - cell_oy;
  
  DBGOUT(printf("origin : %.1lf %.1lf   new_origin: %.1lf %.1lf\n", origin_x, origin_y, new_origin_x, new_origin_y);
	 printf("cell : ox,y %d %d  dimx,y %d %d\n", cell_ox, cell_oy, cell_x_dim, cell_y_dim);
	 printf("Lower Left: Old (%d, %d)  New (%d, %d)\n", g1_lower_left_x, g1_lower_left_y, g2_lower_left_x, g2_lower_left_y);
	 //printf("Lower Left of New Map = (%d, %d) \n", g2_lower_left_x, g2_lower_left_y);
	 printf("Index of Old Map, Index of New Map = %d, %d \n", g1_index, g2_index);
	 printf("Cell_Dimensions: X %d y %d\n", cell_x_dim, cell_y_dim);
	 printf("Dimensions of Overlapping Region = (%d, %d) \n", region_x_dim, region_y_dim));
  //Iterate through grids and assign corresponding max value
  unsigned int total_count = 0;
  unsigned int incr_ct = 0;
  unsigned int count = 0;
  for (int j = 0; j < cell_y_dim; j++) {
    for (int i = 0; i < cell_x_dim; i++) {
      printf("i %d  j %d  g1_index %d  g1_index %d  count %d  tot_count %d DIM^2: %d\n", i, j, g1_index, g2_index, count, total_count, cell_x_dim * cell_y_dim);
      if (g1_index >= cell_x_dim * cell_y_dim) return;
      if (g2_index >= cell_x_dim * cell_y_dim) return;
      if (count >= region_x_dim) {
	//g1_index = g1_index + cell_ox;
	g1_index = (g1_lower_left_y + incr_ct) * cell_x_dim + g1_lower_left_x;
	//g2_index = g2_index + cell_ox;
	g2_index = (g2_lower_left_y + incr_ct) * cell_x_dim + g2_lower_left_x;
	count = 0;
	incr_ct++;
      }
      if (g1_index >= cell_x_dim * cell_y_dim) return;
      if (g2_index >= cell_x_dim * cell_y_dim) return;
      CHECK(if ((g2_index < 0) || (g2_index >= COST_MAP_ENTRIES)) {
	  printf("ERROR : combineGrids g2_index too large at %d vs %d\n", g2_index, COST_MAP_ENTRIES);
	});
      CHECK(if ((g1_index < 0) || (g1_index >= COST_MAP_ENTRIES)) {
	  printf("ERROR : combineGrids g1_index too large at %d vs %d\n", g1_index, COST_MAP_ENTRIES);
	});

      grid2[g2_index] = MMAX(grid2[g2_index], grid1[g1_index]);
      //DBGOUT(printf("%d : %d v %d : %d, %d \n", total_count, count, region_x_dim, g1_index, g2_index));
      g1_index++;
      g2_index++;
      count++;
      total_count++;
    }
  }
  return;
}

/*void updateMap(Observation* obs_ptr,
	       float* data, unsigned int data_size,
	       double robot_x, double robot_y, double robot_z, double robot_yaw) {
  if (obs_ptr->rolling_window) {
    //printf("\nUpdating Map .... \n");
    //printf("   robot_x = %f, robot_y = %f, robot_yaw = %f \n", robot_x, robot_y, robot_yaw);
    //printf("   Master Origin = (%f, %f)\n", obs_ptr->master_origin.x, obs_ptr->master_origin.y);
    double new_origin_x = robot_x - obs_ptr->master_costmap.x_dim / 2;
    double new_origin_y = robot_y - obs_ptr->master_costmap.y_dim / 2;
    updateOrigin(obs_ptr, new_origin_x, new_origin_y);
  }

  double min_x = 1e30;
  double min_y = 1e30;
  double max_x = -1e30;
  double max_y = -1e30;

  //printf("(1) Number of elements : %d ... ", data_size);
  //printf("First Coordinate = <%f, %f>\n", *data, *(data+1));
  //rotating_window = true; //Comment out if not rolling window

  updateBounds(obs_ptr, data, data_size, robot_x, robot_y, robot_z, robot_yaw, &min_x, &min_y, &max_x, &max_y);
  }*/

unsigned char* cloudToOccgrid(Observation* obs_ptr,
			      float* data, unsigned int data_size,
			      double robot_x, double robot_y, double robot_z, double robot_yaw,
			      bool rolling_window,
			      double min_obstacle_height, double max_obstacle_height,
			      double raytrace_range,
			      unsigned int x_dim, unsigned int y_dim, double resolution/*,
			      unsigned char default_value*/ ) {
  DBGOUT(printf("In cloudToOccgrid with Odometry %.1f %.1f %.f1\n", robot_x, robot_y, robot_z));
 #ifdef INT_TIME
  gettimeofday(&ocgr_c2g_total_start, NULL);
 #endif  
  initCostmap(obs_ptr, rolling_window, min_obstacle_height, max_obstacle_height, raytrace_range, x_dim, y_dim, resolution, /*default_value,*/ robot_x, robot_y, robot_z);
 #ifdef INT_TIME
  gettimeofday(&ocgr_c2g_initCM_stop, NULL);
  ocgr_c2g_initCM_sec  += ocgr_c2g_initCM_stop.tv_sec  - ocgr_c2g_total_start.tv_sec;
  ocgr_c2g_initCM_usec += ocgr_c2g_initCM_stop.tv_usec - ocgr_c2g_total_start.tv_usec;
 #endif

  //printf("(1) Number of elements : %d ... ", data_size);
  //printf("First Coordinate = <%f, %f>\n", *data, *(data+1));
  //MOVED to physically inlined here... updateMap(obs_ptr, data, data_size, robot_x, robot_y, robot_z, robot_yaw);
  if (obs_ptr->rolling_window) {
    //printf("\nUpdating Map .... \n");
    //printf("   robot_x = %f, robot_y = %f, robot_yaw = %f \n", robot_x, robot_y, robot_yaw);
    //printf("   Master Origin = (%f, %f)\n", obs_ptr->master_origin.x, obs_ptr->master_origin.y);
    double new_origin_x = robot_x - obs_ptr->master_costmap.x_dim / 2;
    double new_origin_y = robot_y - obs_ptr->master_costmap.y_dim / 2;
    updateOrigin(obs_ptr, new_origin_x, new_origin_y);
  }
 #ifdef INT_TIME
  gettimeofday(&ocgr_c2g_updOrig_stop, NULL);
  ocgr_c2g_updOrig_sec  += ocgr_c2g_updOrig_stop.tv_sec  - ocgr_c2g_initCM_stop.tv_sec;
  ocgr_c2g_updOrig_usec += ocgr_c2g_updOrig_stop.tv_usec - ocgr_c2g_initCM_stop.tv_usec;
 #endif

  double min_x = 1e30;
  double min_y = 1e30;
  double max_x = -1e30;
  double max_y = -1e30;

  //printf("(1) Number of elements : %d ... ", data_size);
  //printf("First Coordinate = <%f, %f>\n", *data, *(data+1));
  //rotating_window = true; //Comment out if not rolling window

  updateBounds(obs_ptr, data, data_size, robot_x, robot_y, robot_z, robot_yaw, &min_x, &min_y, &max_x, &max_y);

  //printMap();
 #ifdef INT_TIME
  gettimeofday(&ocgr_c2g_updBnds_stop, NULL);
  ocgr_c2g_updBnds_sec  += ocgr_c2g_updBnds_stop.tv_sec  - ocgr_c2g_updOrig_stop.tv_sec;
  ocgr_c2g_updBnds_usec += ocgr_c2g_updBnds_stop.tv_usec - ocgr_c2g_updOrig_stop.tv_usec;

  ocgr_c2g_total_sec  += ocgr_c2g_updBnds_stop.tv_sec  - ocgr_c2g_total_start.tv_sec;
  ocgr_c2g_total_usec += ocgr_c2g_updBnds_stop.tv_usec - ocgr_c2g_total_start.tv_usec;
 #endif
  return obs_ptr->master_costmap.costmap;
}


void updateOrigin(Observation* obs_ptr, double new_origin_x, double new_origin_y) {
  //printf("\nUpdating Map Origin\n");
  //printf("New Origin -> <%f, %f>\n ", new_origin_x, new_origin_y);

  //project the new origin into the grid
  int cell_ox, cell_oy;
  //printf("Old Origin = <%f, %f> ... ", obs_ptr->master_origin.x, obs_ptr->master_origin.y);
  //printf("New Origin = <%f, %f>\n", new_origin_x, new_origin_y);
  cell_ox = (int) ((new_origin_x - obs_ptr->master_origin.x) / obs_ptr->master_resolution);
  cell_oy = (int) ((new_origin_y - obs_ptr->master_origin.y) / obs_ptr->master_resolution);
  //printf("New Cell Origin = <%d, %d>\n", cell_ox, cell_oy);

  // compute the associated world coordinates for the origin cell
  // because we want to keep things grid-aligned
  double new_grid_ox, new_grid_oy;
  new_grid_ox = obs_ptr->master_origin.x + cell_ox * obs_ptr->master_resolution;
  new_grid_oy = obs_ptr->master_origin.y + cell_oy * obs_ptr->master_resolution;
  //printf("New Grid Origin = <%f, %f> (Should be same as new_origin_x and new_origin_y)\n", new_grid_ox, new_grid_oy);

  // To save casting from unsigned int to int a bunch of times
  int x_dim = obs_ptr->master_costmap.x_dim / obs_ptr->master_resolution;
  int y_dim = obs_ptr->master_costmap.y_dim / obs_ptr->master_resolution;

  // we need to compute the overlap of the new and existing windows
  int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  lower_left_x = MMIN(MMAX(cell_ox, 0), x_dim);
  lower_left_y = MMIN(MMAX(cell_oy, 0), y_dim);
  upper_right_x = MMIN(MMAX(cell_ox + x_dim, 0), x_dim);
  upper_right_y = MMIN(MMAX(cell_oy + y_dim, 0), y_dim);
  //printf("The Corner Coordinates for Window = {%d, %d} {%d, %d}\n", lower_left_x, lower_left_y, upper_right_x, upper_right_y);

  unsigned int cell_x_dim = (upper_right_x - lower_left_x);
  unsigned int cell_y_dim = (upper_right_y - lower_left_y);
  //printf("Cell Sizes from Corner Coordinates = %d, %d\n", cell_x_dim, cell_y_dim);

  // we need a map to store the obstacles in the window temporarily
  //unsigned char* local_map [cell_x_dim * cell_y_dim]; //Uncomment if in C++

  // now we'll set the costmap to be completely unknown if we track unknown space
  //resetMaps();

  // update the origin with the appropriate world coordinates
  obs_ptr->master_origin.x = new_grid_ox;
  obs_ptr->master_origin.y = new_grid_oy;

  // compute the starting cell location for copying data back in
  int start_x = lower_left_x - cell_ox;
  int start_y = lower_left_y - cell_oy;
  //printf("lower_left_x - cell_ox = start_x ... (%d) - (%d) = %d\n", lower_left_x, cell_ox, start_x);
  //printf("lower_left_y - cell_oy = start_y ... (%d) - (%d) = %d\n", lower_left_y, cell_oy, start_y);

  // copy the local window in the costmap to the local map
  copyMapRegion(obs_ptr, obs_ptr->master_costmap.costmap, lower_left_x, lower_left_y,
		obs_ptr->master_costmap.x_dim / obs_ptr->master_resolution,
		start_x, start_y,
		obs_ptr->master_costmap.x_dim / obs_ptr->master_resolution,
		cell_x_dim, cell_y_dim);


  // now we want to copy the overlapping information back into the map, but in its new location
  //copyMapRegion(local_map, 0, 0, cell_x_dim, obs_ptr->master_costmap.costmap, start_x, start_y, obs_ptr->master_costmap.x_dim, cell_x_dim, cell_y_dim);
}

//TODO: Modify such that it explicitly copies the data to the destination map
void copyMapRegion(Observation* obs_ptr, unsigned char* source_map,
		   unsigned int sm_lower_left_x, unsigned int sm_lower_left_y, unsigned int sm_x_dim,
		   unsigned int dm_lower_left_x, unsigned int dm_lower_left_y, unsigned int dm_x_dim,
		   unsigned int region_x_dim,    unsigned int region_y_dim) {
  // we'll first need to compute the starting points for each map
  //printf("CopyMapRegion() Input Parameters: \n ... sm_lower_left_x = %d,\n ... sm_lowerLeft_y = %d,\n ... sm_x_dim = %d,\n ... dm_lower_left_x = %d,\n ... dm_lower_left_y = %d,\n ... dm_x_dim = %d,\n ... regions_x_dim = %d,\n ... region_y_dim = %d\n",sm_lower_left_x, sm_lower_left_y, sm_x_dim, dm_lower_left_x, dm_lower_left_y, dm_x_dim, region_x_dim, region_y_dim);
  unsigned int sm_index = (sm_lower_left_y * sm_x_dim + sm_lower_left_x);
  unsigned int dm_index = (dm_lower_left_y * dm_x_dim + dm_lower_left_x);
  //printf("%c\n", obs_ptr->master_costmap.default_value);
  //printf("{sm_index = %d, dm_index = %d}\n", sm_index, dm_index);

  unsigned int cell_x_dim = obs_ptr->master_costmap.x_dim / obs_ptr->master_resolution;
  unsigned int cell_y_dim = obs_ptr->master_costmap.y_dim / obs_ptr->master_resolution;

  //printf("\n Copying Map... \nRegion Size of Map -> <%d, %d>\n", region_x_dim, region_y_dim);
  char local_costmap [cell_x_dim * cell_y_dim];
  for (int i = 0; i < cell_x_dim * cell_y_dim; i++) {
    local_costmap[i] = CMV_NO_INFORMATION; // obs_ptr->master_costmap.default_value;
    //printf("%d, ", local_costmap[i]);
  }

  // now, we'll copy the source map into the destination map
  for (unsigned int i = 0; i < region_y_dim; ++i){
    for (unsigned int j = 0; j < region_x_dim; j++) {
      //printf("Source Map Value at Index <%d> = %d\n", sm_index, obs_ptr->master_costmap.costmap[sm_index]);
      CHECK(if (dm_index >= COST_MAP_ENTRIES) {
	  printf("ERROR : copyMapRegion : dm_index is too large at = %d vs %d\n", dm_index, COST_MAP_ENTRIES);
	}
	if (sm_index >= COST_MAP_ENTRIES) {
	  printf("ERROR : copyMapRegion : sm_index is too large at = %d vs %d\n", sm_index, COST_MAP_ENTRIES);
	});
      local_costmap[dm_index] = obs_ptr->master_costmap.costmap[sm_index];
      //printf("dm_index, sm_index = %d, %d\n", dm_index, sm_index);
      sm_index++;
      dm_index++;
    }
    if (obs_ptr->master_costmap.x_dim != region_x_dim) {
      sm_index = sm_index + (obs_ptr->master_costmap.x_dim / obs_ptr->master_resolution - region_x_dim);
      dm_index = dm_index + (obs_ptr->master_costmap.x_dim / obs_ptr->master_resolution - region_x_dim);
    }
    //memcpy(dm_index, sm_index, region_x_dim * sizeof(unsigned char*));
  }

  //printf("We made it!\n");

  CHECK(int chkMaxIdx = cell_x_dim * cell_y_dim;
	if (chkMaxIdx > COST_MAP_ENTRIES) {
	  printf("ERROR : copyMapRegion : Max index is too large at %d vs %d\n", chkMaxIdx, COST_MAP_ENTRIES);
	});
  for (int i = 0; i < cell_x_dim * cell_y_dim; i++) {
    obs_ptr->master_costmap.costmap[i] = local_costmap[i];
  }
}

void updateBounds(Observation* obs_ptr, float* data, unsigned int data_size, double robot_x, double robot_y, double robot_z, double robot_yaw,
		  double* min_x, double* min_y, double* max_x, double* max_y) {
  //printf("(1) Number of elements : %d ... ", data_size);
  //printf("First Coordinate = <%f, %f>\n", *data, *(data+1));

 #ifdef INT_TIME
  gettimeofday(&ocgr_upBd_total_start, NULL);
 #endif  
  //raytrace free space
  raytraceFreespace(obs_ptr, data, data_size, min_x, min_y, max_x, max_y, robot_x, robot_y, robot_z, robot_yaw); //TODO: Reconfigure for 'cloud' parameter
 #ifdef INT_TIME
  gettimeofday(&ocgr_upBd_rayFSp_stop, NULL);
  ocgr_upBd_rayFSp_sec  += ocgr_upBd_rayFSp_stop.tv_sec  - ocgr_upBd_total_start.tv_sec;
  ocgr_upBd_rayFSp_usec += ocgr_upBd_rayFSp_stop.tv_usec - ocgr_upBd_total_start.tv_usec;
 #endif

  //printf("Number of elements : %d\n", sizeof(cloud.data) / sizeof(cloud.data[0]));

  //Iterate through cloud to register obstacles within costmap
  for(unsigned int i = 0; i < data_size; i = i + 3) { //TODO: Test if sizeof(points) works correctly
    //Only consider points within height boundaries
    if ((data[i + 2] <= obs_ptr->max_obstacle_height) && (data[i + 2] >= obs_ptr->min_obstacle_height)) {
      double px = (double) *(data + i);
      double py = (double) *(data + i + 1);
      double pz = (double) *(data + i + 2);

      //if window rotates, then inversely rotate points
      if (rotating_window) {
	px = px*cos(robot_yaw) - py*sin(robot_yaw);
	py = px*sin(robot_yaw) + py*cos(robot_yaw);
      }

      //printf("World Coordinates (wx, wy) = (%f, %f)\n", px, py);
      //printf("Master Origin Coordinate = < %f, %f > \n", obs_ptr->master_origin.x, obs_ptr->master_origin.y);

      //printf("Map Coordinates [BEFORE Conversion] -> (%d, %d)\n", obs_ptr->map_coordinates.x, obs_ptr->map_coordinates.y);
      int wx, wy;
      worldToMapCell(obs_ptr, px, py, robot_x, robot_y, &wx, &wy);
      //printf("Map Coordinates (mx, my) = (%d, %d)\n", obs_ptr->map_coordinates.x, obs_ptr->map_coordinates.y);

      unsigned int index = getIndex(obs_ptr, wx, wy);
      CHECK(if (index >= COST_MAP_ENTRIES) {
	  printf("ERROR : updateBounds : index is too large at %d vs %d\n", index, COST_MAP_ENTRIES);
	});
      //printf("Index of Obstacle -> %d\n", index);
      obs_ptr->master_costmap.costmap[index] = CMV_LETHAL_OBSTACLE; //TODO: Test simple test case (char) 255 = '255' ?
      //touch(px, py, min_x, min_y, max_x, max_y);
    }
  }
 #ifdef INT_TIME
  gettimeofday(&ocgr_upBd_total_stop, NULL);
  ocgr_upBd_regObst_sec  += ocgr_upBd_total_stop.tv_sec  - ocgr_upBd_rayFSp_stop.tv_sec;
  ocgr_upBd_regObst_usec += ocgr_upBd_total_stop.tv_usec - ocgr_upBd_rayFSp_stop.tv_usec;

  ocgr_upBd_total_sec  += ocgr_upBd_total_stop.tv_sec  - ocgr_upBd_total_start.tv_sec;
  ocgr_upBd_total_usec += ocgr_upBd_total_stop.tv_usec - ocgr_upBd_total_start.tv_usec;
 #endif
}

/* Issues:
   (1) Finding the size of the array for iteration
   (2) Difference between origin_x_ and observation.origin_x
*/
void raytraceFreespace(Observation* obs_ptr, float* data, unsigned int data_size,
		       double* min_x, double* min_y, double* max_x, double* max_y,
		       double robot_x, double robot_y, double robot_z, double robot_yaw) {
  //printf("(1) Number of elements : %d ... ", data_size);
  //printf("First Coordinate = <%f, %f>\n", *data, *(data+1));
  /** No need to do this here -- provides no more info than exterior measure, really
      #ifdef INT_TIME
      gettimeofday(&ocgr_ryFS_total_start, NULL);
      #endif  **/
  //Retrieve observation origin (i.e. origin of the pointcloud)
  double ox = robot_x;
  double oy = robot_y;
  //printf(">>> Odometry -> <%f, %f>\n", ox, oy);

  // get the map coordinates of the origin of the sensor
  int x0, y0;
  if (!worldToMapCell(obs_ptr, 0, 0, robot_x, robot_y, &x0, &y0)) {//TODO:
    printf("The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.\n", ox, oy);
    return;
  }
  //printf(">>> Map Coordinates of the Sensor Origin -> <%d, %d>\n", x0, y0);

  // we can pre-compute the endpoints of the map outside of the inner loop... we'll need these later
  double map_end_x = obs_ptr->master_origin.x + obs_ptr->master_costmap.x_dim * obs_ptr->master_resolution;
  double map_end_y = obs_ptr->master_origin.y + obs_ptr->master_costmap.y_dim * obs_ptr->master_resolution;
  //printf(">>> End of Map Coordinates -> <%f, %f>\n", map_end_x, map_end_y);

  //touch(ox, oy, min_x, min_y, max_x, max_y);

  const double sin_yaw = sin(robot_yaw);
  const double cos_yaw = cos(robot_yaw);
  const double mo_x = obs_ptr->master_origin.x;
  const double mo_y = obs_ptr->master_origin.y;
  
  for (int i = 0; i < data_size; i = i + 3) {
    double wx = (double) *(data + i);
    double wy = (double) *(data + i + 1);
    //printf(">>> World Coordinates of Data Point -> <%f, %f>\n", wx, wy);

    //if window rotates, then inversely rotate points
    if (rotating_window) {
      //wx = wx*cos(robot_yaw) - wy*sin(robot_yaw);
      wx = wx*cos_yaw - wy*sin_yaw;
      //wy = wx*sin(robot_yaw) + wy*cos(robot_yaw);
      wy = wx*sin_yaw + wy*cos_yaw;
    }

    // now we also need to make sure that the enpoint we're raytracing
    // to isn't off the costmap and scale if necessary
    double a = wx - ox;
    double b = wy - oy;
    
    if (wx < mo_x) { // obs_ptr->master_origin.x) {
      //double t = (obs_ptr->master_origin.x - ox) / a;
      double t = (mo_x - ox) / a;
      //wx = obs_ptr->master_origin.x;
      wx = mo_x;
      wy = oy + b * t;
    }

    if (wy < mo_y) { // obs_ptr->master_origin.y) {
      //double t = (obs_ptr->master_origin.y - oy) / b;
      double t = (mo_y - oy) / b;
      wx = ox + a * t;
      //wy = obs_ptr->master_origin.y;
      wy = mo_y;
    }

    // the maximum value to raytrace to is the end of the map
    if (wx > map_end_x) {
      double t = (map_end_x - ox) / a;
      wx = map_end_x - .001;
      wy = oy + b * t;
    }
    if (wy > map_end_y) {
      double t = (map_end_y - oy) / b;
      wx = ox + a * t;
      wy = map_end_y - .001;
    }
    
    // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
    int x1, y1;
    if (!worldToMapCell(obs_ptr, wx, wy, robot_x, robot_y, &x1, &y1)) continue;

    unsigned int cell_raytrace_range = cellDistance(obs_ptr, obs_ptr->raytrace_range);
    //printf(">>> Cell Raytrace Range -> %d\n", cell_raytrace_range);

    // and finally... we can execute our trace to clear obstacles along that line
    /** We cannot do this here -- WAY TOO MUCH OVERHEAD!! 
	#ifdef INT_TIME
	gettimeofday(&ocgr_ryFS_rtLine_start, NULL);
	#endif  **/
    raytraceLine(obs_ptr, x0, y0, x1, y1, cell_raytrace_range);
    /** We cannot do this here -- WAY TOO MUCH OVERHEAD!! 
	#ifdef INT_TIME
	gettimeofday(&ocgr_ryFS_rtLine_stop, NULL);
	ocgr_ryFS_rtLine_sec  += ocgr_ryFS_rtLine_stop.tv_sec  - ocgr_ryFS_rtLine_start.tv_sec;
	ocgr_ryFS_rtLine_usec += ocgr_ryFS_rtLine_stop.tv_usec - ocgr_ryFS_rtLine_start.tv_usec;
	#endif  **/

    /* No apparent effect
       updateRaytraceBounds(ox, oy, wx, wy, obs_ptr->raytrace_range, min_x, min_y, max_x, max_y);
    */
  }
  /** No need to do this here -- provides no more info than exterior measure, really
      #ifdef INT_TIME
      gettimeofday(&ocgr_ryFS_total_stop, NULL);
      ocgr_ryFS_total_sec  += ocgr_ryFS_total_stop.tv_sec  - ocgr_ryFS_total_start.tv_sec;
      ocgr_ryFS_total_usec += ocgr_ryFS_total_stop.tv_usec - ocgr_ryFS_total_start.tv_usec;
      #endif  **/
}


static inline void bresenham2D(Observation* obs_ptr, unsigned int abs_da, unsigned int abs_db, int error_b,
			       int offset_a, int offset_b, unsigned int offset, unsigned int max_length) {
  unsigned int end = MMIN(max_length, abs_da);
  //printf("\n\n abs_da, end -> %d, %d\n", abs_da, end);
  for (unsigned int i = 0; i < end; ++i) {
    markCell(obs_ptr, CMV_FREE_SPACE, offset);
    offset += offset_a;
    error_b += abs_db;
    if ((unsigned int)error_b >= abs_da) {
      offset += offset_b;
      error_b -= abs_da;
    }
  }
  markCell(obs_ptr, CMV_FREE_SPACE, offset);
}


void raytraceLine(Observation* obs_ptr, unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1, unsigned int max_length) { //TODO: default parameter for max_length is UNIT_MAX; define UNIT_MAX
  //printf(">>> Raytrace Line from <%d, %d> to <%d, %d> \n", x0, y0, x1, y1);
  int dx = x1 - x0;
  int dy = y1 - y0;
  //printf("dx, dy -> %d ,%d\n", dx, dy);

  unsigned int abs_dx = abs(dx);
  unsigned int abs_dy = abs(dy);

  int offset_dx = MSIGN(dx);
  //printf("offset_dx -> %d, \n", offset_dx);
  //printf("cell_x_dim -> %d \n", (int) (obs_ptr->master_costmap.x_dim / obs_ptr->master_resolution));
  int sdy = MSIGN(dy);
  int offset_dy = sdy * (int)(obs_ptr->master_costmap.x_dim / obs_ptr->master_resolution);
  //printf("offset_dy -> %d \n", offset_dy);

  unsigned int offset = y0 * obs_ptr->master_costmap.x_dim / obs_ptr->master_resolution + x0;
  //printf("offset -> %d \n", offset);

  // we need to chose how much to scale our dominant dimension, based on the maximum length of the line
  double dist = hypot_dist(dx, dy);
  double scale = (dist == 0.0) ? 1.0 : MMIN(1.0, max_length / dist);

  // if x is dominant
  if (abs_dx >= abs_dy) {
    int error_y = abs_dx / 2;
    bresenham2D(obs_ptr, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx));
  } else {
    // otherwise y is dominant
    int error_x = abs_dy / 2;
    bresenham2D(obs_ptr, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy));
  }
}

// Interestingly, we comput min_x, min_y, etc. via touch() calls and this updateRayTraceBounds
//  call, but the computed min/max don't appear to ever get used, or have any real
//  effect on the run results...
// I suspect this was a desgn element that might be intended ot reduce the ray-trace
//  search (distance?) but not either fully implemented or needed...
/*void updateRaytraceBounds(double ox, double oy, double wx, double wy, double range,
                          double* min_x, double* min_y, double* max_x, double* max_y) {
  double dx = wx - ox, dy = wy - oy;
  double full_distance = hypot(dx, dy);
  double scale = MMIN(1.0, range / full_distance);
  double ex = ox + dx * scale, ey = oy + dy * scale;
  touch(ex, ey, min_x, min_y, max_x, max_y);
  }*/




char pr_map_char[256];

void init_occgrid_state()
{
  // Set up the print-map-character array (to xlate map values to ASCII symbols)
  for (int i = 0; i < 256; i++) {
    pr_map_char[i] = '?';
  }
  pr_map_char[CMV_NO_INFORMATION]  = '.';
  pr_map_char[CMV_FREE_SPACE]      = ' ';
  pr_map_char[CMV_LETHAL_OBSTACLE] = 'X';
}

void print_ascii_costmap(FILE*  fptr, Costmap2D* cmap)
{
  fprintf(fptr, "  ");
  unsigned h1 = 0;
  unsigned h10 = 0;
  for (int ii = 0; ii < COST_MAP_X_DIM; ii++) {
    fprintf(fptr, "%u", h10);
    h1++;
    if (h1 == 10) { h1 = 0; h10++; }
    if (h10 == 10) { h10 = 0;}
  }
  fprintf(fptr, "\n  ");
  for (int ii = 0; ii < COST_MAP_X_DIM; ii++) {
    fprintf(fptr, "%u", ii%10);
  }
  fprintf(fptr, "\n  ");
  for (int ii = 0; ii < COST_MAP_X_DIM; ii++) {
    fprintf(fptr, "-");
  }
  fprintf(fptr, "\n  ");
  for (int ii = 0; ii < COST_MAP_X_DIM; ii++) {
    for (int ij = 0; ij < COST_MAP_Y_DIM; ij++) {
      int idx = COST_MAP_X_DIM*ii + ij;
      //printf("%2x ", cmap->costmap[idx]);
      fprintf(fptr, "%c", pr_map_char[cmap->costmap[idx]]);
    }
    fprintf(fptr, " | %3u\n  ", ii);
    //printf("| %3u\n", ii);
  }
  fprintf(fptr, "\n");
  //printf("\n");
}

