#include <stdio.h>
#include <stdlib.h>

#include "globals.h"
#include "occgrid.h"

/** FORWARD DECLARATIONS **/
void touch(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y);


//Define global variables
Worldmap2D theWorldMap;
bool initWorldMap = true;
Observation master_observation;
//char data[199992];
bool rotating_window;

/*************** HELPER FUNCTIONS ******************/

inline double hypot(double x, double y) {
  return sqrt(x * x + y * y);
}

int max(int num1, int num2) {
  return (num1 > num2) ? num1 : num2;
}

int min(int num1, int num2) {
  return (num1 < num2) ? num1 : num2;
}

int sign (int x) {
  return x > 0 ? 1.0 : -1.0;
}

void touch(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y) {
  *min_x = min(x, *min_x);
  *min_y = min(y, *min_y);
  *max_x = max(x, *max_x);
  *max_y = max(y, *max_y);
}

unsigned int getIndex(unsigned int i, unsigned int j) {
  //printf("x_dim * j + i = %d * %d + %d = %d", master_observation.master_costmap.x_dim, j, i, master_observation.master_costmap.x_dim * j + i);
  return (master_observation.master_costmap.x_dim / master_observation.master_resolution) * j + i;
}

/*
  void printMap() {
  printf("map: \n");
  for (int i = 0; i < master_observation.master_costmap.y_dim / master_observation.master_resolution; i++) {
  for (int j = 0; j < master_observation.master_costmap.x_dim / master_observation.master_resolution; j++) {
  int index = i * master_observation.master_costmap.y_dim / master_observation.master_resolution + j;
  printf("%4d", master_observation.master_costmap.costmap[index]);
  }
  printf("\n\n");
  }
  }
*/

/** This is not used?  Should be cleaned up, anyway:
    Move from a string input to an enum type for obstaclt_type?
    Do a proper strcmp -- I think this will ALWAYS FAIL (as it compares pointer addresses)
**/
#if(0)
void addStaticObstacle(unsigned char* obstacle_type) {
  int cell_x_dim = master_observation.master_costmap.x_dim / master_observation.master_resolution;
  int cell_y_dim = master_observation.master_costmap.y_dim / master_observation.master_resolution;
  if (obstacle_type == "border") {
    printf("Add Static Obstacle: Wall Border \n");
    for (int i = 0; i < cell_x_dim; i++) {
      for (int j = 0; j < cell_y_dim; j++) {
	int index = cell_x_dim * j + i;
	CHECK(if (index >= COST_MAP_ENTRIES) {
	    printf("ERROR : addStaticObstacle index too large at %d vs %d\n", index, COST_MAP_ENTRIES);
	  });
	if (i == (int) master_observation.master_origin.x || i == cell_x_dim - 1) {
	  master_observation.master_costmap.costmap[index] = CMV_LETHAL_OBSTACLE;
	} else if (j == (int) master_observation.master_origin.y || j == cell_y_dim - 1 ) {
	  master_observation.master_costmap.costmap[index] = CMV_LETHAL_OBSTACLE;
	}
      }
    }
  }
}
#endif

void initCostmap(Observation* obsvtn,
		 bool rolling_window, double min_obstacle_height, double max_obstacle_height, double raytrace_range, unsigned int x_dim,
                 unsigned int y_dim, double resolution, /*unsigned char default_value,*/ double robot_x, double robot_y, double robot_z) {
  DBGOUT(printf("Initialize Master Costmap\n"));

  obsvtn->rolling_window = rolling_window; //TODO:
  obsvtn->min_obstacle_height = min_obstacle_height; //TODO:
  obsvtn->max_obstacle_height = max_obstacle_height; //TODO:
  obsvtn->raytrace_range = raytrace_range; //TODO:

  obsvtn->master_costmap.cell_size = resolution;
  obsvtn->master_costmap.x_dim = x_dim;
  obsvtn->master_costmap.y_dim = y_dim;
  obsvtn->master_costmap.default_value = CMV_NO_INFORMATION; // default_value;

  obsvtn->master_costmap.av_x = robot_x;
  obsvtn->master_costmap.av_y = robot_y;
  obsvtn->master_costmap.av_z = robot_z;

  obsvtn->master_resolution = resolution;

  obsvtn->master_origin.x = robot_x - (x_dim - 1) / 2;
  obsvtn->master_origin.y = robot_y - (y_dim - 1) / 2;
  obsvtn->master_origin.z = robot_z;
  DBGOUT(printf("Master Origin -> <%f, %f, %f>\n", obsvtn->master_origin.x, obsvtn->master_origin.y, obsvtn->master_origin.z));

  CHECK(int chkMaxIdx = obsvtn->master_costmap.x_dim * obsvtn->master_costmap.y_dim / (obsvtn->master_resolution * obsvtn->master_resolution);
	if (chkMaxIdx > COST_MAP_ENTRIES) {
	  printf("ERROR : initCostMap : Max index is too large at %d vs %d\n", chkMaxIdx, COST_MAP_ENTRIES);
	});
  for (int i = 0; i < obsvtn->master_costmap.x_dim * obsvtn->master_costmap.y_dim / (obsvtn->master_resolution * obsvtn->master_resolution); i++) {
    obsvtn->master_costmap.costmap[i] = CMV_NO_INFORMATION; // obsvtn->master_costmap.default_value;
  }

  DBGOUT(printf("Initialize Master Costmap ... DONE\n\n"));
}

/******************* FUNCTIONS *********************/
#undef DBGOUT
#define DBGOUT(x) x
void fuseIntoWorld(Worldmap2D* theWorld, Costmap2D* theInput)
{
  DBGOUT(printf("Entered fuseIntoWorld...\n"));
  if (theWorld->must_init) {
    theWorld->x = theInput->av_x;
    theWorld->y = theInput->av_y;
    theWorld->z = theInput->av_z;
    DBGOUT(printf("Set worldMap x, y, z = %2lf, %2lf, %2lf\n", theWorldMap.x, theWorldMap.y, theWorldMap.z));
    theWorld->must_init = false;
  }
  //worldMap is the persistent map fo world knowledge; inputMap is the new information to enter into the worldMap
  unsigned char* worldMap = theWorld->map;
  unsigned char* inputMap = theInput->costmap;
  // These are the "position" of the (center) of the map
  int world_x_pos = (int)((float)theWorld->x / theWorld->cell_size);
  int world_y_pos = (int)((float)theWorld->y / theWorld->cell_size);
  int input_x_pos = (int)((float)theInput->av_x / theInput->cell_size);
  int input_y_pos = (int)((float)theInput->av_y / theInput->cell_size);
  // These are the dimension (raw input) of the map
  int world_x_dim = theWorld->x_dim;
  int world_y_dim = theWorld->y_dim;
  int input_x_dim = theInput->x_dim;
  int input_y_dim = theInput->y_dim;
  // Thess are the number of costmap/gridmap "cells" (dimensionally)
  int world_x_cells = (world_x_dim / theWorld->cell_size);
  int world_y_cells = (world_y_dim / theWorld->cell_size);
  int input_x_cells = (input_x_dim / theInput->cell_size);
  int input_y_cells = (input_y_dim / theInput->cell_size);
  //int resolution = theWorld->cell_size;
  /* DBGOUT(printf("in fuseIntoWorld: x1 = %d x2 = %d  y1 = %d y2 = %d\n", input_x, world_x, input_y, world_y); */
  /* 	 printf("      num_cells : w_x = %u  w_y = %u   u_x = %u  u_y = %u\n", world_x_cells, world_y_cells, input_x_cells, input_y_cells)); */

  DBGOUT(printf(" World in-data: X-pos %d y_pos %d x_cells %d y_cells %d\n", world_x_pos, world_y_pos, world_x_cells, world_y_cells);
	 printf(" Input in-data: X-pos %d y_pos %d x_cells %d y_cells %d\n", input_x_pos, input_y_pos, input_x_cells, input_y_cells));
  // Express the world/input maps in terms of "Global Cell Coordinates"
  int input_0_x = input_x_pos - input_x_cells/2;
  int input_0_y = input_y_pos - input_x_cells/2;
  int input_N_x = input_x_pos + input_x_cells/2 - 1;
  int input_N_y = input_y_pos + input_x_cells/2 - 1;
  DBGOUT(printf("Input: 0x Nx = %d  %d   :   0y Ny = %d  %d\n", input_0_x, input_N_x, input_0_y, input_N_y));

  int world_0_x = world_x_pos - world_x_cells/2;
  int world_0_y = world_y_pos - world_x_cells/2;
  int world_N_x = world_x_pos + world_x_cells/2 - 1;
  int world_N_y = world_y_pos + world_x_cells/2 - 1;
  DBGOUT(printf("World: 0x Nx = %d  %d   :   0y Ny = %d  %d\n", world_0_x, world_N_x, world_0_y, world_N_y));

  // Now determine the (localized) X and Y cell dimensions that overlap
  //  We compute the min and max X and Y as for a for loop: for (ix = x_min; ix < x_max; ix++) 
  int input_x_min = max(0, (world_0_x - input_0_x));
  DBGOUT(printf("   input_x_min = max(0, (%d - %d) = %d) = %d\n", world_0_x, input_0_x, (world_0_x - input_0_x), input_x_min));
  int input_x_max = input_x_cells - max(0, (input_N_x - world_N_x));
  DBGOUT(printf("   input_x_max = %d - max(0, (%d - %d) = %d) = %d\n", input_x_cells, input_0_x, world_0_x, (input_N_x - world_N_x), input_x_max));
  int input_y_min = max(0, (world_0_y - input_0_y));
  DBGOUT(printf("   input_y_min = max(0, (%d - %d) = %d) = %d\n", world_0_y, input_0_y, (world_0_y - input_0_y), input_y_min));
  int input_y_max = input_y_cells - max(0, (input_N_y - world_N_y));
  DBGOUT(printf("   input_y_max = %d - max(0, (%d - %d) = %d) = %d\n", input_y_cells, input_0_y, world_0_y, (input_N_y - world_N_y), input_y_max));
  DBGOUT(printf("Input: x_min %d  x_max %d  y_min %d  y_max %d\n", input_x_min, input_x_max, input_y_min, input_y_max));

  int world_x_min = max(0, (input_0_x - world_0_x));
  int world_x_max = world_x_cells - max(0, (world_N_x - input_N_x));
  int world_y_min = max(0, (world_0_y, input_0_y));
  int world_y_max = world_y_cells - max(0, (world_N_y - input_N_y));
  DBGOUT(printf("World: x_min %d  x_max %d  y_min %d  y_max %d\n", world_x_min, world_x_max, world_y_min, world_y_max));

  //Iterate through grids and assign corresponding max value
  int iiy = input_y_min;
  for (int wiy = world_y_min; wiy < world_y_max; wiy++) {
    int wbase = wiy*world_x_cells;
    int ibase = iiy*input_x_cells;
    int iix = input_x_min;
    for (int wix = world_x_min; wix < world_x_max; wix++) {
      int world_idx = wbase + wix;
      int input_idx = ibase + iix;
      CHECK(if ((world_idx < 0) || (world_idx >= WORLD_MAP_ENTRIES)) {
      	  printf("ERROR : fuseIntoWorld world_idx outside bounds at %d vs 0 .. %d\n", world_idx, WORLD_MAP_ENTRIES);
      	});
      CHECK(if ((input_idx < 0) || (input_idx >= COST_MAP_ENTRIES)) {
      	  printf("ERROR : fuseIntoWorld input_idx outside bounds at %d vs 0 .. %d\n", input_idx, COST_MAP_ENTRIES);
      	});
      //DBGOUT(printf("  Setting worldMap[%d] = max(%u, %u = inputMap[%d])\n", world_idx, worldMap[world_idx], inputMap[input_idx], input_idx));
      worldMap[world_idx] = max(worldMap[world_idx], inputMap[input_idx]);
      iix++;
    }
    iiy++;
  }
  return;
}
#if(0)

void fuseIntoWorld(Worldmap2D* theWorld, Costmap2D* theUpdate)
{
  unsigned char* worldMap = theWorld->map;
  unsigned char* updateMap = theUpdate->costmap;
  double world_x = theWorld->x;
  double world_y = theWorld->y;
  double update_x = theUpdate->av_x;
  double update_y = theUpdate->av_y;
  unsigned int world_x_dim = theWorld->x_dim;
  unsigned int world_y_dim = theWorld->y_dim;
  unsigned int update_x_dim = theUpdate->x_dim;
  unsigned int update_y_dim = theUpdate->y_dim;
  double resolution = theWorld->cell_size;
  DBGOUT(printf("in fuseIntoWorld: x1 = %.1f x2 = %.1f  y1 = %.1f y2 = %.1f\n", update_x, world_x, update_y, world_y);
	 printf("     dimensions: w_x = %u  w_y = %u   u_x = %u  u_y = %u\n", world_x_dim, world_y_dim, update_x_dim, update_y_dim));
  //updateMap is previous map, world is current map

  //Calculate the old origin of the map
  double origin_x = update_x - (update_x_dim - 1) / 2;
  double origin_y = update_y - (update_y_dim - 1) / 2;
  DBGOUT(printf("origin_x = %.1f - (%d / 2) = %.1f\n", update_x, update_x_dim, origin_x);
	 printf("origin_y = %.1f - (%d / 2) = %.1f\n", update_y, update_y_dim, origin_y));
  //Calculate the new origin of the map
  double new_origin_x = world_x - (world_x_dim - 1) / 2.0;
  double new_origin_y = world_y - (world_y_dim - 1) / 2.0;
  DBGOUT(printf("new_origin_x = %.1f - (%d / 2) = %.1f\n", world_x, world_x_dim, new_origin_x);
	 printf("new_origin_y = %.1f - (%d / 2) = %.1f\n", world_y, world_y_dim, new_origin_y));

  //Calculate the number of cells between the old and new origin
  int cell_ox = ((new_origin_x - origin_x) / resolution);
  int cell_oy = ((new_origin_y - origin_y) / resolution);
  DBGOUT(printf("cell_ox = (%.1f - %.1f) / %.1f = %.1f = %d\n", new_origin_x, origin_x, resolution, ((new_origin_x - origin_x) / resolution), cell_ox);
	 printf("cell_oy = (%.1f - %.1f) / %.1f = %.1f = %d\n", new_origin_y, origin_y, resolution, ((new_origin_y - origin_y) / resolution), cell_oy));
  // Determine the dimensions (x, y) in terms of Grid cells
  int cell_world_x_dim = (int)(world_x_dim / resolution);
  int cell_world_y_dim = (int)(world_y_dim / resolution);
  DBGOUT(printf("cell_world_x_dim = %d / %.f = %d\n", world_x_dim, resolution, cell_world_x_dim);
	 printf("cell_world_y_dim = %d / %.f = %d\n", world_y_dim, resolution, cell_world_y_dim));
    
  //Determine the lower left cells of the origin
  int max_ux = max(cell_ox, 0);
  int max_uy = max(cell_oy, 0);
  int update_lower_left_x = min(max_ux, cell_world_x_dim);
  int update_lower_left_y = min(max_uy, cell_world_y_dim);
  DBGOUT(printf("update_lower_left_x = min(max(%d, 0) = %d, %d) = %d\n", cell_ox, max_ux, cell_world_x_dim, update_lower_left_x);
	 printf("update_lower_left_y = min(max(%d, 0) = %d, %d) = %d\n", cell_oy, max_uy, cell_world_y_dim, update_lower_left_y));
  int world_lower_left_x = update_lower_left_x - cell_ox;
  int world_lower_left_y = update_lower_left_y - cell_oy;
  DBGOUT(printf("world_lower_left_x = %d - %d = %d\n", update_lower_left_x, cell_ox, world_lower_left_x);
	 printf("world_lower_left_y = %d - %d = %d\n", update_lower_left_y, cell_oy, world_lower_left_y));

  //Calculate the indexes of which to start 'copying' over
  int update_index = (update_lower_left_y * cell_world_x_dim + update_lower_left_x);
  int world_index = (world_lower_left_y * cell_world_x_dim + world_lower_left_x);
  DBGOUT(printf("update_index = %d * %d + %d = %d\n", update_lower_left_y, cell_world_x_dim, update_lower_left_x, update_index);
	 printf("world_index = %d * %d + %d = %d\n", world_lower_left_y, cell_world_x_dim, world_lower_left_x, world_index));

  //The size of the overlapping region
  int region_world_x_dim = cell_world_x_dim - cell_ox;
  int region_world_y_dim = cell_world_y_dim - cell_oy;
  DBGOUT(printf("region_world_x_dim = %d - %d = %d\n", cell_world_x_dim, cell_ox, region_world_x_dim);
	 printf("region_world_y_dim = %d - %d = %d\n", cell_world_y_dim, cell_oy, region_world_y_dim));

  /*DBGOUT(printf("origin : %.1lf %.1lf   new_origin: %.1lf %.1lf\n", origin_x, origin_y, new_origin_x, new_origin_y);
	 printf("cell : ox,y %d %d  dimx,y %d %d\n", cell_ox, cell_oy, cell_world_x_dim, cell_world_y_dim);
	 printf("Lower Left: Update (%d, %d)  World(%d, %d)\n", update_lower_left_x, update_lower_left_y, world_lower_left_x, world_lower_left_y);
	 //printf("Lower Left of New Map = (%d, %d) \n", world_lower_left_x, world_lower_left_y);
	 printf("Index of Old Map, Index of New Map = %d, %d \n", update_index, world_index);
	 printf("Dimensions of Overlapping Region = (%d, %d) \n", region_world_x_dim, region_world_y_dim)); */
  //Iterate through grids and assign corresponding max value
  unsigned int total_count = 0;
  unsigned int count = 0;
  for (int i = 0; i < cell_world_x_dim; i++) {
    for (int j = 0; j < cell_world_y_dim; j++) {
      if (update_index == cell_world_x_dim * cell_world_y_dim) return;
      if (count == region_world_x_dim) {
	update_index = update_index + cell_ox;
	world_index = world_index + cell_ox;
	count = 0;
      }
      CHECK(if ((world_index < 0) || (world_index >= WORLD_MAP_ENTRIES)) {
	  printf("ERROR : fuseIntoWorld world_index too large at %d vs %d\n", world_index, WORLD_MAP_ENTRIES);
	});
      CHECK(if ((update_index < 0) || (update_index >= COST_MAP_ENTRIES)) {
	  printf("ERROR : fuseIntoWorld update_index too large at %d vs %d\n", update_index, COST_MAP_ENTRIES);
	});

      worldMap[world_index] = max(worldMap[world_index], updateMap[update_index]);
      //DBGOUT(printf("%d : %d v %d : %d, %d \n", total_count, count, region_world_x_dim, update_index, world_index));
      update_index++;
      world_index++;
      count++;
      total_count++;
    }
  }
  return;
}

void fuseIntoWorld(Worldmap2D* theWorld, Costmap2D* theUpdate)
//                unsigned char* grid1, unsigned char* grid2,
//		  double robot_x1, double robot_y1,
//		  double robot_x2, double robot_y2,
//		  unsigned int x_dim, unsigned int y_dim, double resolution
//		  /*,char def_val*/ )
{
  unsigned char* worldMap = theWorld->map;
  unsigned char* updateMap = theUpdate->costmap;
  double world_x = theWorld->x;
  double world_y = theWorld->y;
  double update_x = theUpdate->av_x;
  double update_y = theUpdate->av_y;
  unsigned int world_x_dim = theWorld->x_dim;
  unsigned int world_y_dim = theWorld->y_dim;
  unsigned int update_x_dim = theUpdate->x_dim;
  unsigned int update_y_dim = theUpdate->y_dim;
  double resolution = theWorld->cell_size;
    
  DBGOUT(printf("in fuseIntoWorld: x1 = %.1f x2 = %.1f  y1 = %.1f y2 = %.1f\n", world_x, update_x, world_y, update_y);
	 printf("  dims:  world: x = %u  y = %u   update: x = %u  y = %u\n", world_x_dim, world_y_dim, update_x_dim, update_y_dim));
  //worldMap is previous map, updateMap is current map

  //Calculate the old origin of the map
  double origin_x = world_x - (world_x_dim - 1) / 2;
  double origin_y = world_y - (world_y_dim - 1) / 2;
	   
  //Calculate the new origin of the map
  double new_origin_x = update_x - (world_x_dim - 1) / 2.0;
  double new_origin_y = update_y - (world_y_dim - 1) / 2.0;

  //Calculate the number of cells between the old and new origin
  int cell_ox = ((new_origin_x - origin_x) / resolution);
  int cell_oy = ((new_origin_y - origin_y) / resolution);
  DBGOUT(printf("cell_ox = (%.1f - %.1f) / %.1f = %.1f = %d\n", new_origin_x, origin_x, resolution, ((new_origin_x - origin_x) / resolution), cell_ox);
	 printf("cell_oy = (%.1f - %.1f) / %.1f = %.1f = %d\n", new_origin_y, origin_y, resolution, ((new_origin_y - origin_y) / resolution), cell_oy));
  // Determine the dimensions (x, y) in terms of Grid cells
  int cell_world_x_dim = (int)(world_x_dim / resolution);
  int cell_world_y_dim = (int)(world_y_dim / resolution);
  int cell_update_x_dim = (int)(update_x_dim / resolution);
  int cell_update_y_dim = (int)(update_y_dim / resolution);
    
  //Determine the lower left cells of the origin
  int world_lower_left_x = min(max(cell_ox, 0), cell_world_x_dim);
  int world_lower_left_y = min(max(cell_oy, 0), cell_world_y_dim);
  int update_lower_left_x = world_lower_left_x - cell_ox;
  int update_lower_left_y = world_lower_left_y - cell_oy;

  //Calculate the indexes of which to start 'copying' over
  int world_index = (world_lower_left_y * cell_world_x_dim + world_lower_left_x);
  int update_index = (update_lower_left_y * cell_update_x_dim + update_lower_left_x);

  //The size of the overlapping region
  int region_world_x_dim = cell_world_x_dim - cell_ox;
  int region_world_y_dim = cell_world_y_dim - cell_oy;
  int region_update_x_dim = cell_update_x_dim - cell_ox;
  int region_update_y_dim = cell_update_y_dim - cell_oy;

  DBGOUT(printf("origin : %.1lf %.1lf   new_origin: %.1lf %.1lf\n", origin_x, origin_y, new_origin_x, new_origin_y);
	 printf("cell : ox,y %d %d  dimx,y %d %d\n", cell_ox, cell_oy, cell_world_x_dim, cell_world_y_dim);
	 printf("Lower Left: Old (%d, %d)  New (%d, %d)\n", world_lower_left_x, world_lower_left_y, update_lower_left_x, update_lower_left_y);
	 //printf("Lower Left of New Map = (%d, %d) \n", update_lower_left_x, update_lower_left_y);
	 printf("Index of Old Map, Index of New Map = %d, %d \n", world_index, update_index);
	 printf("Dimensions of Overlapping Region = (%d, %d) \n", region_world_x_dim, region_world_y_dim));
  //Iterate through grids and assign corresponding max value
  unsigned int total_count = 0;
  unsigned int count = 0;
  for (int i = 0; i < cell_world_x_dim; i++) {
    for (int j = 0; j < cell_world_y_dim; j++) {
      //DBGOUT(printf(" i %u j %u world_idx %u upd_idx %u cw_dim %u count %u rw_x_dim %u\n", i, j, world_index, update_index, (cell_world_x_dim*cell_world_y_dim), count, region_world_x_dim));
      if (world_index == cell_world_x_dim * cell_world_y_dim) return;
      if (count == region_update_x_dim) {
	world_index = world_index + cell_ox;
	update_index = update_index + cell_ox;
	count = 0;
      }
      CHECK(if ((update_index < 0) || (update_index >= COST_MAP_ENTRIES)) {
	  printf("ERROR : fuseIntoWorld update_index too large at %d vs %d\n", update_index, COST_MAP_ENTRIES);
	});
      CHECK(if ((world_index < 0) || (world_index >= WORLD_MAP_ENTRIES)) {
	  printf("ERROR : fuseIntoWorld world_index too large at %d vs %d\n", world_index, WORLD_MAP_ENTRIES);
	});

      worldMap[world_index] = max(worldMap[update_index], updateMap[world_index]);
      //DBGOUT(printf("%d : %d v %d : %d, %d \n", total_count, count, region_world_x_dim, world_index, update_index));
      world_index++;
      update_index++;
      count++;
      total_count++;
    }
  }
  return;
}
#endif
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
  int g1_lower_left_x = min(max(cell_ox, 0), cell_x_dim);
  int g1_lower_left_y = min(max(cell_oy, 0), cell_y_dim);
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
	 printf("Dimensions of Overlapping Region = (%d, %d) \n", region_x_dim, region_y_dim));
  //Iterate through grids and assign corresponding max value
  unsigned int total_count = 0;
  unsigned int count = 0;
  for (int i = 0; i < cell_x_dim; i++) {
    for (int j = 0; j < cell_y_dim; j++) {
      if (g1_index == cell_x_dim * cell_y_dim) return;
      if (count == region_x_dim) {
	g1_index = g1_index + cell_ox;
	g2_index = g2_index + cell_ox;
	count = 0;
      }
      CHECK(if ((g2_index < 0) || (g2_index >= COST_MAP_ENTRIES)) {
	  printf("ERROR : combineGrids g2_index too large at %d vs %d\n", g2_index, COST_MAP_ENTRIES);
	});
      CHECK(if ((g1_index < 0) || (g1_index >= COST_MAP_ENTRIES)) {
	  printf("ERROR : combineGrids g1_index too large at %d vs %d\n", g1_index, COST_MAP_ENTRIES);
	});

      grid2[g2_index] = max(grid2[g2_index], grid1[g1_index]);
      //DBGOUT(printf("%d : %d v %d : %d, %d \n", total_count, count, region_x_dim, g1_index, g2_index));
      g1_index++;
      g2_index++;
      count++;
      total_count++;
    }
  }
  return;
}

#undef DBGOUT
#define DBGOUT(x)

unsigned char* cloudToOccgrid(float* data, unsigned int data_size,
			      double robot_x, double robot_y, double robot_z, double robot_yaw,
			      bool rolling_window,
			      double min_obstacle_height, double max_obstacle_height,
			      double raytrace_range,
			      unsigned int x_dim, unsigned int y_dim, double resolution/*,
			      unsigned char default_value*/ ) {
  DBGOUT(printf("In cloudToOccgrid with Odometry %.1f %.1f %.f1\n", robot_x, robot_y, robot_z));
  initCostmap(&master_observation, rolling_window, min_obstacle_height, max_obstacle_height, raytrace_range, x_dim, y_dim, resolution, /*default_value,*/ robot_x, robot_y, robot_z);

  //printf("(1) Number of elements : %d ... ", data_size);
  //printf("First Coordinate = <%f, %f>\n", *data, *(data+1));
  updateMap(data, data_size, robot_x, robot_y, robot_z, robot_yaw);

  //printMap();
  return master_observation.master_costmap.costmap;
}

void updateMap(float* data, unsigned int data_size,
	       double robot_x, double robot_y, double robot_z, double robot_yaw) {
  if (master_observation.rolling_window) {
    //printf("\nUpdating Map .... \n");
    //printf("   robot_x = %f, robot_y = %f, robot_yaw = %f \n", robot_x, robot_y, robot_yaw);
    //printf("   Master Origin = (%f, %f)\n", master_observation.master_origin.x, master_observation.master_origin.y);
    double new_origin_x = robot_x - master_observation.master_costmap.x_dim / 2;
    double new_origin_y = robot_y - master_observation.master_costmap.y_dim / 2;
    updateOrigin(new_origin_x, new_origin_y);
  }

  double minx_ = 1e30;
  double miny_ = 1e30;
  double maxx_ = -1e30;
  double maxy_ = -1e30;

  //printf("(1) Number of elements : %d ... ", data_size);
  //printf("First Coordinate = <%f, %f>\n", *data, *(data+1));
  //rotating_window = true; //Comment out if not rolling window

  updateBounds(data, data_size, robot_x, robot_y, robot_z, robot_yaw, minx_, miny_, maxx_, maxy_);
}

void updateOrigin(double new_origin_x, double new_origin_y) {
  //printf("\nUpdating Map Origin\n");
  //printf("New Origin -> <%f, %f>\n ", new_origin_x, new_origin_y);

  //project the new origin into the grid
  int cell_ox, cell_oy;
  //printf("Old Origin = <%f, %f> ... ", master_observation.master_origin.x, master_observation.master_origin.y);
  //printf("New Origin = <%f, %f>\n", new_origin_x, new_origin_y);
  cell_ox = (int) ((new_origin_x - master_observation.master_origin.x) / master_observation.master_resolution);
  cell_oy = (int) ((new_origin_y - master_observation.master_origin.y) / master_observation.master_resolution);
  //printf("New Cell Origin = <%d, %d>\n", cell_ox, cell_oy);

  // compute the associated world coordinates for the origin cell
  // because we want to keep things grid-aligned
  double new_grid_ox, new_grid_oy;
  new_grid_ox = master_observation.master_origin.x + cell_ox * master_observation.master_resolution;
  new_grid_oy = master_observation.master_origin.y + cell_oy * master_observation.master_resolution;
  //printf("New Grid Origin = <%f, %f> (Should be same as new_origin_x and new_origin_y)\n", new_grid_ox, new_grid_oy);

  // To save casting from unsigned int to int a bunch of times
  int x_dim = master_observation.master_costmap.x_dim / master_observation.master_resolution;
  int y_dim = master_observation.master_costmap.y_dim / master_observation.master_resolution;

  // we need to compute the overlap of the new and existing windows
  int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  lower_left_x = min(max(cell_ox, 0), x_dim);
  lower_left_y = min(max(cell_oy, 0), y_dim);
  upper_right_x = min(max(cell_ox + x_dim, 0), x_dim);
  upper_right_y = min(max(cell_oy + y_dim, 0), y_dim);
  //printf("The Corner Coordinates for Window = {%d, %d} {%d, %d}\n", lower_left_x, lower_left_y, upper_right_x, upper_right_y);

  unsigned int cell_x_dim = (upper_right_x - lower_left_x);
  unsigned int cell_y_dim = (upper_right_y - lower_left_y);
  //printf("Cell Sizes from Corner Coordinates = %d, %d\n", cell_x_dim, cell_y_dim);

  // we need a map to store the obstacles in the window temporarily
  //unsigned char* local_map [cell_x_dim * cell_y_dim]; //Uncomment if in C++

  // now we'll set the costmap to be completely unknown if we track unknown space
  //resetMaps();

  // update the origin with the appropriate world coordinates
  master_observation.master_origin.x = new_grid_ox;
  master_observation.master_origin.y = new_grid_oy;

  // compute the starting cell location for copying data back in
  int start_x = lower_left_x - cell_ox;
  int start_y = lower_left_y - cell_oy;
  //printf("lower_left_x - cell_ox = start_x ... (%d) - (%d) = %d\n", lower_left_x, cell_ox, start_x);
  //printf("lower_left_y - cell_oy = start_y ... (%d) - (%d) = %d\n", lower_left_y, cell_oy, start_y);

  // copy the local window in the costmap to the local map
  copyMapRegion(master_observation.master_costmap.costmap, lower_left_x, lower_left_y,
		master_observation.master_costmap.x_dim / master_observation.master_resolution,
		start_x, start_y,
		master_observation.master_costmap.x_dim / master_observation.master_resolution,
		cell_x_dim, cell_y_dim);


  // now we want to copy the overlapping information back into the map, but in its new location
  //copyMapRegion(local_map, 0, 0, cell_x_dim, master_observation.master_costmap.costmap, start_x, start_y, master_observation.master_costmap.x_dim, cell_x_dim, cell_y_dim);
}

//TODO: Modify such that it explicitly copies the data to the destination map
void copyMapRegion(unsigned char* source_map,
		   unsigned int sm_lower_left_x, unsigned int sm_lower_left_y, unsigned int sm_x_dim,
		   unsigned int dm_lower_left_x, unsigned int dm_lower_left_y, unsigned int dm_x_dim,
		   unsigned int region_x_dim,    unsigned int region_y_dim) {
  // we'll first need to compute the starting points for each map
  //printf("CopyMapRegion() Input Parameters: \n ... sm_lower_left_x = %d,\n ... sm_lowerLeft_y = %d,\n ... sm_x_dim = %d,\n ... dm_lower_left_x = %d,\n ... dm_lower_left_y = %d,\n ... dm_x_dim = %d,\n ... regions_x_dim = %d,\n ... region_y_dim = %d\n",sm_lower_left_x, sm_lower_left_y, sm_x_dim, dm_lower_left_x, dm_lower_left_y, dm_x_dim, region_x_dim, region_y_dim);
  unsigned int sm_index = (sm_lower_left_y * sm_x_dim + sm_lower_left_x);
  unsigned int dm_index = (dm_lower_left_y * dm_x_dim + dm_lower_left_x);
  //printf("%c\n", master_observation.master_costmap.default_value);
  //printf("{sm_index = %d, dm_index = %d}\n", sm_index, dm_index);

  unsigned int cell_x_dim = master_observation.master_costmap.x_dim / master_observation.master_resolution;
  unsigned int cell_y_dim = master_observation.master_costmap.y_dim / master_observation.master_resolution;

  //printf("\n Copying Map... \nRegion Size of Map -> <%d, %d>\n", region_x_dim, region_y_dim);
  char local_costmap [cell_x_dim * cell_y_dim];
  for (int i = 0; i < cell_x_dim * cell_y_dim; i++) {
    local_costmap[i] = CMV_NO_INFORMATION; // master_observation.master_costmap.default_value;
    //printf("%d, ", local_costmap[i]);
  }

  // now, we'll copy the source map into the destination map
  for (unsigned int i = 0; i < region_y_dim; ++i){
    for (unsigned int j = 0; j < region_x_dim; j++) {
      //printf("Source Map Value at Index <%d> = %d\n", sm_index, master_observation.master_costmap.costmap[sm_index]);
      CHECK(if (dm_index >= COST_MAP_ENTRIES) {
	  printf("ERROR : copyMapRegion : dm_index is too large at = %d vs %d\n", dm_index, COST_MAP_ENTRIES);
	}
	if (sm_index >= COST_MAP_ENTRIES) {
	  printf("ERROR : copyMapRegion : sm_index is too large at = %d vs %d\n", sm_index, COST_MAP_ENTRIES);
	});
      local_costmap[dm_index] = master_observation.master_costmap.costmap[sm_index];
      //printf("dm_index, sm_index = %d, %d\n", dm_index, sm_index);
      sm_index++;
      dm_index++;
    }
    if (master_observation.master_costmap.x_dim != region_x_dim) {
      sm_index = sm_index + (master_observation.master_costmap.x_dim / master_observation.master_resolution - region_x_dim);
      dm_index = dm_index + (master_observation.master_costmap.x_dim / master_observation.master_resolution - region_x_dim);
    }
    //memcpy(dm_index, sm_index, region_x_dim * sizeof(unsigned char*));
  }

  //printf("We made it!\n");

  CHECK(int chkMaxIdx = cell_x_dim * cell_y_dim;
	if (chkMaxIdx > COST_MAP_ENTRIES) {
	  printf("ERROR : copyMapRegion : Max index is too large at %d vs %d\n", chkMaxIdx, COST_MAP_ENTRIES);
	});
  for (int i = 0; i < cell_x_dim * cell_y_dim; i++) {
    master_observation.master_costmap.costmap[i] = local_costmap[i];
  }
}

void updateBounds(float* data, unsigned int data_size, double robot_x, double robot_y, double robot_z,
                  double robot_yaw, double min_x, double min_y, double max_x, double max_y) {
  //printf("(1) Number of elements : %d ... ", data_size);
  //printf("First Coordinate = <%f, %f>\n", *data, *(data+1));

  //raytrace free space
  raytraceFreespace(data, data_size, min_x, min_y, max_x, max_y, robot_x, robot_y, robot_z, robot_yaw); //TODO: Reconfigure for 'cloud' parameter

  //printf("Number of elements : %d\n", sizeof(cloud.data) / sizeof(cloud.data[0]));

  //Iterate through cloud to register obstacles within costmap
  for(unsigned int i = 0; i < data_size; i = i + 3) { //TODO: Test if sizeof(points) works correctly
    //Only consider points within height boundaries
    if ((data[i + 2] <= master_observation.max_obstacle_height) && (data[i + 2] >= master_observation.min_obstacle_height)) {
      double px = (double) *(data + i);
      double py = (double) *(data + i + 1);
      double pz = (double) *(data + i + 2);

      //if window rotates, then inversely rotate points
      if (rotating_window) {
	px = px*cos(robot_yaw) - py*sin(robot_yaw);
	py = px*sin(robot_yaw) + py*cos(robot_yaw);
      }

      //printf("World Coordinates (wx, wy) = (%f, %f)\n", px, py);
      //printf("Master Origin Coordinate = < %f, %f > \n", master_observation.master_origin.x, master_observation.master_origin.y);

      //printf("Map Coordinates [BEFORE Conversion] -> (%d, %d)\n", master_observation.map_coordinates.x, master_observation.map_coordinates.y);
      worldToMap(px, py, robot_x, robot_y);
      //printf("Map Coordinates (mx, my) = (%d, %d)\n", master_observation.map_coordinates.x, master_observation.map_coordinates.y);

      unsigned int index = getIndex(master_observation.map_coordinates.x, master_observation.map_coordinates.y);
      CHECK(if (index >= COST_MAP_ENTRIES) {
	  printf("ERROR : updateBounds : index is too large at %d vs %d\n", index, COST_MAP_ENTRIES);
	});
      //printf("Index of Obstacle -> %d\n", index);
      master_observation.master_costmap.costmap[index] = CMV_LETHAL_OBSTACLE; //TODO: Test simple test case (char) 255 = '255' ?
      //printf("updateBounds: pre-touch  : min x %lf y %lf  max x %lf y %lf\n", min_x, min_y, max_x, max_y);
      touch(px, py, &min_x, &min_y, &max_x, &max_y);
      //printf("updateBounds: post-touch : min x %lf y %lf  max x %lf y %lf\n", min_x, min_y, max_x, max_y);
    }
  }
}

/* Issues:
   (1) Finding the size of the array for iteration
   (2) Difference between origin_x_ and observation.origin_x
*/
void raytraceFreespace(float* data, unsigned int data_size, double min_x, double min_y, double max_x, double max_y, double robot_x, double robot_y, double robot_z, double robot_yaw) {
  //printf("(1) Number of elements : %d ... ", data_size);
  //printf("First Coordinate = <%f, %f>\n", *data, *(data+1));

  //Retrieve observation origin (i.e. origin of the pointcloud)
  double ox = robot_x;
  double oy = robot_y;
  //printf(">>> Odometry -> <%f, %f>\n", ox, oy);

  // get the map coordinates of the origin of the sensor
  unsigned int x0, y0;
  if (!worldToMap(0, 0, robot_x, robot_y)) {//TODO:
    printf("The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.\n", ox, oy);
    return;
  }
  x0 = master_observation.map_coordinates.x;
  y0 = master_observation.map_coordinates.y;
  //printf(">>> Map Coordinates of the Sensor Origin -> <%d, %d>\n", x0, y0);

  // we can pre-compute the endpoints of the map outside of the inner loop... we'll need these later
  double map_end_x = master_observation.master_origin.x + master_observation.master_costmap.x_dim * master_observation.master_resolution;
  double map_end_y = master_observation.master_origin.y + master_observation.master_costmap.y_dim * master_observation.master_resolution;
  //printf(">>> End of Map Coordinates -> <%f, %f>\n", map_end_x, map_end_y);

  //printf("rayTraceFreespace: pre-touch  : min x %lf y %lf  max x %lf y %lf\n", min_x, min_y, max_x, max_y);
  touch(ox, oy, &min_x, &min_y, &max_x, &max_y);
  //printf("rayTraceFreespace: post-touch : min x %lf y %lf  max x %lf y %lf\n", min_x, min_y, max_x, max_y);

  for (int i = 0; i < data_size; i = i + 3) {
    double wx = (double) *(data + i);
    double wy = (double) *(data + i + 1);
    //printf(">>> World Coordinates of Data Point -> <%f, %f>\n", wx, wy);

    //if window rotates, then inversely rotate points
    if (rotating_window) {
      wx = wx*cos(robot_yaw) - wy*sin(robot_yaw);
      wy = wx*sin(robot_yaw) + wy*cos(robot_yaw);
    }

    // now we also need to make sure that the enpoint we're raytracing
    // to isn't off the costmap and scale if necessary
    double a = wx - ox;
    double b = wy - oy;

    if (wx < master_observation.master_origin.x) {
      double t = (master_observation.master_origin.x - ox) / a;
      wx = master_observation.master_origin.x;
      wy = oy + b * t;
    }

    if (wy < master_observation.master_origin.y) {
      double t = (master_observation.master_origin.y - oy) / b;
      wx = ox + a * t;
      wy = master_observation.master_origin.y;
    }

    // the maximum value to raytrace to is the end of the map
    if (wx > map_end_x)
      {
	double t = (map_end_x - ox) / a;
	wx = map_end_x - .001;
	wy = oy + b * t;
      }
    if (wy > map_end_y)
      {
	double t = (map_end_y - oy) / b;
	wx = ox + a * t;
	wy = map_end_y - .001;
      }

    // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
    unsigned int x1, y1;
    if (!worldToMap(wx, wy, robot_x, robot_y)) continue;
    x1 = master_observation.map_coordinates.x;
    y1 = master_observation.map_coordinates.y;

    unsigned int cell_raytrace_range = cellDistance(master_observation.raytrace_range);
    //printf(">>> Cell Raytrace Range -> %d\n", cell_raytrace_range);

    // and finally... we can execute our trace to clear obstacles along that line
    raytraceLine(x0, y0, x1, y1, cell_raytrace_range);

    updateRaytraceBounds(ox, oy, wx, wy, master_observation.raytrace_range, min_x, min_y, max_x, max_y);
  }
}

bool worldToMap(double wx, double wy, double robot_x, double robot_y) {
  //Calculate world coordinates relative to origin (and not robot's odometry)
  double wx_rel_origin = wx + robot_x;
  double wy_rel_origin = wy + robot_y;
  //printf("World To Map (Relative to Origin) = (%d, %d)\n", (int)((wx_rel_origin - master_observation.master_origin.x) / master_observation.master_resolution), (int)((wy_rel_origin - master_observation.master_origin.y) / master_observation.master_resolution));
  if ((wx_rel_origin < master_observation.master_origin.x) || (wy_rel_origin < master_observation.master_origin.y)) {
    DBGOUT(printf("Coordinates Out Of Bounds .... (wx, wy) = (%f, %f); (ox, oy) = (%f, %f)\n", wx, wy, master_observation.master_origin.x, master_observation.master_origin.y));
    return false;
  }

  master_observation.map_coordinates.x = (int)((wx_rel_origin - master_observation.master_origin.x) / master_observation.master_resolution);
  master_observation.map_coordinates.y = (int)((wy_rel_origin - master_observation.master_origin.y) / master_observation.master_resolution);

  //printf("World To Map (wx, wy) = (%f, %f) -> (mx, my) = (%d, %d)\n\n", wx, wy, master_observation.map_coordinates.x, master_observation.map_coordinates.y);

  if ((master_observation.map_coordinates.x < master_observation.master_costmap.x_dim) && (master_observation.map_coordinates.y < master_observation.master_costmap.y_dim)) {
    return true;
  }
  return false;
}

unsigned int cellDistance(double world_dist) {
  double cells_dist = max (0.0, ceil(world_dist/master_observation.master_resolution));
  return (unsigned int) cells_dist;
}

void raytraceLine(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1, unsigned int max_length) { //TODO: default parameter for max_length is UNIT_MAX; define UNIT_MAX
  //printf(">>> Raytrace Line from <%d, %d> to <%d, %d> \n", x0, y0, x1, y1);
  int dx = x1 - x0;
  int dy = y1 - y0;
  //printf("dx, dy -> %d ,%d\n", dx, dy);

  unsigned int abs_dx = abs(dx);
  unsigned int abs_dy = abs(dy);

  int offset_dx = sign(dx);
  //printf("offset_dx -> %d, \n", offset_dx);
  //printf("cell_x_dim -> %d \n", (int) (master_observation.master_costmap.x_dim / master_observation.master_resolution));
  int offset_dy = sign(dy) * (int) (master_observation.master_costmap.x_dim / master_observation.master_resolution);
  //printf("offset_dy -> %d \n", offset_dy);

  unsigned int offset = y0 * master_observation.master_costmap.x_dim / master_observation.master_resolution + x0;
  //printf("offset -> %d \n", offset);

  // we need to chose how much to scale our dominant dimension, based on the maximum length of the line
  double dist = hypot(dx, dy);
  double scale = (dist == 0.0) ? 1.0 : min(1.0, max_length / dist);

  // if x is dominant
  if (abs_dx >= abs_dy) {
    int error_y = abs_dx / 2;
    bresenham2D(abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx));
    return;
  }

  // otherwise y is dominant
  int error_x = abs_dy / 2;
  bresenham2D(abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy));
}

void bresenham2D(unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
		 int offset_b, unsigned int offset, unsigned int max_length) {
  unsigned int end = min(max_length, abs_da);
  //printf("\n\n abs_da, end -> %d, %d\n", abs_da, end);
  for (unsigned int i = 0; i < end; ++i)
    {
      markCell(CMV_FREE_SPACE, offset);
      offset += offset_a;
      error_b += abs_db;
      if ((unsigned int)error_b >= abs_da)
        {
	  offset += offset_b;
	  error_b -= abs_da;
        }
    }
  markCell(CMV_FREE_SPACE, offset);
}

void markCell(unsigned char value, unsigned int offset) {
  //printf("OFFSET -> %d\n", offset);
  CHECK(if (offset >= COST_MAP_ENTRIES) {
      printf("ERROR : updateBounds : offset is too large at %d vs %d\n", offset, COST_MAP_ENTRIES);
    });
  master_observation.master_costmap.costmap[offset] = value;
}

void updateRaytraceBounds(double ox, double oy, double wx, double wy, double range,
                          double min_x, double min_y, double max_x, double max_y) {
  double dx = wx - ox, dy = wy - oy;
  double full_distance = hypot(dx, dy);
  double scale = min(1.0, range / full_distance);
  double ex = ox + dx * scale, ey = oy + dy * scale;
  //printf("updateRaytraceBounds: pre-touch  : min x %lf y %lf  max x %lf y %lf\n", min_x, min_y, max_x, max_y);
  touch(ex, ey, &min_x, &min_y, &max_x, &max_y);
  //printf("updateRaytraceBounds: post-touch : min x %lf y %lf  max x %lf y %lf\n", min_x, min_y, max_x, max_y);
}




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

  int error = 0;
  if (WORLD_GRID_X_DIM < GRID_MAP_X_DIM) {
    printf("ERROR : WORLD_GRID_X_DIM = %u  < %u = GRID_MAP_X_DIM\n", WORLD_GRID_X_DIM, GRID_MAP_X_DIM);
    error++;
  }
  if (WORLD_GRID_Y_DIM < GRID_MAP_Y_DIM) {
    printf("ERROR : WORLD_GRID_Y_DIM = %u  < %u = GRID_MAP_Y_DIM\n", WORLD_GRID_Y_DIM, GRID_MAP_Y_DIM);
    error++;
  }
  int pop_x_count = 0;
  int pop_y_count = 0;
  for (int ii = 0; ii < 32; ii++) {
    if (((WORLD_MAP_X_DIM >> ii)&0x1) == 1) {
      pop_x_count++;
    }
    if (((WORLD_MAP_X_DIM >> ii)&0x1) == 1) {
      pop_y_count++;
    }
  }
  if (pop_x_count > 1) {
    printf("ERROR : WORLD_GRID_X_DIM id %u - must be power-of-2 >= 2\n", WORLD_GRID_X_DIM);
    error++;
  }
  if (pop_y_count > 1) {
    printf("ERROR : WORLD_GRID_Y_DIM id %u - must be power-of-2 >= 2\n", WORLD_GRID_Y_DIM);
    error++;
  }
  // Initialize thge entire world-map to the default value.
  // We could set these here; I am going to take the first local odometry and use that...
  //  theWorldMap.av_x = 0;
  //  theWorldMap.av_y = 0;
  theWorldMap.x_dim = WORLD_GRID_X_DIM;
  theWorldMap.y_dim = WORLD_GRID_Y_DIM;
  theWorldMap.cell_size = GRID_MAP_RESLTN;
  theWorldMap.must_init = true;
  for (int i = 0; i < WORLD_MAP_ENTRIES; i++) {
    theWorldMap.map[i] = CMV_NO_INFORMATION;
  }

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
      fprintf(fptr, "%c", pr_map_char[cmap->costmap[idx]]);
    }
    fprintf(fptr, " | %3u\n  ", ii);
  }
  fprintf(fptr, "\n");
}

void print_ascii_worldmap(FILE*  fptr, Worldmap2D* wmap)
{
  fprintf(fptr, "  ");
  unsigned h1 = 0;
  unsigned h10 = 0;
  for (int ii = 0; ii < WORLD_MAP_X_DIM; ii++) {
    fprintf(fptr, "%u", h10);
    h1++;
    if (h1 == 10) { h1 = 0; h10++; }
    if (h10 == 10) { h10 = 0;}
  }
  fprintf(fptr, "\n  ");
  for (int ii = 0; ii < WORLD_MAP_X_DIM; ii++) {
    fprintf(fptr, "%u", ii%10);
  }
  fprintf(fptr, "\n  ");
  for (int ii = 0; ii < WORLD_MAP_X_DIM; ii++) {
    fprintf(fptr, "-");
  }
  fprintf(fptr, "\n  ");
  for (int ii = 0; ii < WORLD_MAP_X_DIM; ii++) {
    for (int ij = 0; ij < WORLD_MAP_Y_DIM; ij++) {
      int idx = WORLD_MAP_X_DIM*ii + ij;
      fprintf(fptr, "%c", pr_map_char[wmap->map[idx]]);
    }
    fprintf(fptr, " | %3u\n  ", ii);
  }
  fprintf(fptr, "\n");
}

