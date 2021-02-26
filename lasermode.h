#ifndef LASERMODE_H
#define LASERMODE_H

#include "gcode.h"

typedef struct {
  gc_modal_t modal;
  gc_values_t values;
  float a;
  float v[2];
  float M;    // M = acos(maxangle)^2
} laser_state_t;


  // state: uninitialized
// a: Accelleration in mm / s2
// max_angle_deg: Maximum angle between two lines which will not cause a stop

void lasermode_init(laser_state_t *state, double a, double max_angle_deg);

// state: must be inited with lasermode_init
// block must called with one block, but must have room for 4 parser_block_t
// return value: Number of blocks returned
// adds extra moves so that laser can move at nominal speed when it is on

int lasermode(laser_state_t *state,
	       parser_block_t *block);


#endif
