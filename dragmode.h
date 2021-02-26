#ifndef DRAGMODE_H
#define DRAGMODE_H

#include "gcode.h"

typedef struct {
  gc_modal_t modal;
  gc_values_t values;
  float v[2]; // unit vector from machine position to blade tip
  float d;
  float angle0;
  float cosminangle;
} drag_state_t;

// d is blade offset
// angle0 is the direction the blade is oriented in initially
// 0 = pointing towards +x, 90 = pointing towards +y
// minangle: Minimum angle between two line segments that leads to a
// swivel action (degrees)
void dragmode_init(drag_state_t *state, float d, float angle0, float minangle);
int dragmode(drag_state_t *state,
	     parser_block_t *block);


#endif
