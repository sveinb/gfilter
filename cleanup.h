#ifndef CLEANUP_H
#define CLEANUP_H

#include "gcode.h"

// cleans up unneccesary words

typedef struct {
  gc_modal_t modal;
  gc_values_t values;
} cleanup_state_t;

void cleanup_init(cleanup_state_t *state);
int cleanup(cleanup_state_t *state,
	       parser_block_t *block);


#endif
