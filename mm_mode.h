#ifndef MM_MODE_H
#define MM_MODE_H

#include "gcode.h"

typedef struct {
  uint8_t units;
} to_mm_state_t, from_mm_state_t;

// converts all movements to mm
// removes all g20/g21 commands that don't change the state, assuming
// that the machine is in g21 mode in the beginning
// converts all g20 commands into g21 commands
int to_mm_init(to_mm_state_t *state);
int to_mm(to_mm_state_t *state, parser_block_t *block);

// spits out a g21 command at the beginning, unless the first
// block contains a g20 command
// for every g21 command received, toggles between g21 and g20
// converts all movements to inches when g20 is used
int from_mm_init(from_mm_state_t *state);
int from_mm(from_mm_state_t *state, parser_block_t *block);

#endif
