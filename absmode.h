#ifndef ABSMODE_H
#define ABSMODE_H

#include "gcode.h"

typedef struct {
  float xyz[3];
  uint8_t distance;
} toabs_state_t, fromabs_state_t;

// converts all movements to absolute coordinates
// removes all g91/g90 commands that don't change the state, assuming
// that the machine is in g90 mode in the beginning
// converts all g91 commands into g90 commands
int toabs_init(toabs_state_t *state);
int toabs(toabs_state_t *state, parser_block_t *block);

// spits out a g90 command at the beginning, unless the first
// block contains a g90 command
// for every g90 command received, toggles between g90 and g91
// converts all movements to relative when g91 is used
int fromabs_init(fromabs_state_t *state);
int fromabs(fromabs_state_t *state, parser_block_t *block);

#endif
