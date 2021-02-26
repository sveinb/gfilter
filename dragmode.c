#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "nuts_bolts.h"
#include "dragmode.h"
#include "geom.h"

void dragmode_init(drag_state_t *state, float d, float angle0, float minangle) {
  memset(&state->modal, 0, sizeof(state->modal));
  memset(&state->values, 0, sizeof(state->values));
  state->angle0 = angle0;
  state->d = d;
  // state->v is the vector from blade tip to swivel center
  state->v[0] = cos(angle0 * 3.141 / 180);
  state->v[1] = sin(angle0 * 3.141 / 180);
  // state->values.xyz is the location of the blade tip
  state->values.xyz[0] = -state->v[0] * d;
  state->values.xyz[1] = -state->v[1] * d;
  // machine coordinates are 0,0
  state->cosminangle = cos(minangle / 180 * 3.141);
}

int dragmode(drag_state_t *state,
	       parser_block_t *block) {
  int retval = 1;

  block->command_words &= ~bit(MODAL_GROUP_M7); // no spindle action
  
  drag_state_t oldstate = *state;

  update_state(&state->modal, &state->values, block);
  // state represents the desired knife tip location after block
  // oldstate represents the knife tip location before block
  
  normarcs(block, state->modal.motion, 
	   state->values.xyz[0]-oldstate.values.xyz[0], // Delta x between current position and target
	   state->values.xyz[1]-oldstate.values.xyz[1]); // Delta y between current position and target
  
  
  // oldstate.v: direction at end of last block
  // v0: direction at beginning of this block
  // state->v: direction at end of this block
  
  float v0[2]; 

  float dx = state->values.xyz[0] - oldstate.values.xyz[0];
  float dy = state->values.xyz[1] - oldstate.values.xyz[1];

  calcv(block, state->modal.motion, dx, dy, v0, state->v);
  if (state->values.xyz[2] >= 0 || oldstate.values.xyz[2] >= 0) // not cutting, knife must continue pointing in old direction
    memcpy(state->v, oldstate.v, sizeof(float) * 2);

  float xy0[2] = {
		   oldstate.values.xyz[0] + oldstate.v[0] * oldstate.d,
		   oldstate.values.xyz[1] + oldstate.v[1] * oldstate.d
  }; // machine coordinates before block

  for (int i = 0; i < 2; i++)
    block->values.xyz[i] = state->values.xyz[i] + state->v[i] * state->d;

  if (block->value_words & bit(WORD_R)) {
    block->values.r = sqrt(block->values.r * block->values.r + state->d * state->d);
  } else if (block->value_words & (bit(WORD_I) | bit(WORD_J))) {
    if ((block->value_words & bit(WORD_I)) == 0)
      block->values.ijk[0] = 0;
    if ((block->value_words & bit(WORD_J)) == 0)
      block->values.ijk[1] = 0;
    block->value_words |= bit(WORD_I) | bit(WORD_J);
    for (int i = 0; i < 2; i++)
      block->values.ijk[i] -= oldstate.v[i] * oldstate.d;
  }

  block->value_words |= bit(WORD_X) | bit(WORD_Y);
  // machine coordinates after block

  float dp = v0[0] * oldstate.v[0] + v0[1] * oldstate.v[1];

  if (dp < state->cosminangle && state->values.xyz[2] < 0 && oldstate.values.xyz[2] < 0) { // there is a discontinuity of direction at start of this move
    // and the knife is in the material
    // must create arc
    // move original move to make space for curve
    memcpy(block + 1, block, sizeof(*block));
    float dir = v0[0] * oldstate.v[1] - v0[1] * oldstate.v[0];
    if (dir > 0)
      block[0].modal.motion = MOTION_MODE_CW_ARC;
    else
      block[0].modal.motion = MOTION_MODE_CCW_ARC;
    block[0].command_words = bit(MODAL_GROUP_G1);
    // machine coordinates at beginning of this move
    block[0].values.xyz[0] = oldstate.values.xyz[0] + v0[0] * state->d;
    block[0].values.xyz[1] = oldstate.values.xyz[1] + v0[1] * state->d;
    block[0].values.r = state->d;
    block[0].value_words = bit(WORD_R) | bit(WORD_X) | bit(WORD_Y);
    block[1].modal.motion = state->modal.motion;
    block[1].command_words |= bit(MODAL_GROUP_G1);
    retval++;
  }
  
  return retval;
}

