#include "lasermode.h"
#include "geom.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

// *block has room for up to three more blocks after it, which may be
// populated by this function. They should be executed in reverse order.
// return value: number of blocks to process in *block

void lasermode_init(laser_state_t *state, double a, double max_angle_deg) {
  memset(&state->modal, 0, sizeof(state->modal));
  memset(&state->values, 0, sizeof(state->values));
  state->a = a;
  state->M = acos(max_angle_deg / 180. * 3.14);
  state->M = state->M * state->M;
}


int lasermode(laser_state_t *state,
	       parser_block_t *block) {

  laser_state_t oldstate = *state;

  update_state(&state->modal, &state->values, block);

  float dx = state->values.xyz[0]-oldstate.values.xyz[0];
  float dy = state->values.xyz[1]-oldstate.values.xyz[1];

  float v0[2];
  
  normarcs(block, state->modal.motion, 
	   dx, // Delta x between current position and target
	   dy); // Delta y between current position and target

  calcv(block, state->modal.motion, dx, dy, v0, state->v);
    

    float dv2 = 0;
    for (int i = 0; i < 2; i++)
      dv2 += v0[i] * oldstate.v[i];

    bool extprev = false;
    bool extnext = false;


    if (dv2 < state->M || state->values.f != oldstate.values.f ||
	((state->values.s == 0) != (oldstate.values.s == 0)) ||
	state->modal.spindle != oldstate.modal.spindle) {
      extprev = oldstate.values.s != 0 &&
	oldstate.modal.spindle != SPINDLE_DISABLE &&
	oldstate.modal.motion != MOTION_MODE_SEEK;
      extnext = state->values.s != 0 &&
	state->modal.spindle != SPINDLE_DISABLE &&
	state->modal.motion != MOTION_MODE_SEEK;
    }

    //    printf("extprev = %d extnext = %d @ %g %g\n", extprev, extnext, oldstate.values.xyz[0], oldstate.values.xyz[1]);

    int retval = 1;
    if (extprev)
      retval += 2;
    if (extnext)
      retval += 2;
    if (extprev && extnext)
      retval --;
    
    if (retval > 1) {
      memcpy(block + retval - 1, block, sizeof(block[0]));
      block[retval-1].value_words |= bit(WORD_S);
      block[retval-1].values.s = state->values.s;
      block[retval-1].command_words |= bit(MODAL_GROUP_G1);
      block[retval-1].modal.motion = state->modal.motion;
    }

    float x1[2]; // extension of end of previous leg
    float x2[2]; // extension of start of next leg
    float x2b[2]; // extension of start of next leg, close to x2
    
    if (extprev) {
      // extend previous leg
      // v^2 = 2 as
      float d;
      d = oldstate.values.f / 60.f; // mm / s
      d = d * d;
      d = d / 2. / oldstate.a;
      
      for (int i = 0; i < 2; i++)
	x1[i] = oldstate.values.xyz[i] + d * oldstate.v[i];
    }

    if (extnext) {
      float d;
      d = state->values.f / 60.f; // mm / s
      d = d * d;
      d = d / 2 / state->a;
      
      for (int i = 0; i < 2; i++)
	x2[i] = oldstate.values.xyz[i] - d * v0[i];
    }

    float *curpos = oldstate.values.xyz;
    parser_block_t *curblock = &block[0];
    
    if (extprev) { // move to the extension of the previous segment
      //      curblock->command_words = 0;
      curblock->value_words = bit(WORD_X) | bit(WORD_Y) | bit(WORD_S) | (curblock->value_words & bit(WORD_F));
      curblock->values.s = 0;
      curblock->modal.motion = MOTION_MODE_LINEAR;
      curblock->command_words = bit(MODAL_GROUP_G1);
      for (int i = 0; i < 2; i++)
	curblock->values.xyz[i] = x1[i];
	
      curpos = x1;
      curblock++;
    }

    if (extnext) { // move to the extension of the next segment
      //      curblock->command_words = 0;
      memcpy(curblock, block, sizeof(block[0]));

      curblock->value_words = bit(WORD_X) | bit(WORD_Y) | bit(WORD_S) | (curblock->value_words & bit(WORD_F));
      curblock->values.s = 0;
      curblock->values.f = state->values.f;
      curblock->modal.motion = MOTION_MODE_LINEAR;
      curblock->command_words = bit(MODAL_GROUP_G1);
      for (int i = 0; i < 2; i++)
	curblock->values.xyz[i] = x2[i];
	
      curpos = x2;
      curblock++;
    }
    
    if (extnext || extprev) { // move to the beginning of the next segment
      //      curblock->command_words = 0;
      memcpy(curblock, block + retval - 1, sizeof(block[0]));
      curblock->value_words = bit(WORD_X) | bit(WORD_Y) | bit(WORD_S) | (curblock->value_words & bit(WORD_F));
      curblock->values.s = 0;
      curblock->values.f = state->values.f;
      curblock->modal.motion = MOTION_MODE_LINEAR;
      curblock->command_words = bit(MODAL_GROUP_G1);
      for (int i = 0; i < 2; i++)
	curblock->values.xyz[i] = oldstate.values.xyz[i];
	
      curpos = oldstate.values.xyz;
      curblock++;
    }

    return retval;
}


