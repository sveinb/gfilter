#include <string.h>
#include "absmode.h"
#include "nuts_bolts.h"

int toabs_init(toabs_state_t *state) {
  memset(&state->xyz, 0, sizeof(state->xyz));
  state->distance = DISTANCE_MODE_ABSOLUTE;
}

int toabs(toabs_state_t *state, parser_block_t *block) {
  if (block->command_words & bit(MODAL_GROUP_G3)) {
    if (state->distance == block->modal.distance) {
      block->command_words &= ~bit(MODAL_GROUP_G3);
    } else {
      state->distance = block->modal.distance;
      block->modal.distance = DISTANCE_MODE_ABSOLUTE;
    }
  }
  
  for (int i = 0; i < 3; i++)
    if (block->value_words & bit(WORD_X + i)) {
      if (state->distance == DISTANCE_MODE_ABSOLUTE)
	state->xyz[i] = block->values.xyz[i];
      else {
	state->xyz[i] += block->values.xyz[i];
	block->values.xyz[i] = state->xyz[i];
      }
    }
  return 1;
}

int fromabs_init(fromabs_state_t *state) {
  memset(&state->xyz, 0, sizeof(state->xyz));
  state->distance = 255;
}

int fromabs(fromabs_state_t *state, parser_block_t *block) {
  if (state->distance == 255) {
    if (block->command_words & bit(MODAL_GROUP_G3)) {
      state->distance = DISTANCE_MODE_INCREMENTAL;
    } else {
      block->command_words |= bit(MODAL_GROUP_G3);
      state->distance = DISTANCE_MODE_ABSOLUTE;
    }
    block->modal.distance = state->distance;
  } else if (block->command_words & bit(MODAL_GROUP_G3)) {
    state->distance = 1 - state->distance; // toggle
    block->modal.distance = state->distance;
  }

  for (int i = 0; i < 3; i++)
    if (block->value_words & bit(WORD_X + i)) {
      if (state->distance == DISTANCE_MODE_ABSOLUTE)
	state->xyz[i] = block->values.xyz[i];
      else {
	block->values.xyz[i] -= state->xyz[i];
	state->xyz[i] += block->values.xyz[i];
      }
    }

  return 1;
}


