#include "mm_mode.h"

int to_mm_init(to_mm_state_t *state) {
  state->units = UNITS_MODE_MM;
}

int to_mm(to_mm_state_t *state, parser_block_t *block) {
  if (block->command_words & bit(MODAL_GROUP_G6)) {
    if (state->units == block->modal.units) {
      block->command_words &= ~bit(MODAL_GROUP_G6);
    } else {
      state->units = block->modal.units;
      block->modal.units = UNITS_MODE_MM;
    }
  }

  if (state->units == UNITS_MODE_INCHES) {
    for (int i = 0; i < 3; i++) {
      block->values.xyz[i] *= 25.4;
      block->values.ijk[i] *= 25.4;
    }
    block->values.f *= 25.4;
    block->values.r *= 25.4;
  }
  return 1;
}

int from_mm_init(from_mm_state_t *state) {
  state->units = 255;
}

int from_mm(from_mm_state_t *state, parser_block_t *block) {
  if (state->units == 255) {
    if ((block->command_words & bit(MODAL_GROUP_G6)) == 0) {
      block->command_words |= bit(MODAL_GROUP_G6);
      state->units = UNITS_MODE_MM;
    } else {
      state->units = UNITS_MODE_INCHES;
    }
    block->modal.units = state->units;
  } else if (block->command_words & bit(MODAL_GROUP_G6)) {
    state->units = 1 - state->units; // toggle
    block->modal.units = state->units;
  }

  if (state->units == UNITS_MODE_INCHES) {
    for (int i = 0; i < 3; i++) {
      block->values.xyz[i] /= 25.4;
      block->values.ijk[i] /= 25.4;
    }
    block->values.f /= 25.4;
    block->values.r /= 25.4;
  }

  return 1;
}

