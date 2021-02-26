#include "cleanup.h"
#include <string.h>

void cleanup_init(cleanup_state_t *state) {
  memset(&state->modal, 255, sizeof(state->modal));
  memset(&state->values, 0, sizeof(state->values));
}

int cleanup(cleanup_state_t *state,
	      parser_block_t *block) {
  update_state(&state->modal, &state->values, block);
  return 1;
}

