#include <assert.h>
#include <math.h>
#include <string.h>
#include "geom.h"
#include "nuts_bolts.h"

void normarcs(parser_block_t *block, uint8_t motion, float x, float y) {
    // normalize arcs
  if (block->value_words & bit(WORD_R)){ // calculate i, j
    float h_x2_div_d = 4.0 * block->values.r*block->values.r - x*x - y*y;

    assert (h_x2_div_d >= 0);
    
    // Finish computing h_x2_div_d.
    h_x2_div_d = -sqrt(h_x2_div_d)/hypot_f(x,y); // == -(h * 2 / d)
    // Invert the sign of h_x2_div_d if the circle is counter clockwise (see sketch below)
    if (motion == MOTION_MODE_CCW_ARC) { h_x2_div_d = -h_x2_div_d; }
    
    if (block->values.r < 0) {
      h_x2_div_d = -h_x2_div_d;
      block->values.r = -block->values.r; // Finished with r. Set to positive for mc_arc
    }
    // Complete the operation by calculating the actual center of the arc
    block->values.ijk[0] = 0.5*(x-(y*h_x2_div_d));
    block->values.ijk[1] = 0.5*(y+(x*h_x2_div_d));
  } else if (block->value_words & (bit(WORD_I) | bit(WORD_J))) {
    x -= block->values.ijk[0]; // Delta x between circle center and target
    y -= block->values.ijk[1]; // Delta y between circle center and target
    float target_r = hypot_f(x,y);
    
    // Compute arc radius for mc_arc. Defined from current location to center.
    block->values.r = hypot_f(block->values.ijk[0], block->values.ijk[1]);
    
    // Compute difference between current location and target radii for final error-checks.
    float delta_r = fabs(target_r-block->values.r);
    assert (delta_r < 0.5);
    assert (delta_r < (0.001*block->values.r));
  }
}

void calcv(parser_block_t *block, uint8_t motion, float dx, float dy, float v0[2], float v1[2]) {
    if (block->values.r != 0) {
      // arc
      v0[0] = -block->values.ijk[1];
      v0[1] = block->values.ijk[0];
      
      for (int i = 0; i < 2; i++)
	v0[i] = v0[i] / block->values.r; // dir at beginning of line

      if (motion == MOTION_MODE_CCW_ARC)
	for (int i = 0; i < 2; i++)
	  v0[i] = -v0[i];

      v1[0] = dy - block->values.ijk[1];
      v1[1] = -dx + block->values.ijk[0];

      for (int i = 0; i < 2; i++)
	v1[i] = v1[i] / block->values.r; // dir at end of line
    } else {
      v0[0] = dx;
      v0[1] = dy;

      float d2 = 0.;
      for (int i = 0; i < 2; i++)
	d2 += v0[i] * v0[i];
      if (d2 != 0) {
	float d = sqrt(d2);
	for (int i = 0; i < 2; i++)
	  v1[i] = v0[i] / d; // dir at end of line
	memcpy(v0, v1, sizeof(float) * 2);
      }
    }
}
