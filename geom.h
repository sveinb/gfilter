#ifndef GEOM_H
#define GEOM_H

#include "gcode.h"

void normarcs(parser_block_t *block, uint8_t motion, float x, float y);
void calcv(parser_block_t *block, uint8_t motion, float dx, float dy, float v0[2], float v1[2]);

#endif

