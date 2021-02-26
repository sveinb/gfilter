/*
  gcode.c - rs274/ngc parser.
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "gcode.h"
#include "report.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// NOTE: Max line number is defined by the g-code standard to be 99999. It seems to be an
// arbitrary value, and some GUIs may require more. So we increased it based on a max safe
// value when converting a float (7.2 digit precision)s to an integer.
#define MAX_LINE_NUMBER 10000000
#define MAX_TOOL_NUMBER 255 // Limited by max unsigned 8-bit value

#define AXIS_COMMAND_NONE 0
#define AXIS_COMMAND_NON_MODAL 1
#define AXIS_COMMAND_MOTION_MODE 2
#define AXIS_COMMAND_TOOL_LENGTH_OFFSET 3 // *Undefined but required

// Declare gc extern struct

#define FAIL(status) return(status);


uint8_t gc_parse_line(char *line, parser_block_t *gc_block)
{
  /* -------------------------------------------------------------------------------------
     STEP 1: Initialize parser block struct and copy current g-code state modes. The parser
     updates these modes and commands as the block line is parser and will only be used and
     executed after successful error-checking. The parser block struct also contains a block
     values struct, word tracking variables, and a non-modal commands tracker for the new
     block. This struct contains all of the necessary information to execute the block. */

  memset(gc_block, 0, sizeof(parser_block_t)); // Initialize the parser block struct.

  uint8_t axis_command = AXIS_COMMAND_NONE;
  uint8_t axis_0, axis_1, axis_linear;
  uint8_t coord_select = 0; // Tracks G10 P coordinate selection for execution

  // Initialize bitflag tracking variables for axis indices compatible operations.
  uint8_t axis_words = 0; // XYZ tracking
  uint8_t ijk_words = 0; // IJK tracking

  // Initialize command and value words and parser flags variables.
//  uint16_t command_words = 0; // Tracks G and M command words. Also used for modal group violations.
//  uint16_t value_words = 0; // Tracks value words.
  uint8_t gc_parser_flags = GC_PARSER_NONE;

  // Determine if the line is a jogging motion or a normal g-code block->
  if (line[0] == '$') { // NOTE: `$J=` already parsed when passed to this function.
    // Set G1 and G94 enforced modes to ensure accurate error checks.
    gc_parser_flags |= GC_PARSER_JOG_MOTION;
    gc_block->modal.motion = MOTION_MODE_LINEAR;
    gc_block->modal.feed_rate = FEED_RATE_MODE_UNITS_PER_MIN;
    #ifdef USE_LINE_NUMBERS
      gc_block->values.n = JOG_LINE_NUMBER; // Initialize default line number reported during jog.
    #endif
  }

  /* -------------------------------------------------------------------------------------
     STEP 2: Import all g-code words in the block line. A g-code word is a letter followed by
     a number, which can either be a 'G'/'M' command or sets/assigns a command value. Also,
     perform initial error-checks for command word modal group violations, for any repeated
     words, and for negative values set for the value words F, N, P, T, and S. */

  uint8_t word_bit; // Bit-value for assigning tracking variables
  uint8_t char_counter;
  char letter;
  float value;
  uint8_t int_value = 0;
  uint16_t mantissa = 0;
  if (gc_parser_flags & GC_PARSER_JOG_MOTION) { char_counter = 3; } // Start parsing after `$J=`
  else { char_counter = 0; }

  while (line[char_counter] != 0) { // Loop until no more g-code words in line.

    // Import the next g-code word, expecting a letter followed by a value. Otherwise, error out.
    letter = line[char_counter];
    if((letter < 'A') || (letter > 'Z')) { FAIL(STATUS_EXPECTED_COMMAND_LETTER); } // [Expected word letter]
    char_counter++;
    if (!read_float(line, &char_counter, &value)) { FAIL(STATUS_BAD_NUMBER_FORMAT); } // [Expected word value]

    // Convert values to smaller uint8 significand and mantissa values for parsing this word.
    // NOTE: Mantissa is multiplied by 100 to catch non-integer command values. This is more
    // accurate than the NIST gcode requirement of x10 when used for commands, but not quite
    // accurate enough for value words that require integers to within 0.0001. This should be
    // a good enough comprimise and catch most all non-integer errors. To make it compliant,
    // we would simply need to change the mantissa to int16, but this add compiled flash space.
    // Maybe update this later.
    int_value = trunc(value);
    mantissa =  round(100*(value - int_value)); // Compute mantissa for Gxx.x commands.
    // NOTE: Rounding must be used to catch small floating point errors.

    // Check if the g-code word is supported or errors due to modal group violations or has
    // been repeated in the g-code block-> If ok, update the command or record its value.
    switch(letter) {

      /* 'G' and 'M' Command Words: Parse commands and check for modal group violations.
         NOTE: Modal group numbers are defined in Table 4 of NIST RS274-NGC v3, pg.20 */

      case 'G':
        // Determine 'G' command and its modal group
        switch(int_value) {
          case 10: case 28: case 30: case 92:
            // Check for G10/28/30/92 being called with G0/1/2/3/38 on same block->
            // * G43.1 is also an axis command but is not explicitly defined this way.
            if (mantissa == 0) { // Ignore G28.1, G30.1, and G92.1
              if (axis_command) { FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT); } // [Axis word/command conflict]
              axis_command = AXIS_COMMAND_NON_MODAL;
            }
            // No break. Continues to next line.
          case 4: case 53:
            word_bit = MODAL_GROUP_G0;
            gc_block->non_modal_command = int_value;
            if ((int_value == 28) || (int_value == 30) || (int_value == 92)) {
              if (!((mantissa == 0) || (mantissa == 10))) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); }
              gc_block->non_modal_command += mantissa;
              mantissa = 0; // Set to zero to indicate valid non-integer G command.
            }                
            break;
          case 0: case 1: case 2: case 3: case 38:
            // Check for G0/1/2/3/38 being called with G10/28/30/92 on same block->
            // * G43.1 is also an axis command but is not explicitly defined this way.
            if (axis_command) { FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT); } // [Axis word/command conflict]
            axis_command = AXIS_COMMAND_MOTION_MODE;
            // No break. Continues to next line.
          case 80:
            word_bit = MODAL_GROUP_G1;
            gc_block->modal.motion = int_value;
            if (int_value == 38){
              if (!((mantissa == 20) || (mantissa == 30) || (mantissa == 40) || (mantissa == 50))) {
                FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported G38.x command]
              }
              gc_block->modal.motion += (mantissa/10)+100;
              mantissa = 0; // Set to zero to indicate valid non-integer G command.
            }  
            break;
          case 17: case 18: case 19:
            word_bit = MODAL_GROUP_G2;
            gc_block->modal.plane_select = int_value - 17;
            break;
          case 90: case 91:
            if (mantissa == 0) {
              word_bit = MODAL_GROUP_G3;
              gc_block->modal.distance = int_value - 90;
            } else {
              word_bit = MODAL_GROUP_G4;
              if ((mantissa != 10) || (int_value == 90)) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [G90.1 not supported]
              mantissa = 0; // Set to zero to indicate valid non-integer G command.
              // Otherwise, arc IJK incremental mode is default. G91.1 does nothing.
            }
            break;
          case 93: case 94:
            word_bit = MODAL_GROUP_G5;
            gc_block->modal.feed_rate = 94 - int_value;
            break;
          case 20: case 21:
            word_bit = MODAL_GROUP_G6;
            gc_block->modal.units = 21 - int_value;
            break;
          case 40:
            word_bit = MODAL_GROUP_G7;
            // NOTE: Not required since cutter radius compensation is always disabled. Only here
            // to support G40 commands that often appear in g-code program headers to setup defaults.
            // gc_block->modal.cutter_comp = CUTTER_COMP_DISABLE; // G40
            break;
          case 43: case 49:
            word_bit = MODAL_GROUP_G8;
            // NOTE: The NIST g-code standard vaguely states that when a tool length offset is changed,
            // there cannot be any axis motion or coordinate offsets updated. Meaning G43, G43.1, and G49
            // all are explicit axis commands, regardless if they require axis words or not.
            if (axis_command) { FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT); } // [Axis word/command conflict] }
            axis_command = AXIS_COMMAND_TOOL_LENGTH_OFFSET;
            if (int_value == 49) { // G49
              gc_block->modal.tool_length = TOOL_LENGTH_OFFSET_CANCEL;
            } else if (mantissa == 10) { // G43.1
              gc_block->modal.tool_length = TOOL_LENGTH_OFFSET_ENABLE_DYNAMIC;
            } else { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [Unsupported G43.x command]
            mantissa = 0; // Set to zero to indicate valid non-integer G command.
            break;
          case 54: case 55: case 56: case 57: case 58: case 59:
            // NOTE: G59.x are not supported. (But their int_values would be 60, 61, and 62.)
            word_bit = MODAL_GROUP_G12;
            gc_block->modal.coord_select = int_value - 54; // Shift to array indexing.
            break;
          case 61:
            word_bit = MODAL_GROUP_G13;
            if (mantissa != 0) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [G61.1 not supported]
            // gc_block->modal.control = CONTROL_MODE_EXACT_PATH; // G61
            break;
          default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported G command]
        }
        if (mantissa > 0) { FAIL(STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER); } // [Unsupported or invalid Gxx.x command]
        // Check for more than one command per modal group violations in the current block
        // NOTE: Variable 'word_bit' is always assigned, if the command is valid.
        if ( bit_istrue(gc_block->command_words,bit(word_bit)) ) { FAIL(STATUS_GCODE_MODAL_GROUP_VIOLATION); }
        gc_block->command_words |= bit(word_bit);
        break;

      case 'M':

        // Determine 'M' command and its modal group
        if (mantissa > 0) { FAIL(STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER); } // [No Mxx.x commands]
        switch(int_value) {
          case 0: case 1: case 2: case 30:
            word_bit = MODAL_GROUP_M4;
            switch(int_value) {
              case 0: gc_block->modal.program_flow = PROGRAM_FLOW_PAUSED; break; // Program pause
              case 1: break; // Optional stop not supported. Ignore.
              default: gc_block->modal.program_flow = int_value; // Program end and reset
            }
            break;
          case 3: case 4: case 5:
            word_bit = MODAL_GROUP_M7;
            switch(int_value) {
              case 3: gc_block->modal.spindle = SPINDLE_ENABLE_CW; break;
              case 4: gc_block->modal.spindle = SPINDLE_ENABLE_CCW; break;
              case 5: gc_block->modal.spindle = SPINDLE_DISABLE; break;
            }
            break;
          #ifdef ENABLE_M7
            case 7: case 8: case 9:
          #else
            case 8: case 9:
          #endif
            word_bit = MODAL_GROUP_M8;
            switch(int_value) {
              #ifdef ENABLE_M7
                case 7: gc_block->modal.coolant |= COOLANT_MIST_ENABLE; break;
              #endif
              case 8: gc_block->modal.coolant |= COOLANT_FLOOD_ENABLE; break;
              case 9: gc_block->modal.coolant = COOLANT_DISABLE; break; // M9 disables both M7 and M8.
            }
            break;
          #ifdef ENABLE_PARKING_OVERRIDE_CONTROL
            case 56:
              word_bit = MODAL_GROUP_M9;
              gc_block->modal.override = OVERRIDE_PARKING_MOTION;
              break;
          #endif
          default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported M command]
        }

        // Check for more than one command per modal group violations in the current block
        // NOTE: Variable 'word_bit' is always assigned, if the command is valid.
        if ( bit_istrue(gc_block->command_words,bit(word_bit)) ) { FAIL(STATUS_GCODE_MODAL_GROUP_VIOLATION); }
        gc_block->command_words |= bit(word_bit);
        break;

      // NOTE: All remaining letters assign values.
      default:

        /* Non-Command Words: This initial parsing phase only checks for repeats of the remaining
           legal g-code words and stores their value. Error-checking is performed later since some
           words (I,J,K,L,P,R) have multiple connotations and/or depend on the issued commands. */
        switch(letter){
          // case 'A': // Not supported
          // case 'B': // Not supported
          // case 'C': // Not supported
          // case 'D': // Not supported
          case 'F': word_bit = WORD_F; gc_block->values.f = value; break;
          // case 'H': // Not supported
          case 'I': word_bit = WORD_I; gc_block->values.ijk[X_AXIS] = value; ijk_words |= (1<<X_AXIS); break;
          case 'J': word_bit = WORD_J; gc_block->values.ijk[Y_AXIS] = value; ijk_words |= (1<<Y_AXIS); break;
          case 'K': word_bit = WORD_K; gc_block->values.ijk[Z_AXIS] = value; ijk_words |= (1<<Z_AXIS); break;
          case 'L': word_bit = WORD_L; gc_block->values.l = int_value; break;
          case 'N': word_bit = WORD_N; gc_block->values.n = trunc(value); break;
          case 'P': word_bit = WORD_P; gc_block->values.p = value; break;
          // NOTE: For certain commands, P value must be an integer, but none of these commands are supported.
          // case 'Q': // Not supported
          case 'R': word_bit = WORD_R; gc_block->values.r = value; break;
          case 'S': word_bit = WORD_S; gc_block->values.s = value; break;
          case 'T': word_bit = WORD_T; 
					  if (value > MAX_TOOL_NUMBER) { FAIL(STATUS_GCODE_MAX_VALUE_EXCEEDED); }
            gc_block->values.t = int_value;
						break;
          case 'X': word_bit = WORD_X; gc_block->values.xyz[X_AXIS] = value; axis_words |= (1<<X_AXIS); break;
          case 'Y': word_bit = WORD_Y; gc_block->values.xyz[Y_AXIS] = value; axis_words |= (1<<Y_AXIS); break;
          case 'Z': word_bit = WORD_Z; gc_block->values.xyz[Z_AXIS] = value; axis_words |= (1<<Z_AXIS); break;
          default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND);
        }

        // NOTE: Variable 'word_bit' is always assigned, if the non-command letter is valid.
        if (bit_istrue(gc_block->value_words,bit(word_bit))) { FAIL(STATUS_GCODE_WORD_REPEATED); } // [Word repeated]
        // Check for invalid negative values for words F, N, P, T, and S.
        // NOTE: Negative value check is done here simply for code-efficiency.
        if ( bit(word_bit) & (bit(WORD_F)|bit(WORD_N)|bit(WORD_P)|bit(WORD_T)|bit(WORD_S)) ) {
          if (value < 0.0) { FAIL(STATUS_NEGATIVE_VALUE); } // [Word value cannot be negative]
        }
        gc_block->value_words |= bit(word_bit); // Flag to indicate parameter assigned.

    }
  }

 
  return(STATUS_OK);
}


/*
  Not supported:

  - Canned cycles
  - Tool radius compensation
  - A,B,C-axes
  - Evaluation of expressions
  - Variables
  - Override control (TBD)
  - Tool changes
  - Switches

   (*) Indicates optional parameter, enabled through config.h and re-compile
   group 0 = {G92.2, G92.3} (Non modal: Cancel and re-enable G92 offsets)
   group 1 = {G81 - G89} (Motion modes: Canned cycles)
   group 4 = {M1} (Optional stop, ignored)
   group 6 = {M6} (Tool change)
   group 7 = {G41, G42} cutter radius compensation (G40 is supported)
   group 8 = {G43} tool length offset (G43.1/G49 are supported)
   group 8 = {M7*} enable mist coolant (* Compile-option)
   group 9 = {M48, M49, M56*} enable/disable override switches (* Compile-option)
   group 10 = {G98, G99} return mode canned cycles
   group 13 = {G61.1, G64} path control mode (G61 is supported)
*/

void update_state(gc_modal_t *modal, gc_values_t *values,
		  parser_block_t *block) {
  // updates state and removes commands from block that are redundant
  
  if ((block->value_words & bit(WORD_F))) {
    if (block->values.f == values->f)
      block->value_words &= ~bit(WORD_F);
    else
      values->f = block->values.f;
  }
  
  if ((block->value_words & bit(WORD_I)) &&
      block->values.ijk[0] == 0)
    block->value_words &= ~bit(WORD_I);
  if ((block->value_words & bit(WORD_J)) &&
      block->values.ijk[1] == 0)
    block->value_words &= ~bit(WORD_J);
  if ((block->value_words & bit(WORD_K)) &&
      block->values.ijk[2] == 0)
    block->value_words &= ~bit(WORD_K);

  if ((block->value_words & bit(WORD_L))) {
    if (block->values.l == values->l)
      block->value_words &= ~bit(WORD_L);
    else
      values->l = block->values.l;
  }
  
  if ((block->value_words & bit(WORD_N))) {
    if (block->values.n == values->n)
      block->value_words &= ~bit(WORD_N);
    else
      values->n = block->values.n;
  }
  
  if ((block->value_words & bit(WORD_P))) {
    if (block->values.p == values->p)
      block->value_words &= ~bit(WORD_P);
    else
      values->p = block->values.p;
  }
  
  if ((block->value_words & bit(WORD_R)) &&
      block->values.r == 0)
    block->value_words &= ~bit(WORD_R);
    
  if ((block->value_words & bit(WORD_S))) {
    if (block->values.s == values->s)
      block->value_words &= ~bit(WORD_S);
    else
      values->s = block->values.s;
  }

  if ((block->value_words & bit(WORD_T))) {
    if (block->values.t == values->t)
      block->value_words &= ~bit(WORD_T);
    else
      values->t = block->values.t;
  }

  if (block->command_words & bit(MODAL_GROUP_G1)) {
    if (modal->motion == block->modal.motion)
      block->command_words &= ~bit(MODAL_GROUP_G1);
    else
      modal->motion = block->modal.motion;
  }
  if (block->command_words & bit(MODAL_GROUP_G2)) {
    if (modal->plane_select == block->modal.plane_select)
      block->command_words &= ~bit(MODAL_GROUP_G2);
    else
      modal->plane_select = block->modal.plane_select;
  }
  if (block->command_words & bit(MODAL_GROUP_G3)) {
    if (modal->distance == block->modal.distance)
      block->command_words &= ~bit(MODAL_GROUP_G3);
    else
      modal->distance = block->modal.distance;
  }
  if (block->command_words & bit(MODAL_GROUP_G5)) {
    if (modal->feed_rate == block->modal.feed_rate)
      block->command_words &= ~bit(MODAL_GROUP_G5);
    else
      modal->feed_rate = block->modal.feed_rate;
  }
  if (block->command_words & bit(MODAL_GROUP_G6)) {
    if (modal->units == block->modal.units)
      block->command_words &= ~bit(MODAL_GROUP_G6);
    else
      modal->units = block->modal.units;
  }
  if (block->command_words & bit(MODAL_GROUP_G8)) {
    if (modal->tool_length == block->modal.tool_length)
      block->command_words &= ~bit(MODAL_GROUP_G8);
    else
      modal->tool_length = block->modal.tool_length;
  }
  if (block->command_words & bit(MODAL_GROUP_G12)) {
    if (modal->coord_select == block->modal.coord_select)
      block->command_words &= ~bit(MODAL_GROUP_G12);
    else
      modal->coord_select = block->modal.coord_select;
  }
  if (block->command_words & bit(MODAL_GROUP_M4)) {
    if (modal->program_flow == block->modal.program_flow)
      block->command_words &= ~bit(MODAL_GROUP_M4);
    else
      modal->program_flow = block->modal.program_flow;
  }
  if (block->command_words & bit(MODAL_GROUP_M7)) {
    if (modal->spindle == block->modal.spindle)
      block->command_words &= ~bit(MODAL_GROUP_M7);
    else
      modal->spindle = block->modal.spindle;
  }
  if (block->command_words & bit(MODAL_GROUP_M8)) {
    if (modal->coolant == block->modal.coolant)
      block->command_words &= ~bit(MODAL_GROUP_M8);
    else
      modal->coolant = block->modal.coolant;
  }

  if (modal->distance == DISTANCE_MODE_ABSOLUTE) {
    for (int i = 0; i < N_AXIS; i++)
      if ((block->value_words & bit(WORD_X + i))) {
	if (block->values.xyz[i] == values->xyz[i])
	  block->value_words &= ~bit(WORD_X + i);
	else
	  values->xyz[i] = block->values.xyz[i];
      }
  } else {
    for (int i = 0; i < N_AXIS; i++)
      if ((block->value_words & bit(WORD_X + i)) &&
	  block->values.xyz[i] == 0)
	block->value_words &= ~bit(WORD_X + i);
      else
	  values->xyz[i] += block->values.xyz[i];
  }
  
#if 0
  //  bool moves = false;
  for (int i = 0; i < N_AXIS; i++) {
    if (block->value_words & bit(WORD_X + i)) {
      if (modal->distance == DISTANCE_MODE_ABSOLUTE)
	values->xyz[i] = block->values.xyz[i];
      else
	values->xyz[i] += block->values.xyz[i];
      //      if (state->values.xyz[i] != oldstate.values.xyz[i])
      //	moves = true;
    }
  }
#endif
  
}

void gc_print_line(parser_block_t *block, FILE *output) {
  if (block->value_words & bit(WORD_F))
    fprintf(output, "F%g", block->values.f);
  if (block->value_words & bit(WORD_I))
    fprintf(output, "I%g", block->values.ijk[0]);
  if (block->value_words & bit(WORD_J))
    fprintf(output, "J%g", block->values.ijk[1]);
  if (block->value_words & bit(WORD_K))
    fprintf(output, "K%g", block->values.ijk[2]);
  if (block->value_words & bit(WORD_L))
    fprintf(output, "L%d", block->values.l);
  if (block->value_words & bit(WORD_N))
    fprintf(output, "N%d", block->values.n);
  if (block->value_words & bit(WORD_P))
    fprintf(output, "P%g", block->values.p);
  if (block->value_words & bit(WORD_R))
    fprintf(output, "R%g", block->values.r);
  if (block->value_words & bit(WORD_S))
    fprintf(output, "S%g", block->values.s);
  if (block->value_words & bit(WORD_T))
    fprintf(output, "T%d", block->values.t);
  if (block->value_words & bit(WORD_X))
    fprintf(output, "X%g", block->values.xyz[0]);
  if (block->value_words & bit(WORD_Y))
    fprintf(output, "Y%g", block->values.xyz[1]);
  if (block->value_words & bit(WORD_Z))
    fprintf(output, "Z%g", block->values.xyz[2]);

  if (block->command_words & bit(MODAL_GROUP_G0)) {
    switch(block->non_modal_command) {
    case 38:
    case 40:
    case 102:
      fprintf(output, "G%d.1", block->non_modal_command - 10);
      break;
    default:
      fprintf(output, "G%d", block->non_modal_command);
      break;
    }
  }
  if (block->command_words & bit(MODAL_GROUP_G1)) {
    float val = block->modal.motion;
    if (val > 100) {
      val = (val - 138) / 10. + 38;
    }
    fprintf(output, "G%g", val);
  }
  if (block->command_words & bit(MODAL_GROUP_G2)) {
    fprintf(output, "G%d", 17 + block->modal.plane_select);
  }
  if (block->command_words & bit(MODAL_GROUP_G3)) {
    fprintf(output, "G%d", 90 + block->modal.distance);
  }
  if (block->command_words & bit(MODAL_GROUP_G4)) {
    fprintf(output, "G91.1");
  }
  if (block->command_words & bit(MODAL_GROUP_G5)) {
    fprintf(output, "G%d", 94 - block->modal.feed_rate);
  }
  if (block->command_words & bit(MODAL_GROUP_G6)) {
    fprintf(output, "G%d", 21 - block->modal.units);
  }
  if (block->command_words & bit(MODAL_GROUP_G7)) {
    fprintf(output, "G40");
  }
  if (block->command_words & bit(MODAL_GROUP_G8)) {
    switch(block->modal.tool_length) {
    case TOOL_LENGTH_OFFSET_CANCEL:
      fprintf(output, "G49");
      break;
    case TOOL_LENGTH_OFFSET_ENABLE_DYNAMIC:
      fprintf(output, "G43.1");
      break;
    }
  }
  if (block->command_words & bit(MODAL_GROUP_G12)) {
    fprintf(output, "G%d", block->modal.coord_select + 54);
    
  }
  if (block->command_words & bit(MODAL_GROUP_G13)) {
    fprintf(output, "G61");
  }
  if (block->command_words & bit(MODAL_GROUP_M4)) {
    if (block->modal.program_flow == PROGRAM_FLOW_PAUSED)
      fprintf(output, "M0");
    else
      fprintf(output, "M%d", block->modal.program_flow);
  }
  if (block->command_words & bit(MODAL_GROUP_M7)) {
    switch(block->modal.spindle) {
    case SPINDLE_ENABLE_CW:
      fprintf(output, "M3");
      break;
    case SPINDLE_ENABLE_CCW:
      fprintf(output, "M4");
      break;
    case SPINDLE_DISABLE:
      fprintf(output, "M5");
      break;
    }
  }
  if (block->command_words & bit(MODAL_GROUP_M8)) {
    if (block->modal.coolant & COOLANT_MIST_ENABLE)
      fprintf(output, "M7");
    if (block->modal.coolant & COOLANT_FLOOD_ENABLE)
      fprintf(output, "M8");
    if (block->modal.coolant == COOLANT_DISABLE)
      fprintf(output, "M9");
  }
  if (block->command_words & bit(MODAL_GROUP_M9)) {
    fprintf(output, "M56");
  }
}
