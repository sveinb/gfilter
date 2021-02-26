#define LINE_BUFFER_SIZE 1024
#define LINE_FLAG_OVERFLOW 1
#define LINE_FLAG_COMMENT_PARENTHESES 2
#define LINE_FLAG_COMMENT_SEMICOLON 4

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include "report.h"
#include "gcode.h"
#include "lasermode.h"
#include "dragmode.h"
#include "mm_mode.h"
#include "cleanup.h"
#include "absmode.h"

#define MODE_LASER 1
#define MODE_DRAG 2

void usage() {
  fprintf(stderr, "Usage: gfilter <-l acc | -d offs> [-a deg] [infile [outfile]]\n");
  fprintf(stderr, "options:\n");
  fprintf(stderr, "  -l <acc>  Laser mode / accelleration (mm/s2)\n");
  fprintf(stderr, "  -d <offs> Drag knife mode / offset (mm)\n");
  fprintf(stderr, "  -a <deg>  Max deflection angle which should be treated as continuous curve\n");
  fprintf(stderr, "            Default = 2\n");
  exit(1);
}

int main(int argc, char **argv) {
  int mode = 0;
  float angle = 2;
  FILE *infile = NULL;
  FILE *outfile = NULL;
  float acc = 0;
  float offset = 0;
  
  int opt;
  while ((opt = getopt(argc, argv, "l:d:a:")) != -1) {
    switch (opt) {
    case 'l':
      mode = MODE_LASER;
      acc = atof(optarg);
      break;
    case 'd':
      mode = MODE_DRAG;
      offset = atof(optarg);
      break;
    case 'a':
      angle = atof(optarg);
      break;
    default:
      usage();
    }
  }

  if (mode == 0)
    usage();
  
  if (optind < argc) {
    infile = fopen(argv[optind], "rt");
    if (!infile) {
      perror("Could not open input file");
      exit(2);
    }
  } else {
    infile = stdin;
  }
  
  if (optind < argc - 1) {
    outfile = fopen(argv[optind + 1], "wt");
    if (!outfile) {
      perror("Could not open output");
      exit(3);
    }
  } else {
    outfile = stdout;
  }
  
  char line[LINE_BUFFER_SIZE];
  
  uint8_t line_flags = 0;
  uint8_t char_counter = 0;
  int c;
  parser_block_t blocks[6];

  laser_state_t laser_state;
  if (mode == MODE_LASER)
    lasermode_init(&laser_state, acc, angle);

  cleanup_state_t cleanup_state;
  cleanup_init(&cleanup_state);

  drag_state_t drag_state;
  if (mode == MODE_DRAG)
    dragmode_init(&drag_state, offset, 0, angle);

  toabs_state_t toabs_state;
  toabs_init(&toabs_state);

  fromabs_state_t fromabs_state;
  fromabs_init(&fromabs_state);

  to_mm_state_t to_mm_state;
  to_mm_init(&to_mm_state);

  from_mm_state_t from_mm_state;
  from_mm_init(&from_mm_state);

  
  // Process one line of incoming serial data, as the data becomes available. Performs an
  // initial filtering by removing spaces and comments and capitalizing all letters.
  while(!feof(infile) && (c = fgetc(infile)) != EOF) {
    if ((c == '\n') || (c == '\r')) { // End of line reached

      line[char_counter] = 0; // Set string termination character.

      // Direct and execute one line of formatted input, and report status of execution.
      if (line_flags & LINE_FLAG_OVERFLOW) {
	// Report line overflow error.
	report_status_message(STATUS_OVERFLOW);
      } else if (line[0] == 0) {
	// Empty or comment line.
	fprintf(outfile, "\n");
      } else if (line[0] == '$') {
	// Grbl '$' system command
	fprintf(outfile, "%s\n", line);
      } else {
	// Parse and execute g-code block.
	report_status_message(gc_parse_line(line, &blocks[0]));

	to_mm(&to_mm_state, &blocks[0]);
	toabs(&toabs_state, &blocks[0]);

	int nblocks = 1;

	switch (mode) {
	case MODE_LASER:
	  nblocks = lasermode(&laser_state, blocks);
	  break;
	case MODE_DRAG:
	  nblocks = dragmode(&drag_state, blocks);
	  break;
	}
	
	for (int i = 0; i < nblocks; i++) {
	  fromabs(&fromabs_state, &blocks[i]);
	  from_mm(&from_mm_state, &blocks[i]);
	  cleanup(&cleanup_state, &blocks[i]);
	  gc_print_line(&blocks[i], outfile);
	  fprintf(outfile, "\n");
	}
      }

      // Reset tracking data for next line.
      line_flags = 0;
      char_counter = 0;

    } else {

      if (line_flags) {
	// Throw away all (except EOL) comment characters and overflow characters.
	//	fputc(c, outfile);
	if (c == ')') {
	  // End of '()' comment. Resume line allowed.
	  if (line_flags & LINE_FLAG_COMMENT_PARENTHESES) { line_flags &= ~(LINE_FLAG_COMMENT_PARENTHESES); }
	}
      } else {
	if (c <= ' ') {
	  // Throw away whitepace and control characters
	} else if (c == '/') {
	  // Block delete NOT SUPPORTED. Ignore character.
	  // NOTE: If supported, would simply need to check the system if block delete is enabled.
	} else if (c == '(') {
	  // Enable comments flag and ignore all characters until ')' or EOL.
	  // NOTE: This doesn't follow the NIST definition exactly, but is good enough for now.
	  // In the future, we could simply remove the items within the comments, but retain the
	  // comment control characters, so that the g-code parser can error-check it.
	  line_flags |= LINE_FLAG_COMMENT_PARENTHESES;
	  //	  fputc(c, outfile);
	} else if (c == ';') {
	  // NOTE: ';' comment to EOL is a LinuxCNC definition. Not NIST.
	  line_flags |= LINE_FLAG_COMMENT_SEMICOLON;
	  // TODO: Install '%' feature
	  // } else if (c == '%') {
	  // Program start-end percent sign NOT SUPPORTED.
	  // NOTE: This maybe installed to tell Grbl when a program is running vs manual input,
	  // where, during a program, the system auto-cycle start will continue to execute
	  // everything until the next '%' sign. This will help fix resuming issues with certain
	  // functions that empty the planner buffer to execute its task on-time.
	  //	  fputc(c, outfile);
	} else if (char_counter >= (LINE_BUFFER_SIZE-1)) {
	  // Detect line buffer overflow and set flag.
	  line_flags |= LINE_FLAG_OVERFLOW;
	} else if (c >= 'a' && c <= 'z') { // Upcase lowercase
	  line[char_counter++] = c-'a'+'A';
	} else {
	  line[char_counter++] = c;
	}
      }

    }
  }

  fclose(infile);
  fclose(outfile);
  
  return 0;
}
