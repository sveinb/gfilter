# gfilter
Laser cutter and dragknife filter for gcode files.

This little program has two modes: Laser cutter mode and drag knife mode. You'll probably only be interested in one of them, but they share a lot of the same code so they've been rolled up into one program. They are both used with CNC machines that accept gcode. The program loans some code from the grbl project, but is not otherwise associated with it.

# The laser cutter mode 

A problem with CNC machines used as laser cutters is that the machine will accelerate and decelerate at the beginnings and ends of each line segment, thus increasing the laser dose applied to the work piece around corners. This is partially solved in some drivers like grbl by modulating the laser intensity to compensate for the speed variation, so that the total dose remains constant. However, laser cutting is not a linear process, so the cutting parameters that work on the straights might not give ideal results close to corners.

The solution is the gfilter laser mode. This filter will modify a gcode file so that all cutting paths are extended at both ends. The laser is cut off during these extra path segments, so the result is that the laser will always move at the prescribed speed while it is cutting. The downside is that the job will take longer because of the extra path segments. The length of the segments is calculated to be exactly the length needed to accellerate the CNC machine up to the prescribed cutting speed. The accelleration of the CNC machine must be supplied as a command-line parameter.

Acceleration segments can be foregone at corners that are very blunt, since the CNC machine will not slow down for them. The threshold angle can be given as a command-line parameter.

# The drag knife mode

A problem with CNC machines used as drag knife cutters is that the knife needs to swivel whenever a tight corner is required, and these swivel actions are not done automatically by the CNC machine, nor are they added by most gcode editors.

The gfilter drag knife mode will add swivel actions to a gcode file whenever the corner angle is below a certain angle. It will also increase the radius of any curves to accomodate the drag knife.

The drag knife is assumed to be pointing in the positive x direction at the start of the program (i.e. the tip of the blade is left of the swivel axis), so you should orient the blade this way manually before running the job. It is not necessarily returned to this orientation at the end of the job.

The offset between the swivel axis and the cutting edge must be specified on the command line (i mm). The minimum deflection angle which will get a swivel action can optionally be specified (in degrees).

# Usage

    Usage: gfilter <-l acc | -d offs> [-a deg] [infile [outfile]]
    options:
      -l <acc>  Laser mode / accelleration (mm/s2)
      -d <offs> Drag knife mode / offset (mm)
      -a <deg>  Max deflection angle which should be treated as continuous curve
                Default = 2
