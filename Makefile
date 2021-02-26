CFLAGS = -g

all:	gfilter

OBJS = absmode.o cleanup.o dragmode.o gcode.o geom.o gfilter.o lasermode.o mm_mode.o nuts_bolts.o report.o

gfilter:	$(OBJS)
	$(CC) $(OBJS) -o gfilter -lm

clean:
	rm -f $(OBJS) gfilter *~
