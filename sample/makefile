RASPIKE_LIB_ROOT=..

INCLUDES := -I$(RASPIKE_LIB_ROOT)/include -I$(RASPIKE_LIB_ROOT)/drivers/include -I$(RASPIKE_LIB_ROOT)/drivers -I$(RASPIKE_LIB_ROOT)/external/libpybricks/lib/pbio/include -I$(RASPIKE_LIB_ROOT) -I.

LIBS=$(RASPIKE_LIB_ROOT)/lib/libraspike-art.a
LIB_SRCS=$(wildcard $(RASPIKE_LIB_ROOT)/src/*.[ch]) $(wildcard $(RASPIKE_LIB_ROOT)/include/*.h)

.PHONY: default clean

default:loop_period_measure

clean:
	rm -f *~ *.o *.so *.a loop_period_measure pc_loop_period.csv spike_usb_loop_period.csv spike_usb_loop_stats.csv

raspike_test:raspike_test.o $(LIBS)
	rm -f $@
	gcc -g -O2 -o $@ $^ -pthread
	test -s $@

loop_period_measure:loop_period_measure.o $(LIBS)
	rm -f $@
	gcc -g -O2 -o $@ $^ -pthread
	test -s $@

$(LIBS): $(LIB_SRCS)
	$(MAKE) -C $(RASPIKE_LIB_ROOT)/build/obj-libraspike-art

.c.o:
	gcc -g -O2 -c $(INCLUDES) $(CFLAGS) -o $@ $<
