#
# This work is licensed under the terms of the GNU GPL, version 2 or
# (at your option) any later version. See the COPYING file in the
# top-level directory.
#
CC=gcc
CFLAGS = -I.
GFLAGS = `pkg-config --cflags glib-2.0`
GLIBS = `pkg-config --libs glib-2.0`
DEPS = vfio_utils.h
OBJ = vfio_utils.o vfio_device_test.o

%.o: %.c $(DEPS)
	$(CC) -c $(GFLAGS) -o $@ $< $(CFLAGS)

vfio_dev_test: $(OBJ)
	$(CC) $(GFLAGS) -o $@ $^ $(CFLAGS) $(GLIBS)

clean:
	rm -f vfio_device_test.o vfio_dev_test vfio_utils.o
