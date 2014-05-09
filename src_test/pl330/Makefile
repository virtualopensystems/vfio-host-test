CC=gcc
CFLAGS = -I.
GFLAGS = `pkg-config --cflags glib-2.0`
GLIBS = `pkg-config --libs glib-2.0`
PTHREAD_LIBS = -lpthread
DEPS = pl330_vfio_driver/pl330_vfio.h ../vfio_utils.h
OBJ = pl330_vfio_driver/pl330_vfio.o test_pl330_vfio_driver.o ../vfio_utils.o

%.o: %.c $(DEPS)
	$(CC) -c $(GFLAGS) -o $@ $< $(CFLAGS)

test_pl330_vfio_driver: $(OBJ)
	$(CC) $(GFLAGS) -o $@ $^ $(CFLAGS) $(GLIBS) $(PTHREAD_LIBS)

clean:
	rm -f pl330_vfio_driver/pl330_vfio.o test_pl330_vfio_driver test_pl330_vfio_driver.o
