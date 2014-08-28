CC            = g++
CFLAGS        = -I/usr/local/include -O2 -Wall
LDFLAGS       = -L/usr/local/lib -D__STDC_CONSTANT_MACROS
LIBS          = -lm                          \
                -lwsock32                    \
                -lpthread                    \
                -lavcodec.dll                \
                -lavdevice.dll               \
                -lavfilter.dll               \
                -lavformat.dll               \
                -lavutil.dll                 \ 
                -lswscale.dll                \
                -lopencv_calib3d300.dll      \
                -lopencv_core300.dll         \
                -lopencv_features2d300.dll   \
                -lopencv_flann300.dll        \
                -lopencv_highgui300.dll      \
                -lopencv_imgcodec300.dll     \
                -lopencv_imgproc300.dll      \
                -lopencv_ml300.dll           \
                -lopencv_objdetect300.dll    \
                -lopencv_photo300.dll        \
                -lopencv_shape300.dll        \
                -lopencv_stitching300.dll    \
                -lopencv_superres300.dll     \
                -lopencv_video300.dll        \
                -lopencv_videoio300.dll      \
                -lopencv_videostab300.dll
OBJS          = ../../src/ardrone/ardrone.o \
                ../../src/ardrone/command.o \
                ../../src/ardrone/config.o  \
                ../../src/ardrone/udp.o     \
                ../../src/ardrone/tcp.o     \
                ../../src/ardrone/navdata.o \
                ../../src/ardrone/version.o \
                ../../src/ardrone/video.o   \
                ../../src/main.o            
PROGRAM       = test.exe

$(PROGRAM):     $(OBJS)
		$(CC) $(OBJS) -o $(PROGRAM) $(CFLAGS) $(LDFLAGS) $(LIBS) 

clean:;         rm -f *.o *~ $(PROGRAM)

install:        $(PROGRAM)
		install -s $(PROGRAM) $(DEST)