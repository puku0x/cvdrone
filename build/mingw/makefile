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
                -lopencv_calib3d249.dll      \
                -lopencv_contrib249.dll      \
                -lopencv_core249.dll         \
                -lopencv_features2d249.dll   \
                -lopencv_flann249.dll        \
                -lopencv_highgui249.dll      \
                -lopencv_imgproc249.dll      \
                -lopencv_legacy249.dll       \
                -lopencv_ml249.dll           \
                -lopencv_objdetect249.dll    \
                -lopencv_photo249.dll        \
                -lopencv_stitching249.dll    \
                -lopencv_superres249.dll     \
                -lopencv_video249.dll        \
                -lopencv_videostab249.dll
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