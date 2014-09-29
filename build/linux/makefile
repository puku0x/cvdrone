#sudo apt-get install ffmpeg
#sudo apt-get install libopencv-dev

CXX           = g++
CXXFLAGS      = -O2 -Wall -D__STDC_CONSTANT_MACROS 
LIBS          = -lm                     \
                -lpthread               \
                -lavutil                \
                -lavformat              \
                -lavcodec               \
                -lswscale               \
                -lopencv_calib3d        \
                -lopencv_contrib        \
                -lopencv_core           \
                -lopencv_features2d     \
                -lopencv_flann          \
                -lopencv_highgui        \
                -lopencv_imgproc        \
                -lopencv_legacy         \
                -lopencv_ml             \
                -lopencv_objdetect      \
                -lopencv_photo          \
                -lopencv_stitching      \
                -lopencv_superres       \
                -lopencv_video          \
                -lopencv_videostab
OBJS          = ../../src/ardrone/ardrone.o \
                ../../src/ardrone/command.o \
                ../../src/ardrone/config.o  \
                ../../src/ardrone/udp.o     \
                ../../src/ardrone/tcp.o     \
                ../../src/ardrone/navdata.o \
                ../../src/ardrone/version.o \
                ../../src/ardrone/video.o   \
                ../../src/main.o
PROGRAM       = test.a

$(PROGRAM):     $(OBJS)
		$(CXX) $(OBJS) -o $(PROGRAM) $(CXXFLAGS) $(LDFLAGS) $(LIBS) 

clean:;         rm -f *.o *~ $(PROGRAM) $(OBJS)

install:        $(PROGRAM)
		install -s $(PROGRAM) $(DEST)
