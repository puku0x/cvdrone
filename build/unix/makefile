#sudo apt-get install build-essential
#sudo apt-get install ffmpeg
#sudo apt-get install libav-tools
#sudo apt-get install libopencv-dev

CXX           = g++
CXXFLAGS      = -O2 -Wall -D__STDC_CONSTANT_MACROS `pkg-config --libs --cflags opencv`
LIBS          = -lm                     \
                -lpthread               \
                -lavutil                \
                -lavformat              \
                -lavcodec               \
                -lswscale
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
