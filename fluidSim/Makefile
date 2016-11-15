CXX    = g++
CFLAGS = -O2
INCLUDES = -I ./

TARGET = main.out
ifeq ($(OS),Windows_NT) 
	LIBS_GL = -lfreeglut -lglu32 -lopengl32		#Windows
	TARGET = main.exe	
else ifeq ($(shell uname -s),Darwin)	
	LIBS_GL = -framework OpenGL -framework GLUT	#Mac
else	
	LIBS_GL = -lglut -lGL -lGLU			#Linux(defalut)
endif

SOURCES=$(shell find . -name '*.cpp')
HEADERS=$(shell find . -name '*.h')
OBJS =$(SOURCES:%.cpp=%.o)

all: $(TARGET)
					
$(TARGET): $(OBJS) $(HEADERS)
	$(CXX)  -o $@ $(OBJS) $(LIBS_GL)

clean:
	-rm -f $(OBJS)
.cpp.o:
	$(CXX) $(INCLUDES) -c $<
