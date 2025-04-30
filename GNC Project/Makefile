# Makefile

CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2

# Sources and headers
SRCS = main.cpp KalmanFilter.cpp PIDController.cpp
OBJS = $(SRCS:.cpp=.o)
DEPS = KalmanFilter.h PIDController.h

# Output binary name
TARGET = main

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp $(DEPS)
	$(CXX) $(CXXFLAGS) -c $<

clean:
	rm -f *.o $(TARGET)