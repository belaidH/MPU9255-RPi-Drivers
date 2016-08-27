all:
	g++ src/MPU9255.cpp examples/AccelGyro.cpp -o bin/AccelGyro -I./include -std=c++11 -lwiringPi