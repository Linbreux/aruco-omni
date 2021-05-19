all:
	g++ `pkg-config --cflags opencv4 --libs opencv4` -lpigpio detect_aruco.cpp lijn.cpp pid.cpp -o aruco1

detect:
	g++ `pkg-config --cflags opencv4 --libs opencv4` -lpigpio -lm  detect_aruco.cpp lijn.cpp pid.cpp motorcontrol.c -o aruco1

gen_code:
	g++ `pkg-config --cflags opencv4 --libs opencv4` gen_aruco_id.cpp -o gen_aruco
