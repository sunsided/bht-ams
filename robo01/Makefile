robo01: robo01.cpp
	c++ -g -O0 robo01.cpp robodrive.cpp `pkg-config --cflags vmc` `pkg-config --libs vmc` `pkg-config --libs log4cpp` -lpthread -o robo01

clean:
	rm -f robo01 *.o
