robo01: robo01.cpp robodrive.cpp configureRobot.cpp commandLine.cpp robi.h physik.h
	c++ -g -O0 robo01.cpp robodrive.cpp configureRobot.cpp commandLine.cpp `pkg-config --cflags vmc` `pkg-config --libs vmc` `pkg-config --libs log4cpp` -lpthread -o robo01

measure: measure.cpp configureRobot.cpp commandLine.cpp robi.h
	c++ -g -O0 measure.cpp configureRobot.cpp commandLine.cpp `pkg-config --cflags vmc` `pkg-config --libs vmc` `pkg-config --libs log4cpp` -lpthread -o measure

clean:
	rm -f robo01 *.o
