vmc_regparam_auto: regparam_auto.cpp
	c++ -g -O0 regparam_auto.cpp `pkg-config --cflags vmc` `pkg-config --libs vmc` `pkg-config --libs log4cpp` -lpthread -o vmc_regparam_auto

clean:
	rm -f vmc_regparam_auto *.txt *.png *.o
