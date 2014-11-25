
all: pid_test

pid_test: pid.o pid_test.o motor.o daq.o
	gcc -o pid_test $^

pid.o: pid.c pid.h daq.h
	gcc -c pid.c

pid_test.o: pid_test.c pid.o
	gcc -c pid_test.c

motor.o: motor.c pid.h daq.h
	gcc -c motor.c
daq.o: daq.c daq.h
	gcc -c daq.c

clean:
	rm *.o pid_test
