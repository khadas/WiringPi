#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

int main()
{
	int fd;
	int data;
	int address;

	if(-1 == wiringPiSetup()){
		printf("set up error");
		exit(1);
	}

	fd = wiringPiI2CSetup(address);
	
	if(-1 == fd){
		printf("I2C Set up error\n\n");
		exit(-1);
	}

	data = wiringPiI2CReadReg8(fd, 0x01);
	printf("data : %x\n\n" , data);

	exit(0);
}
