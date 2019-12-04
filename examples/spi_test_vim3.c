#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

int main()
{
	unsigned char data1 = 'a';
	unsigned char data2;
	int reg;

	if(-1 == wiringPiSetup()){
		printf("set up error");
		exit(1);
	}

	if(-1 == wiringPiSPISetup(0, 10000000)){
		printf("set up spi error");
		exit(-1);
	}
	reg = wiringPiSPIDataRW_khadas(0, &data1, $data2, 1);
	if(reg < 0){
		printf("RW error");
		exit(-1);
	}
	printf("data : %c\n\n", data2);

	exit(0);
}

