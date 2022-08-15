/*--------------------------------------------------------------------------------------*/
/*																						*/
/*				WiringPi KHADAS VIM1S Board control file									*/
/*																						*/
/*--------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <sys/mman.h>
#include <sys/utsname.h>
#include <string.h>

#include "softPwm.h"
#include "softTone.h"
#include "wiringPi.h"
#include "khadas_vim1s.h"

/*--------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------------*/
/*								WiringPi gpio map define								*/
/*--------------------------------------------------------------------------------------*/

static const int pinToGpio_rev[64] = {
    //wiringPi number to native gpio number
     -1,470,        //   0 | 1  :
    457,456,        //   2 | 3  :                    
    455,454,        //   4 | 5  :
    461,460,        //   6 | 7  :
    509,510,        //   8 | 9  :
    499,504,        //  10 | 11 :
    500,498,        //  12 | 13 :
    501,503,        //  14 | 15 :
    506,508,        //  16 | 17 :
    475, -1,        //  18 | 19 :
     -1, -1,        //  20 | 21 :
     -1, -1,        //  22 | 23 :
     -1, -1,        //  24 | 25 :
     -1, -1,        //  26 | 27 :
     -1, -1,        //  28 | 29 :
     -1, -1,        //  30 | 31 :
    // Padding:
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, //32to47
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, //48to63

};

static const int phyToGpio_rev[64] = {
    //physical header pin number to native gpio number
     -1,                //  0
     -1, -1,            //   1 | 21 :                        5V | GND
     -1,461,            //   2 | 22 :                        5V | GPIOD_7(I2CM_F_SCL)
     -1,460,            //   3 | 23 :                    USB_DM | GPIOD_6(I2CM_F_SDA)
     -1, -1,            //   4 | 24 :                    USB_DP | GND
     -1,509,            //   5 | 25 :                       GND | GPIOZ_11(I2CM_A_SCL)
     -1,510,            //   6 | 26 :                    MCU3.3 | GPIOZ_12(I2CM_A_SDA)
     -1, -1,            //   7 | 27 :                   MCUNrST | 3.3V
     -1, -1,            //   8 | 28 :                   MCUSWIM | GND
     -1,499,            //   9 | 29 :                       GND | GPIOZ_1
     -1,504,            //  10 | 30 :                      ADC0 | GPIOZ_6
     -1,500,            //  11 | 31 :                      1.8V | GPIOZ_2
     -1,498,            //  12 | 32 :                      ADC1 | GPIOZ_0
    470,501,            //  13 | 33 :         (SPDIFOUT)GPIOH_4 | GPIOZ_3
     -1, -1,            //  14 | 34 :                       GND | GND
    457,503,            //  15 | 35 :        (GPIOD_3)UART_RX_E | PWM-F(GPIOZ_5)
    456,506,            //  16 | 36 :        (GPIOD_2)UART_TX_E | GPIOZ_8
     -1,508,            //  17 | 37 :                       GND | GPIOZ_10
    455, -1,            //  18 | 38 :         (GPIOD_1)Linux_RX | 
    454,475,            //  19 | 39 :         (GPIOD_0)Linux_TX | GPIOH_9
     -1, -1,            //  20 | 40 :                      3.3V | GND
     //Not used
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1,                                        //41-63
};


static uint16_t pwmPinToALT = 4;

static uint16_t pwmPinToRange = 0;


/*--------------------------------------------------------------------------------------*/
/*																						*/
/*								Global variable define									*/
/*																						*/
/*--------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------------*/
/*							wiringPi Pinmap control array								*/
/*--------------------------------------------------------------------------------------*/
static const int *pinToGpio, *phyToGpio;

/*	ADC file descriptor	*/
static char *adcFds[2];

/*	GPIO mmap control	*/
static volatile uint32_t *gpio = NULL;
static volatile uint32_t *pwm = NULL;

/* 	wiringPi Global library	*/
static struct libkhadas *lib = NULL;

/*--------------------------------------------------------------------------------------*/
/*								Function prototype define								*/
/*--------------------------------------------------------------------------------------*/

static int  gpioToGPSETReg  (int pin);
static int  gpioToGPLEVReg  (int pin);
static int  gpioToPUENReg   (int pin);
static int  gpioToPUPDReg   (int pin);
static int  gpioToShiftReg  (int pin);
static int  gpioToGPFSELReg (int pin);
static int	gpioToDSReg		(int pin);
static int	gpioToMuxReg	(int pin);

/*--------------------------------------------------------------------------------------*/
/*								 wiringPi core function									*/
/*--------------------------------------------------------------------------------------*/
static int      _getModeToGpio      (int mode, int pin);
static void		_setPadDrive		(int pin, int value);
static int		_getPadDrive		(int pin);
static void     _pinMode        (int pin, int mode);
static int      _getAlt         (int pin);
static int      _getPUPD        (int pin);
static void     _pullUpDnControl    (int pin, int pud);
static int      _digitalRead        (int pin);
static void     _digitalWrite       (int pin, int value);
static int      _analogRead     (int pin);
static void     _digitalWriteByte   (const int value);
static unsigned int _digitalReadByte    (void);
static int      _pwmWrite       (int pin, int value);
static void     _pwmSetRange        (unsigned int range);
static void     _pwmSetClock        (int divisor);

/*--------------------------------------------------------------------------------------*/
/*								board init function										*/
/*--------------------------------------------------------------------------------------*/
static  int init_gpio_mmap (void);
static  void init_adc_fds   (void);
void init_khadas_vim1s(struct libkhadas *libwiring);

/*--------------------------------------------------------------------------------------*/
/*							offset to the GPIO Set regsiter								*/
/*--------------------------------------------------------------------------------------*/
static int gpioToGPSETReg (int pin)
{
    if(pin >= VIM1S_GPIOD_PIN_START && pin <= VIM1S_GPIOD_PIN_END)
		return VIM1S_GPIOD_OUTP_REG_OFFSET;
    if(pin >= VIM1S_GPIOH_PIN_START && pin <= VIM1S_GPIOH_PIN_END)
		return VIM1S_GPIOH_OUTP_REG_OFFSET;
    if(pin >= VIM1S_GPIOC_PIN_START && pin <= VIM1S_GPIOC_PIN_END)
		return VIM1S_GPIOC_OUTP_REG_OFFSET;
	if(pin >= VIM1S_GPIOZ_PIN_START && pin <= VIM1S_GPIOZ_PIN_END)
		return VIM1S_GPIOZ_OUTP_REG_OFFSET;
	return -1; 
}

/*------------------------------------------------------------------------------------------*/
/*                          offset to the GPIO Input regsiter                               */
/*------------------------------------------------------------------------------------------*/
static int gpioToGPLEVReg (int pin)
{
	if(pin >= VIM1S_GPIOC_PIN_START && pin <= VIM1S_GPIOC_PIN_END)
		return VIM1S_GPIOC_INP_REG_OFFSET;
	if(pin >= VIM1S_GPIOZ_PIN_START && pin <= VIM1S_GPIOZ_PIN_END)
		return VIM1S_GPIOZ_INP_REG_OFFSET;
	if(pin >= VIM1S_GPIOH_PIN_START && pin <= VIM1S_GPIOH_PIN_END)
		return VIM1S_GPIOH_INP_REG_OFFSET;
	if(pin >= VIM1S_GPIOD_PIN_START && pin <= VIM1S_GPIOD_PIN_END)
		return VIM1S_GPIOD_INP_REG_OFFSET;
	return -1; 
}

/*------------------------------------------------------------------------------------------*/
/*                      offset to the GPIO Pull up/down enable regsiter                     */
/*------------------------------------------------------------------------------------------*/
static int gpioToPUENReg(int pin)
{
    if(pin >= VIM1S_GPIOC_PIN_START && pin <= VIM1S_GPIOC_PIN_END)
		return VIM1S_GPIOC_PUEN_REG_OFFSET;
    if(pin >= VIM1S_GPIOH_PIN_START && pin <= VIM1S_GPIOH_PIN_END)
		return VIM1S_GPIOH_PUEN_REG_OFFSET;
	if(pin >= VIM1S_GPIOZ_PIN_START && pin <= VIM1S_GPIOZ_PIN_END)
		return VIM1S_GPIOZ_PUEN_REG_OFFSET;
	if(pin >= VIM1S_GPIOD_PIN_START && pin <= VIM1S_GPIOD_PIN_END)
		return VIM1S_GPIOD_PUEN_REG_OFFSET;
	return -1;
}

/*------------------------------------------------------------------------------------------*/
/*                          offset to the GPIO Pull up/down regsiter                        */
/*------------------------------------------------------------------------------------------*/
static int gpioToPUPDReg(int pin)
{
    if(pin >= VIM1S_GPIOH_PIN_START && pin <= VIM1S_GPIOH_PIN_END)
		return VIM1S_GPIOH_PUPD_REG_OFFSET;
    if(pin >= VIM1S_GPIOC_PIN_START && pin <= VIM1S_GPIOC_PIN_END)
		return VIM1S_GPIOC_PUPD_REG_OFFSET;
	if(pin >= VIM1S_GPIOZ_PIN_START && pin <= VIM1S_GPIOZ_PIN_END)
		return VIM1S_GPIOZ_PUPD_REG_OFFSET;
	if(pin >= VIM1S_GPIOD_PIN_START && pin <= VIM1S_GPIOD_PIN_END)
		return VIM1S_GPIOD_PUPD_REG_OFFSET;
	return -1;
}

/*------------------------------------------------------------------------------------------*/
/*                          offset to the GPIO bit                                          */
/*------------------------------------------------------------------------------------------*/
static int gpioToShiftReg (int pin)
{
    if(pin >= VIM1S_GPIOZ_PIN_START && pin <= VIM1S_GPIOZ_PIN_END)
		return pin - VIM1S_GPIOZ_PIN_START;
    if(pin >= VIM1S_GPIOC_PIN_START && pin <= VIM1S_GPIOC_PIN_END)
		return pin - VIM1S_GPIOC_PIN_START;
	if(pin >= VIM1S_GPIOH_PIN_START && pin <= VIM1S_GPIOH_PIN_END)
		return pin - VIM1S_GPIOH_PIN_START;
	if(pin >= VIM1S_GPIOD_PIN_START && pin <= VIM1S_GPIOD_PIN_END)
		return pin - VIM1S_GPIOD_PIN_START;
	return -1;
}

/*------------------------------------------------------------------------------------------*/
/*                          offset to the GPIO Function register                            */
/*------------------------------------------------------------------------------------------*/

static int gpioToGPFSELReg(int pin)
{
    if(pin >= VIM1S_GPIOD_PIN_START && pin <= VIM1S_GPIOD_PIN_END)
	    return VIM1S_GPIOD_FSEL_REG_OFFSET;
    if(pin >= VIM1S_GPIOH_PIN_START && pin <= VIM1S_GPIOH_PIN_END)
	    return VIM1S_GPIOH_FSEL_REG_OFFSET;
    if(pin >= VIM1S_GPIOC_PIN_START && pin <= VIM1S_GPIOC_PIN_END)
	    return VIM1S_GPIOC_FSEL_REG_OFFSET;
    if(pin >= VIM1S_GPIOZ_PIN_START && pin <= VIM1S_GPIOZ_PIN_END)
	    return VIM1S_GPIOZ_FSEL_REG_OFFSET;
	return -1;
}

/*------------------------------------------------------------------------------------------*/
/*				offset to the GPIO Drive Strength register									*/
/*------------------------------------------------------------------------------------------*/
static int gpioToDSReg (int pin)
{
    if(pin >= VIM1S_GPIOD_PIN_START && pin <= VIM1S_GPIOD_PIN_END)
	    return VIM1S_GPIOD_DS_REG_OFFSET;
    if(pin >= VIM1S_GPIOC_PIN_START && pin <= VIM1S_GPIOC_PIN_END)
	    return VIM1S_GPIOC_DS_REG_OFFSET;
	if(pin >= VIM1S_GPIOH_PIN_START && pin <= VIM1S_GPIOH_PIN_END)
		return VIM1S_GPIOH_DS_REG_OFFSET;
	if(pin >= VIM1S_GPIOZ_PIN_START && pin <= VIM1S_GPIOZ_PIN_END)
		return VIM1S_GPIOZ_DS_REG_OFFSET;
	return -1;
}

/*------------------------------------------------------------------------------------------*/
/*							offset to the GPIO Pin Mux register								*/
/*------------------------------------------------------------------------------------------*/
static int gpioToMuxReg(int pin)
{	
	switch(pin){
		case VIM1S_GPIOD_PIN_START  ...VIM1S_GPIOD_PIN_START + 7:
			return  VIM1S_GPIOD_MUX_A_REG_OFFSET;
		case VIM1S_GPIOD_PIN_START + 8  ...VIM1S_GPIOD_PIN_END:
			return  VIM1S_GPIOD_MUX_B_REG_OFFSET;
		case VIM1S_GPIOC_PIN_START  ...VIM1S_GPIOC_PIN_END:
			return  VIM1S_GPIOC_MUX_7_REG_OFFSET;
		case VIM1S_GPIOH_PIN_START  ...VIM1S_GPIOH_PIN_START + 7:
			return  VIM1S_GPIOH_MUX_B_REG_OFFSET;
		case VIM1S_GPIOH_PIN_START + 8  ...VIM1S_GPIOH_PIN_END:
			return  VIM1S_GPIOH_MUX_C_REG_OFFSET;
		case VIM1S_GPIOZ_PIN_START  ...VIM1S_GPIOZ_PIN_START + 7:
			return  VIM1S_GPIOZ_MUX_5_REG_OFFSET;
		case VIM1S_GPIOZ_PIN_START + 8  ...VIM1S_GPIOZ_PIN_END:
			return  VIM1S_GPIOZ_MUX_6_REG_OFFSET;
	}
	return -1;
}


/*------------------------------------------------------------------------------------------*/
static int _getModeToGpio(int mode, int pin)
{
	int retPin = -1;
	switch(mode){
		/* Native gpio number */
		case MODE_GPIO:
			retPin = pin;
			break;
		/* Native gpio number for sysfs */
		case MODE_GPIO_SYS:
			retPin = lib->sysFds[pin] != -1 ? pin : -1;
			break;
		/* wiringPi number */
		case MODE_PINS:
			retPin = pin < 64 ? pinToGpio[pin] : -1;
			break;
		/* header pin number */
		case MODE_PHYS:
			retPin = pin < 64 ? phyToGpio[pin] : -1;
			break;
		default:
			msg(MSG_WARN, "%s : Unknown Mode %d\n", __func__, mode);
			return -1;
	}
	return retPin;
}

/*------------------------------------------------------------------------------------------*/
static void _setPadDrive(int pin, int value)
{
	int ds, shift;

	if(lib->mode == MODE_GPIO_SYS)
		return;

	if((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return;
	
	if(value < 0 || value > 3){
		msg(MSG_WARN, "%s : Invalid value %d (Must be 0 ~ 3)\n", __func__, value);
		return;
	}

	ds = gpioToDSReg(pin);
	shift = gpioToShiftReg(pin);
	if (shift>=16) shift -=16;
	shift *= 2;
	
	*(gpio + ds) &= ~(0b11 << shift);
	*(gpio + ds) |= (value << shift);
}

/*-----------------------------------------------------------------------------------------*/
static int _getPadDrive(int pin)
{	
	int ds, shift;

	if(lib->mode == MODE_GPIO_SYS)
		return -1;

	if((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	ds = gpioToDSReg(pin);
	shift = gpioToShiftReg(pin);
	if (shift>=16) shift -=16;
	shift *= 2;

	return (*(gpio + ds) >> shift) & 0b11;
}

/*------------------------------------------------------------------------------------------*/
static void _pinMode(int pin, int mode)
{
	int fsel, mux, shift, origPin = pin;
	int alt;

	if (lib->mode == MODE_GPIO_SYS)
		return;
	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return;

	softPwmStop  (origPin);
	softToneStop (origPin);

	fsel  = gpioToGPFSELReg(pin);
	shift = gpioToShiftReg (pin);
	mux = gpioToMuxReg(pin);

	switch (mode) {
		case INPUT:
			*(gpio + fsel) = (*(gpio + fsel) |  (1 << shift));
			break;
		case OUTPUT:
			*(gpio + fsel) = (*(gpio + fsel) & ~(1 << shift));
			break;
		case SOFT_PWM_OUTPUT:
			softPwmCreate (pin, 0, 100);
			break;
		case SOFT_TONE_OUTPUT:
			softToneCreate (pin);
			break;
		case PWM_OUTPUT:
			alt = pwmPinToALT;
			*(gpio + mux) = (*(gpio + mux) & ~(0xF << (shift*4))) | (alt << (shift*4));
			_pwmSetClock(120);
			_pwmSetRange(500);
			break;
		default:
			msg(MSG_WARN, "%s : Unknown Mode %d\n", __func__, mode);
			break;
	}
}

/*------------------------------------------------------------------------------------------*/
static int _getAlt(int pin)
{
	int fsel, mux, shift, target, mode;
	
	if(lib->mode == MODE_GPIO_SYS)
		return 0;
	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return 2;

	fsel = gpioToGPFSELReg(pin);
	mux    = gpioToMuxReg(pin);
	target = shift = gpioToShiftReg(pin);

	while(target >= 8) {
		target -= 8;
	}

	mode = (*(gpio + mux) >> (target * 4)) & 0xF;
	return  mode ? mode + 1 : (*(gpio + fsel) & (1 << shift)) ? 0 : 1;
}

/*------------------------------------------------------------------------------------------*/
static int _getPUPD(int pin)
{
	int puen, pupd, shift;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;
	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;
	
	puen  = gpioToPUENReg(pin);
	pupd  = gpioToPUPDReg(pin);
	shift = gpioToShiftReg(pin);
	
	if(*(gpio + puen) & (1 << shift)){
		return *(gpio + pupd) & (1 << shift) ? 1 : 2;
	}else{
		return 0;
	}
}

/*------------------------------------------------------------------------------------------*/
static void _pullUpDnControl(int pin, int pud)
{
	int shift = 0;
	if (lib->mode == MODE_GPIO_SYS)
		return;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)	
		return;
	
	shift = gpioToShiftReg(pin);

	if(pud){
		//Enable Pull/Pull-down resister
		*(gpio + gpioToPUENReg(pin)) =
			(*(gpio + gpioToPUENReg(pin)) | (1 << shift));

		if (pud == PUD_UP)
			*(gpio + gpioToPUPDReg(pin)) =
				(*(gpio + gpioToPUPDReg(pin)) | (1 << shift));

		else
			*(gpio + gpioToPUPDReg(pin)) =
				(*(gpio + gpioToPUPDReg(pin)) & ~(1 << shift));
	}else //Disable Pull/Pull-down resister
		*(gpio + gpioToPUENReg(pin)) =
			(*(gpio + gpioToPUENReg(pin)) & ~(1 << shift));
}

/*------------------------------------------------------------------------------------------*/
static int _digitalRead(int pin)
{
	char c;

	if (lib->mode == MODE_GPIO_SYS) {
		if (lib->sysFds[pin] == -1)
			return LOW;

		lseek(lib->sysFds[pin], 0L, SEEK_SET);
		read(lib->sysFds[pin], &c, 1);

		return (c=='0') ? LOW : HIGH;
	}

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return 0;
	if ((*(gpio + gpioToGPLEVReg(pin)) & (1 << gpioToShiftReg(pin))) != 0)
		return HIGH;
	else
		return LOW;
}

/*------------------------------------------------------------------------------------------*/
static void _digitalWrite(int pin, int value)
{
	if (lib->mode == MODE_GPIO_SYS) {
		if (lib->sysFds[pin] != -1) {
			if(value == LOW)
				write (lib->sysFds[pin], "0\n", 2);
			else
				 write (lib->sysFds[pin], "1\n", 2);
		}
		return;
	}

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return;

	if (value == LOW)
		*(gpio + gpioToGPSETReg(pin)) &= ~(1 << gpioToShiftReg(pin));
	else
		*(gpio + gpioToGPSETReg(pin)) |=  (1 << gpioToShiftReg(pin));
}

/*------------------------------------------------------------------------------------------*/
static int _analogRead (int UNU pin)
{
	char value[5] = {0,};
	if (lib->mode == MODE_GPIO_SYS)
		return -1;
	switch (pin) {
		case 19: pin = 0; break;
		case 20: pin = 1; break;
		default: return 0;
	}
	if (adcFds [pin] == -1)
		return -1;
	lseek(adcFds [pin], 0L, SEEK_SET);
	if (read(adcFds [pin], &value[0], 4) < 0) {
		msg(MSG_WARN, "%s: Error occurs when it reads from ADC file descriptor. \n", __func__);
		return -1;
	}
	return  atoi(value);	
}

/*-------------------------------------------------------------------------------------------*/
static void _digitalWriteByte(const int UNU value)
{
	return;
}
static unsigned int _digitalReadByte (void)
{
	return -1;
}

/*----------------------------------------------------------------------------*/
// PWM signal ___-----------___________---------------_______-----_
//               <--value-->           <----value---->
//               <-------range--------><-------range-------->
// PWM frequency == (PWM clock) / range
/*----------------------------------------------------------------------------*/
static void _pwmSetRange (unsigned int range)
{
	range = range & 0xFFFF;
	pwmPinToRange = range;
}

/*----------------------------------------------------------------------------*/
// Internal clock == 24MHz
// PWM clock == (Internal clock) / divisor
// PWM frequency == (PWM clock) / range
/*----------------------------------------------------------------------------*/
static void _pwmSetClock (int divisor)
{       
	if((divisor < 1) || (divisor > 128)){
		msg(MSG_ERR,
				"Set the clock prescaler (divisor) to 1 or more and 128 or less.: %s\n",   
				strerror (errno));
	}
	divisor = (divisor - 1);

		*( pwm + VIM1S_PWM_MISC_REG_01_OFFSET ) = \
			(1 << VIM1S_PWM_1_CLK_EN) \
			| ( divisor << VIM1S_PWM_1_CLK_DIV0) \
			| (1 << VIM1S_PWM_0_CLK_EN) \
			| ( divisor << VIM1S_PWM_0_CLK_DIV0) \ 
			| (0 << VIM1S_PWM_1_CLK_SEL0) \
			| (0 << VIM1S_PWM_0_CLK_SEL0) \
			| (1 << VIM1S_PWM_1_EN) \
			| (1 << VIM1S_PWM_0_EN);
}

/*----------------------------------------------------------------------------*/
// PWM signal ___-----------___________---------------_______-----_
//               <--value-->           <----value---->
//               <-------range--------><-------range-------->
/*----------------------------------------------------------------------------*/
static int _pwmWrite (int pin, int value)
{
    /**
     * @todo Add node
     * struct wiringPiNodeStruct *node = wiringPiNodes;
     */

    if (lib->mode == MODE_GPIO_SYS){
		printf("MODE_GPIO_SYS\n");
        return -1;
	}

    if ((pin = _getModeToGpio(lib->mode, pin)) < 0){
		printf("error: lib->mode, pin\n");
        return -1;
	}

    uint16_t range  = pwmPinToRange;

    if( value > range ) {
        value = range;
    }

	printf("address:0x%p,range:%d\n", pwm,range);
    *(pwm + VIM1S_PWM_1_DUTY_CYCLE_OFFSET) = (value << 16) | (range - value);
    *(pwm + VIM1S_PWM_0_DUTY_CYCLE_OFFSET) = (value << 16) | (range - value);

    return 0;
}


/*------------------------------------------------------------------------------------------*/
static int init_gpio_mmap(void)
{
	int fd;

	/* GPIO mmap setup */
		if ((fd = open ("/dev/gpiomem", O_RDWR | O_SYNC) ) < 0)
			return msg (MSG_ERR,
					"wiringPiSetup: Unable to open /dev/gpiomem: %s\n",
					strerror (errno));

	gpio  = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE,
						MAP_SHARED, open ("/dev/gpiomem", O_RDWR | O_SYNC), 0xfe004000);

	if ((int32_t)gpio == -1)
		return msg (MSG_ERR,
				"wiringPiSetup: mmap (GPIO) failed: %s\n",
				strerror (errno));

//	pwm = ( uint32_t * )mmap( 0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, VIM1S_GPIO_PWM_BASE);

//	if( (int32_t)pwm == -1 )
//		msg(MSG_ERR, "wiringPiSetup: mmap (PWM) failed: %s \n", strerror (errno));

	return 0;
}

/*------------------------------------------------------------------------------------------*/
static void init_adc_fds(void)
{
	const char *AIN0_NODE, *AIN1_NODE;

	/* ADC node setup */
	AIN0_NODE = "/sys/bus/iio/devices/iio:device0/in_voltage6_raw";
	AIN1_NODE = "/sys/bus/iio/devices/iio:device0/in_voltage3_raw";

	adcFds[0] = open(AIN0_NODE, O_RDONLY);
	adcFds[1] = open(AIN1_NODE, O_RDONLY);
}

/*------------------------------------------------------------------------------------------*/
void init_khadas_vim1s(struct libkhadas *libwiring)
{
	init_gpio_mmap();

	init_adc_fds();
	
	pinToGpio = pinToGpio_rev;
	phyToGpio = phyToGpio_rev;

	/* wiringPi core function initialize */
	libwiring->getModeToGpio    = _getModeToGpio;
	libwiring->setPadDrive      = _setPadDrive;
	libwiring->getPadDrive      = _getPadDrive;
	libwiring->pinMode      	= _pinMode;
	libwiring->getAlt       	= _getAlt;
	libwiring->getPUPD      	= _getPUPD;
	libwiring->pullUpDnControl  = _pullUpDnControl;
	libwiring->digitalRead      = _digitalRead;
	libwiring->digitalWrite     = _digitalWrite;
	libwiring->analogRead       = _analogRead;
	libwiring->digitalWriteByte = _digitalWriteByte;
	libwiring->digitalReadByte  = _digitalReadByte;
	libwiring->pwmWrite			= _pwmWrite;
	libwiring->pwmSetRange		= _pwmSetRange;
	libwiring->pwmSetClock		= _pwmSetClock;


	/* specify pin base number */
	libwiring->pinBase = VIM1S_GPIO_PIN_BASE;

	/* global variable setup */
	lib = libwiring;
}

/*------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------*/
