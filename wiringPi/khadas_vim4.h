/*------------------------------------------------------------------------------------------*/
/*																							*/
/*					WiringPi KHADAS_VIM4 Board Headler file									*/
/*																							*/
/*------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------*/
#ifndef __KHADAS_VIM4_H__
#define __KHADAS_VIM4_H__

/*------------------------------------------------------------------------------------------*/
#define VIM4_GPIO_MASK					(0xFFFFFF00)
#define VIM4_GPIO_BASE					0xfe004000
#define VIM4_GPIO_PWM_BASE        0xfe05c000

#define VIM4_GPIO_PIN_BASE				355

#define VIM4_GPIOB_PIN_START            (VIM4_GPIO_PIN_BASE + 0)
#define VIM4_GPIOB_PIN_END            (VIM4_GPIO_PIN_BASE + 12)
#define VIM4_GPIOC_PIN_START            (VIM4_GPIO_PIN_BASE + 13)
#define VIM4_GPIOC_PIN_END            (VIM4_GPIO_PIN_BASE + 19)
#define VIM4_GPIOX_PIN_START            (VIM4_GPIO_PIN_BASE + 20)
#define VIM4_GPIOX_PIN_END            (VIM4_GPIO_PIN_BASE + 39)
#define VIM4_GPIOW_PIN_START            (VIM4_GPIO_PIN_BASE + 40)
#define VIM4_GPIOW_PIN_END            (VIM4_GPIO_PIN_BASE + 56)
#define VIM4_GPIOD_PIN_START            (VIM4_GPIO_PIN_BASE + 57)
#define VIM4_GPIOD_PIN_END            (VIM4_GPIO_PIN_BASE + 69)
#define VIM4_GPIOE_PIN_START            (VIM4_GPIO_PIN_BASE + 70)
#define VIM4_GPIOE_PIN_END            (VIM4_GPIO_PIN_BASE + 76)
#define VIM4_GPIOZ_PIN_START            (VIM4_GPIO_PIN_BASE + 77)
#define VIM4_GPIOZ_PIN_END            (VIM4_GPIO_PIN_BASE + 90)
#define VIM4_GPIOT_PIN_START            (VIM4_GPIO_PIN_BASE + 91)
#define VIM4_GPIOT_PIN_END            (VIM4_GPIO_PIN_BASE + 114)
#define VIM4_GPIOM_PIN_START            (VIM4_GPIO_PIN_BASE + 115)
#define VIM4_GPIOM_PIN_END            (VIM4_GPIO_PIN_BASE + 129)
#define VIM4_GPIOY_PIN_START            (VIM4_GPIO_PIN_BASE + 130)
#define VIM4_GPIOY_PIN_END            (VIM4_GPIO_PIN_BASE + 147)
#define VIM4_GPIOH_PIN_START            (VIM4_GPIO_PIN_BASE + 148)
#define VIM4_GPIOH_PIN_END            (VIM4_GPIO_PIN_BASE + 156)


#define VIM4_GPIOD_INP_REG_OFFSET		0x030
#define VIM4_GPIOD_OUTP_REG_OFFSET		0x031
#define VIM4_GPIOD_FSEL_REG_OFFSET		0x032
#define VIM4_GPIOD_PUEN_REG_OFFSET    	0x033
#define VIM4_GPIOD_PUPD_REG_OFFSET		0x034
#define VIM4_GPIOD_DS_REG_OFFSET   	    0x037
#define VIM4_GPIOD_MUX_A_REG_OFFSET   	0x00A
#define VIM4_GPIOD_MUX_B_REG_OFFSET   	0x00B


#define VIM4_GPIOE_FSEL_REG_OFFSET		0x03A
#define VIM4_GPIOE_OUTP_REG_OFFSET		0x039
#define VIM4_GPIOE_INP_REG_OFFSET		0x038
#define VIM4_GPIOE_PUPD_REG_OFFSET		0x03C
#define VIM4_GPIOE_PUEN_REG_OFFSET    	0x03B
#define VIM4_GPIOE_DS_REG_OFFSET   	    0x037
#define VIM4_GPIOE_MUX_C_REG_OFFSET   	0x00C

#define VIM4_GPIOZ_INP_REG_OFFSET		0x040
#define VIM4_GPIOZ_OUTP_REG_OFFSET		0x041
#define VIM4_GPIOZ_FSEL_REG_OFFSET		0x042
#define VIM4_GPIOZ_PUEN_REG_OFFSET    	0x043
#define VIM4_GPIOZ_PUPD_REG_OFFSET		0x044
#define VIM4_GPIOZ_DS_REG_OFFSET   	    0x047
#define VIM4_GPIOZ_MUX_5_REG_OFFSET   	0x005
#define VIM4_GPIOZ_MUX_6_REG_OFFSET   	0x006

#define VIM4_GPIOH_INP_REG_OFFSET		0x048
#define VIM4_GPIOH_OUTP_REG_OFFSET		0x049
#define VIM4_GPIOH_FSEL_REG_OFFSET		0x04A
#define VIM4_GPIOH_PUEN_REG_OFFSET    	0x04B
#define VIM4_GPIOH_PUPD_REG_OFFSET		0x04C
#define VIM4_GPIOH_DS_REG_OFFSET   	    0x04F
#define VIM4_GPIOH_MUX_8_REG_OFFSET   	0x008

#define VIM4_GPIOC_INP_REG_OFFSET		0x050
#define VIM4_GPIOC_OUTP_REG_OFFSET		0x051
#define VIM4_GPIOC_FSEL_REG_OFFSET		0x052
#define VIM4_GPIOC_PUEN_REG_OFFSET    	0x053
#define VIM4_GPIOC_PUPD_REG_OFFSET		0x054
#define VIM4_GPIOC_DS_REG_OFFSET   	    0x057
#define VIM4_GPIOC_MUX_7_REG_OFFSET   	0x007

#define VIM4_GPIOB_INP_REG_OFFSET		0x058
#define VIM4_GPIOB_OUTP_REG_OFFSET		0x059
#define VIM4_GPIOB_FSEL_REG_OFFSET		0x05A
#define VIM4_GPIOB_PUEN_REG_OFFSET    	0x05B
#define VIM4_GPIOB_PUPD_REG_OFFSET		0x05C
#define VIM4_GPIOB_DS_REG_OFFSET   	    0x05F
#define VIM4_GPIOB_MUX_0_REG_OFFSET   	0x000
#define VIM4_GPIOB_MUX_1_REG_OFFSET   	0x001

#define VIM4_GPIOX_INP_REG_OFFSET		0x060
#define VIM4_GPIOX_OUTP_REG_OFFSET		0x061
#define VIM4_GPIOX_FSEL_REG_OFFSET		0x062
#define VIM4_GPIOX_PUEN_REG_OFFSET    	0x063
#define VIM4_GPIOX_PUPD_REG_OFFSET		0x064
#define VIM4_GPIOX_DS_REG_OFFSET   	    0x067
#define VIM4_GPIOX_DS_EXT_REG_OFFSET   	0x068
#define VIM4_GPIOX_MUX_2_REG_OFFSET   	0x002
#define VIM4_GPIOX_MUX_3_REG_OFFSET   	0x003
#define VIM4_GPIOX_MUX_4_REG_OFFSET   	0x004

#define VIM4_GPIOT_INP_REG_OFFSET		0x070
#define VIM4_GPIOT_OUTP_REG_OFFSET		0x071
#define VIM4_GPIOT_FSEL_REG_OFFSET		0x072
#define VIM4_GPIOT_PUEN_REG_OFFSET    	0x073
#define VIM4_GPIOT_PUPD_REG_OFFSET		0x074
#define VIM4_GPIOT_DS_REG_OFFSET   	    0x077
#define VIM4_GPIOT_DS_EXT_REG_OFFSET   	0x078
#define VIM4_GPIOT_MUX_F_REG_OFFSET   	0x00F
#define VIM4_GPIOT_MUX_G_REG_OFFSET   	0x010
#define VIM4_GPIOT_MUX_H_REG_OFFSET   	0x011

#define VIM4_GPIOY_INP_REG_OFFSET		0x080
#define VIM4_GPIOY_OUTP_REG_OFFSET		0x081
#define VIM4_GPIOY_FSEL_REG_OFFSET		0x082
#define VIM4_GPIOY_PUEN_REG_OFFSET    	0x083
#define VIM4_GPIOY_PUPD_REG_OFFSET		0x084
#define VIM4_GPIOY_DS_REG_OFFSET   	    0x087
#define VIM4_GPIOY_DS_EXT_REG_OFFSET   	0x088
#define VIM4_GPIOY_MUX_J_REG_OFFSET   	0x013
#define VIM4_GPIOY_MUX_K_REG_OFFSET   	0x014
#define VIM4_GPIOY_MUX_L_REG_OFFSET   	0x015

#define VIM4_GPIOW_INP_REG_OFFSET		0x090
#define VIM4_GPIOW_OUTP_REG_OFFSET		0x091
#define VIM4_GPIOW_FSEL_REG_OFFSET		0x092
#define VIM4_GPIOW_PUEN_REG_OFFSET    	0x093
#define VIM4_GPIOW_PUPD_REG_OFFSET		0x094
#define VIM4_GPIOW_DS_REG_OFFSET   	    0x097
#define VIM4_GPIOW_DS_EXT_REG_OFFSET   	0x098
#define VIM4_GPIOW_MUX_M_REG_OFFSET   	0x016
#define VIM4_GPIOW_MUX_N_REG_OFFSET   	0x017
#define VIM4_GPIOW_MUX_O_REG_OFFSET   	0x018

#define VIM4_GPIOM_INP_REG_OFFSET		0x0A0
#define VIM4_GPIOM_OUTP_REG_OFFSET		0x0A1
#define VIM4_GPIOM_FSEL_REG_OFFSET		0x0A2
#define VIM4_GPIOM_PUEN_REG_OFFSET    	0x0A3
#define VIM4_GPIOM_PUPD_REG_OFFSET		0x0A4
#define VIM4_GPIOM_DS_REG_OFFSET   	    0x0A7
#define VIM4_GPIOM_DS_EXT_REG_OFFSET   	0x0A8
#define VIM4_GPIOM_MUX_D_REG_OFFSET   	0x00D
#define VIM4_GPIOM_MUX_E_REG_OFFSET   	0x00E

#define VIM4_PWM_EF_OFFSET        0
#define VIM4_PWM_0_DUTY_CYCLE_OFFSET  0x00
#define VIM4_PWM_1_DUTY_CYCLE_OFFSET  0x01
#define VIM4_PWM_MISC_REG_01_OFFSET   0x02

/// PWM_MISC_REG_CD
#define VIM4_PWM_1_INV_EN         ( 27 )
#define VIM4_PWM_0_INV_EN         ( 26 )
#define VIM4_PWM_1_CLK_EN         ( 23 )
#define VIM4_PWM_1_CLK_DIV0       ( 16 )  /// 22 ~ 16
#define VIM4_PWM_0_CLK_EN         ( 15 )
#define VIM4_PWM_0_CLK_DIV0       ( 8 )   /// 14 ~ 8
#define VIM4_PWM_1_CLK_SEL0       ( 6 )   /// 7 ~ 6
#define VIM4_PWM_0_CLK_SEL0       ( 4 )   /// 5 ~ 4
#define VIM4_PWM_1_DS_EN          ( 3 )
#define VIM4_PWM_0_DS_EN          ( 2 )
#define VIM4_PWM_1_EN         ( 1 )
#define VIM4_PWM_0_EN         ( 0 )

#ifdef __cplusplus
extern "C"{
#endif

extern void init_khadas_vim4(struct libkhadas *libwiring);

#ifdef __cplusplus
}
#endif

#endif /* __KHADAS_VIM4_H__ */

