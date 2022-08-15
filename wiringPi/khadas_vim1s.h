/*------------------------------------------------------------------------------------------*/
/*																							*/
/*					WiringPi KHADAS_VIM1S Board Headler file									*/
/*																							*/
/*------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------*/
#ifndef __KHADAS_VIM1S_H__
#define __KHADAS_VIM1S_H__

/*------------------------------------------------------------------------------------------*/
#define VIM1S_GPIO_MASK					(0xFFFFFF00)
#define VIM1S_GPIO_BASE					0xfe004000
#define VIM1S_GPIO_PWM_BASE        0xfe058004

#define VIM1S_GPIO_PIN_BASE				430

#define VIM1S_GPIOB_PIN_START            (VIM1S_GPIO_PIN_BASE + 0)
#define VIM1S_GPIOB_PIN_END            (VIM1S_GPIO_PIN_BASE + 13)
#define VIM1S_GPIOC_PIN_START            (VIM1S_GPIO_PIN_BASE + 14)
#define VIM1S_GPIOC_PIN_END            (VIM1S_GPIO_PIN_BASE + 21)
#define VIM1S_GPIOE_PIN_START            (VIM1S_GPIO_PIN_BASE + 22)
#define VIM1S_GPIOE_PIN_END            (VIM1S_GPIO_PIN_BASE + 23)
#define VIM1S_GPIOD_PIN_START            (VIM1S_GPIO_PIN_BASE + 24)
#define VIM1S_GPIOD_PIN_END            (VIM1S_GPIO_PIN_BASE + 35)
#define VIM1S_GPIOH_PIN_START            (VIM1S_GPIO_PIN_BASE + 36)
#define VIM1S_GPIOH_PIN_END            (VIM1S_GPIO_PIN_BASE + 47)
#define VIM1S_GPIOX_PIN_START            (VIM1S_GPIO_PIN_BASE + 48)
#define VIM1S_GPIOX_PIN_END            (VIM1S_GPIO_PIN_BASE + 67)
#define VIM1S_GPIOZ_PIN_START            (VIM1S_GPIO_PIN_BASE + 68)
#define VIM1S_GPIOZ_PIN_END            (VIM1S_GPIO_PIN_BASE + 80)
#define VIM1S_GPIOT_PIN_START            (VIM1S_GPIO_PIN_BASE + 91)
#define VIM1S_GPIOT_PIN_END            (VIM1S_GPIO_PIN_BASE + 114)
#define VIM1S_GPIOM_PIN_START            (VIM1S_GPIO_PIN_BASE + 115)
#define VIM1S_GPIOM_PIN_END            (VIM1S_GPIO_PIN_BASE + 129)
#define VIM1S_GPIOY_PIN_START            (VIM1S_GPIO_PIN_BASE + 130)
#define VIM1S_GPIOY_PIN_END            (VIM1S_GPIO_PIN_BASE + 147)

#define VIM1S_GPIOD_INP_REG_OFFSET		0x030
#define VIM1S_GPIOD_OUTP_REG_OFFSET		0x031
#define VIM1S_GPIOD_FSEL_REG_OFFSET		0x032
#define VIM1S_GPIOD_PUEN_REG_OFFSET    	0x033
#define VIM1S_GPIOD_PUPD_REG_OFFSET		0x034
#define VIM1S_GPIOD_DS_REG_OFFSET   	    0x037
#define VIM1S_GPIOD_MUX_A_REG_OFFSET   	0x010
#define VIM1S_GPIOD_MUX_B_REG_OFFSET   	0x011

#define VIM1S_GPIOE_FSEL_REG_OFFSET		0x070
#define VIM1S_GPIOE_OUTP_REG_OFFSET		0x071
#define VIM1S_GPIOE_INP_REG_OFFSET		0x072
#define VIM1S_GPIOE_PUPD_REG_OFFSET		0x073
#define VIM1S_GPIOE_PUEN_REG_OFFSET    	0x074
#define VIM1S_GPIOE_DS_REG_OFFSET   	    0x077
#define VIM1S_GPIOE_MUX_C_REG_OFFSET   	0x012

#define VIM1S_GPIOZ_INP_REG_OFFSET		0x030
#define VIM1S_GPIOZ_OUTP_REG_OFFSET		0x031
#define VIM1S_GPIOZ_FSEL_REG_OFFSET		0x032
#define VIM1S_GPIOZ_PUEN_REG_OFFSET    	0x033
#define VIM1S_GPIOZ_PUPD_REG_OFFSET		0x034
#define VIM1S_GPIOZ_DS_REG_OFFSET   	    0x037
#define VIM1S_GPIOZ_MUX_5_REG_OFFSET   	0x006
#define VIM1S_GPIOZ_MUX_6_REG_OFFSET   	0x007

#define VIM1S_GPIOH_INP_REG_OFFSET		0x060
#define VIM1S_GPIOH_OUTP_REG_OFFSET		0x061
#define VIM1S_GPIOH_FSEL_REG_OFFSET		0x062
#define VIM1S_GPIOH_PUEN_REG_OFFSET    	0x063
#define VIM1S_GPIOH_PUPD_REG_OFFSET		0x064
#define VIM1S_GPIOH_DS_REG_OFFSET   	    0x067
#define VIM1S_GPIOH_MUX_B_REG_OFFSET   	0x00B
#define VIM1S_GPIOH_MUX_C_REG_OFFSET   	0x00C

#define VIM1S_GPIOC_INP_REG_OFFSET		0x080
#define VIM1S_GPIOC_OUTP_REG_OFFSET		0x081
#define VIM1S_GPIOC_FSEL_REG_OFFSET		0x082
#define VIM1S_GPIOC_PUEN_REG_OFFSET    	0x083
#define VIM1S_GPIOC_PUPD_REG_OFFSET		0x084
#define VIM1S_GPIOC_DS_REG_OFFSET   	    0x087
#define VIM1S_GPIOC_MUX_7_REG_OFFSET   	0x009

#define VIM1S_GPIOB_INP_REG_OFFSET		0x090
#define VIM1S_GPIOB_OUTP_REG_OFFSET		0x091
#define VIM1S_GPIOB_FSEL_REG_OFFSET		0x092
#define VIM1S_GPIOB_PUEN_REG_OFFSET    	0x093
#define VIM1S_GPIOB_PUPD_REG_OFFSET		0x094
#define VIM1S_GPIOB_DS_REG_OFFSET   	    0x097
#define VIM1S_GPIOB_MUX_0_REG_OFFSET   	0x000
#define VIM1S_GPIOB_MUX_1_REG_OFFSET   	0x001

#define VIM1S_GPIOX_INP_REG_OFFSET		0x040
#define VIM1S_GPIOX_OUTP_REG_OFFSET		0x041
#define VIM1S_GPIOX_FSEL_REG_OFFSET		0x042
#define VIM1S_GPIOX_PUEN_REG_OFFSET    	0x043
#define VIM1S_GPIOX_PUPD_REG_OFFSET		0x044
#define VIM1S_GPIOX_DS_REG_OFFSET   	    0x047
#define VIM1S_GPIOX_DS_EXT_REG_OFFSET   	0x048
#define VIM1S_GPIOX_MUX_2_REG_OFFSET   	0x003
#define VIM1S_GPIOX_MUX_3_REG_OFFSET   	0x004
#define VIM1S_GPIOX_MUX_4_REG_OFFSET   	0x005

#define VIM1S_PWM_EF_OFFSET        0
#define VIM1S_PWM_0_DUTY_CYCLE_OFFSET  0x00
#define VIM1S_PWM_1_DUTY_CYCLE_OFFSET  0x01
#define VIM1S_PWM_MISC_REG_01_OFFSET   0x02

/// PWM_MISC_REG_CD
#define VIM1S_PWM_1_INV_EN         ( 27 )
#define VIM1S_PWM_0_INV_EN         ( 26 )
#define VIM1S_PWM_1_CLK_EN         ( 23 )
#define VIM1S_PWM_1_CLK_DIV0       ( 16 )  /// 22 ~ 16
#define VIM1S_PWM_0_CLK_EN         ( 15 )
#define VIM1S_PWM_0_CLK_DIV0       ( 8 )   /// 14 ~ 8
#define VIM1S_PWM_1_CLK_SEL0       ( 6 )   /// 7 ~ 6
#define VIM1S_PWM_0_CLK_SEL0       ( 4 )   /// 5 ~ 4
#define VIM1S_PWM_1_DS_EN          ( 3 )
#define VIM1S_PWM_0_DS_EN          ( 2 )
#define VIM1S_PWM_1_EN         ( 1 )
#define VIM1S_PWM_0_EN         ( 0 )

#ifdef __cplusplus
extern "C"{
#endif

extern void init_khadas_vim1s(struct libkhadas *libwiring);

#ifdef __cplusplus
}
#endif

#endif /* __KHADAS_VIM1S_H__ */

