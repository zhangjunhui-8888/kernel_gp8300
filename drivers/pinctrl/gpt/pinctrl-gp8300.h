/*
 *
 * Copyright (C) 2012 Maxime Ripard
 *
 * Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __PINCTRL_GP8300_H
#define __PINCTRL_GP8300_H
//
#define GROUP1_GPIO1_MUX_SEL 0x0
#define GROUP1_VCAP1_L8_MUX_SEL 0x1
#define GROUP1_VOUT_H8_MUX_SEL 0x2
#define GROUP1_VCAP2__H8_MUX_SEL 0x3

#define GROUP2_GPIO2_MUX_SEL 0x0 << 3
#define GROUP2_VCAP2_L8_MUX_SEL 0x1 << 3
#define GROUP2_VOUT_H8_MUX_SEL 0x2 << 3
#define GROUP2_VCAP1_H8_MUX_SEL 0x3 << 3

#define GROUP4_GPIO3_L4_MUX_SEL 0x0 << 9
#define GROUP4_SPI0_MUX_SEL 0x1 << 9

#define GROUP5_GPIO3_L4_MUX_SEL 0x0 << 12
#define GROUP5_SPI1_MUX_SEL 0x1 << 12
#define GROUP5_VCAP1_H8_MUX_SEL 0x2 << 12
#define GROUP5_VOUT_H8_MUX_SEL 0x3 << 12

#define GROUP6_GPIO3_H4_MUX_SEL 0x0 << 15
#define GROUP6_SPI2_MUX_SEL 0x1 << 15
#define GROUP6_VCAP1_HH4_MUX_SEL 0x2 << 15
#define GROUP6_VOUT_HH4_MUX_SEL 0x3 << 15

#define GROUP7_UART1_MUX_SEL 0x1 << 18
#define GROUP7_UART0_CTS_RTS_MUX_SEL 0x2 << 18

#define GROUP7_UART2_MUX_SEL 0x1 << 21
#define GROUP7_UART0_DTR_DSR_MUX_SEL 0x2 << 21

#define GROUP7_GPIO3_4_5_MUX_SEL 0x0 << 24
#define GROUP7_I2C1_MUX_SEL 0x1 << 24

#define GROUP7_GPIO3_6_7_MUX_SEL 0x0 << 27
#define GROUP7_I2C2_MUX_SEL 0x1 << 27

#define PINMUX_DEFAULT_CONFIG GROUP1_VCAP1_L8_MUX_SEL | GROUP2_VCAP1_H8_MUX_SEL |GROUP2_VCAP2_L8_MUX_SEL | GROUP7_I2C1_MUX_SEL | GROUP7_I2C2_MUX_SEL | GROUP4_SPI0_MUX_SEL 
#endif /* __PINCTRL_GP8300_H*/
