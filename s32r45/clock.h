/*
 * (C) Copyright 2015-2016 Freescale Semiconductor, Inc.
 * (C) Copyright 2017-2020 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __S32R45_CLOCK_H
#define __S32R45_CLOCK_H

#include <sys/util.h>

enum mxc_clock {
	MXC_ARM_CLK = 0,
	MXC_BUS_CLK,
	MXC_PERIPHERALS_CLK,
	MXC_UART_CLK,
	MXC_USDHC_CLK,
	MXC_FEC_CLK,
	MXC_I2C_CLK,
	MXC_SYS6_CLK,
	MXC_QSPI_CLK,
	MXC_DCU_PIX_CLK,
	MXC_DSPI_CLK,
	MXC_XBAR_CLK,
	MXC_DDR_CLK,
};

enum pll_type {
        ARM_PLL = 0,
        PERIPH_PLL,
        ACCEL_PLL,
        DDR_PLL,
};

void clock_init(void);

int mux_source_clk_config(uintptr_t cgm_addr, uint8_t mux, uint8_t source);
void mux_div_clk_config(uintptr_t cgm_addr, uint8_t mux, uint8_t dc, uint8_t divider);

#endif /* __S32R45_CLOCK_H */
