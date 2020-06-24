/*
 * (C) Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __S32R45_S32GEN1_MCRGM_REGS_H__
#define __S32R45_S32GEN1_MCRGM_REGS_H__

#define RGM_DES				(MC_RGM_BASE_ADDR + 0x0)
#define RGM_DES_POR			(0x00000001)

#define RGM_FES				(MC_RGM_BASE_ADDR + 0x8)
#define RGM_FES_EXT			(0x00000001)

#define RGM_PRST(per)			(MC_RGM_BASE_ADDR + 0x40 + \
					 ((per) * 0x8))
#define MC_RGM_PRST_CM7			(0)
#define PRST_PERIPH_n_RST(n)		BIT(n)
#define PRST_PERIPH_CM7n_RST(n)		PRST_PERIPH_n_RST(n)

#define RGM_PSTAT(per)			(MC_RGM_BASE_ADDR + 0x140 + \
					 ((per) * 0x8))
#define MC_RGM_PSTAT_CM7		(0)
#define PSTAT_PERIPH_n_STAT(n)		BIT(n)
#define PSTAT_PERIPH_CM7n_STAT(n)	PSTAT_PERIPH_n_STAT(n)

#define RGM_PERI_RESET_GROUP		0
#define RGM_CORES_RESET_GROUP		1
#define RGM_PFE_RESET_GROUP		2

#define RGM_CORE_RST(num)		BIT((num) + 1)
#define RGM_PERIPH_RST(num)		BIT(num)

/* S32G */
#define PRST_CM7_CLUSTER_0		0
#define PRST_CM7_CLUSTER_1		1
#define PRST_CM7_CLUSTER_2		2
#define PRST_DDR			3
#define PRST_PCIE_0_SERDES		4
#define PRST_PCIE_0_FUNC		5
#define PRST_PCIE_1_SERDES		16
#define PRST_PCIE_1_FUNC		17
#define PRST_PFE			128

#endif /* __S32R45_S32GEN1_MCRGM_REGS_H__ */

