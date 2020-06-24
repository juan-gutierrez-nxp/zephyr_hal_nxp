// SPDX-License-Identifier:     GPL-2.0+
/*
 * Copyright 2019-2020 NXP
 */
#ifndef S32GEN1_CLOCK_H
#define S32GEN1_CLOCK_H

void s32gen1_setup_fxosc(void);
void s32gen1_enable_partition_blocks(uint32_t partition_n, uint32_t *blocks,
				     size_t n_blocks);
int s32gen1_program_pll(enum pll_type pll, uint32_t refclk_freq, uint32_t phi_nr,
			uint64_t freq[], uint32_t dfs_nr, uint32_t dfs[][DFS_PARAMS_Nr],
			uint32_t plldv_rdiv, uint32_t plldv_mfi, uint32_t pllfd_mfn);

#endif
