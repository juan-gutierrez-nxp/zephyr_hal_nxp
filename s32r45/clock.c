// SPDX-License-Identifier:     GPL-2.0+
/*
 * (C) Copyright 2018-2020 NXP
 */

#include <mc_cgm_regs.h>
#include <mc_me_regs.h>
#include <mc_rgm_regs.h>
#include <clock.h>

#define readb(a)                  (*(volatile unsigned char *)(a))
#define readw(a)                  (*(volatile unsigned short *)(a))
#define readl(a)                  (*(volatile unsigned int *)(a))
#define readq(a)                  (*(volatile unsigned long long *)(a))

#define writeb(v,a)                (*(volatile unsigned char *)(a) = (v))
#define writew(v,a)                (*(volatile unsigned short *)(a) = (v))
#define writel(v,a)                (*(volatile unsigned int *)(a) = (v))
#define writeq(v,a)                (*(volatile unsigned long long *)(a) = (v))

static void mc_me_wait_update(uint32_t partition_n, unsigned long address, uint32_t mask)
{
	writel(mask, MC_ME_PRTN_N_PUPD(partition_n));
	writel(MC_ME_CTL_KEY_KEY, (MC_ME_BASE_ADDR));
	writel(MC_ME_CTL_KEY_INVERTEDKEY, (MC_ME_BASE_ADDR));

	while (readl(address) & mask)
		;
}

static uint32_t get_blocks_mask(uint32_t *blocks, size_t n_blocks)
{
	size_t i;
	uint32_t blocks_mask = 0U;

	for (i = 0U; i < n_blocks; i++)
		blocks_mask |= MC_ME_PRTN_N_REQ(blocks[i]);

	return blocks_mask;
}

void s32gen1_enable_partition_blocks(uint32_t partition_n, uint32_t *blocks,
				     size_t n_blocks)
{
	uint32_t blocks_mask = get_blocks_mask(blocks, n_blocks);

	writel(readl(MC_ME_PRTN_N_PCONF(partition_n)) | MC_ME_PRTN_N_PCE,
	       MC_ME_PRTN_N_PCONF(partition_n));

	/*
	 * Partition 0 is enabled by default. The second activation of a
	 * partition will be blocked, unless it has been previously disabled.
	 */
	if (partition_n != 0) {
		mc_me_wait_update(partition_n, MC_ME_PRTN_N_PUPD(partition_n),
				  MC_ME_PRTN_N_PCUD);

		/* Unlock RDC register write */
		writel(RD_CTRL_UNLOCK_MASK, RDC_RD_N_CTRL(partition_n));

		/* Enable the XBAR interface */
		writel(readl(RDC_RD_N_CTRL(partition_n)) &
		       ~RD_XBAR_DISABLE_MASK, RDC_RD_N_CTRL(partition_n));

		/* Wait until XBAR interface enabled */
		while (readl(RDC_RD_N_STATUS(partition_n)) &
		       RDC_RD_STAT_XBAR_DISABLE_MASK)
			;

		/* Lift reset for partition */
		writel(readl(RGM_PRST(partition_n)) & (~PRST_PERIPH_n_RST(0)),
		       RGM_PRST(partition_n));

		/* Follow steps to clear OSSE bit */
		writel(readl(MC_ME_PRTN_N_PCONF(partition_n)) &
		       ~MC_ME_PRTN_N_OSSE, MC_ME_PRTN_N_PCONF(partition_n));

		mc_me_wait_update(partition_n, MC_ME_PRTN_N_PUPD(partition_n),
				  MC_ME_PRTN_N_OSSUD);

		while (readl(MC_ME_PRTN_N_STAT(partition_n)) &
		       MC_ME_PRTN_N_OSSS)
			;

		while (readl(RGM_PSTAT(partition_n)) &
		       PSTAT_PERIPH_n_STAT(0))
			;

		/* Lock RDC register write */
		writel(readl(RDC_RD_N_CTRL(partition_n)) & ~RD_CTRL_UNLOCK_MASK,
		       RDC_RD_N_CTRL(partition_n));
	}

#ifndef CONFIG_TARGET_TYPE_S32GEN1_SIMULATOR
	writel(readl(MC_ME_PRTN_N_COFB0_CLKEN(partition_n)) | blocks_mask,
	       MC_ME_PRTN_N_COFB0_CLKEN(partition_n));

	mc_me_wait_update(partition_n, MC_ME_PRTN_N_PUPD(partition_n),
			  MC_ME_PRTN_N_PCUD);

	while (!(readl(MC_ME_PRTN_N_COFB0_STAT(partition_n)) &
		 blocks_mask))
		;
#endif
}

/*
 * Select the clock reference for required pll.
 * pll - ARM_PLL, PERIPH_PLL, ACCEL_PLL, DDR_PLL.
 * refclk_freq - input referece clock frequency (FXOSC - 40 MHZ, FIRC - 48 MHZ)
 */
static int select_pll_source_clk(enum pll_type pll, uint32_t refclk_freq)
{
	uint32_t clk_src;

	/* select the pll clock source */
	switch (refclk_freq) {
	case FIRC_CLK_FREQ:
		clk_src = PLLDIG_PLLCLKMUX_REFCLKSEL_SET_FIRC;
		break;
	case XOSC_CLK_FREQ:
		clk_src = PLLDIG_PLLCLKMUX_REFCLKSEL_SET_XOSC;
		break;
	default:
		/* The clock frequency unknown */
		return -1;
	}
	writel(clk_src, PLLDIG_PLLCLKMUX(pll));

	return 0;
}

/*
 * Program the pll according to the input parameters.
 * pll - ARM_PLL, PERIPH_PLL, ACCEL_PLL, DDR_PLL.
 * refclk_freq - input reference clock frequency (FXOSC - 40 MHZ, FIRC - 48 MHZ)
 * phi_nr - number of PHIn
 * freq - array of PHY frequencies
 * dfs_nr - number of DFS modules for current PLL
 * dfs - array with the activation dfs field, mfn and mfi
 * plldv_rdiv - divider of clkfreq_ref
 * plldv_mfi - loop multiplication factor divider
 * pllfd_mfn - numerator loop multiplication factor divider
 * Please consult the PLLDIG chapter of platform manual
 * before to use this function.
 *)
 */
int s32gen1_program_pll(enum pll_type pll, uint32_t refclk_freq, uint32_t phi_nr,
		       uint64_t freq[], uint32_t dfs_nr, uint32_t dfs[][DFS_PARAMS_Nr],
		       uint32_t plldv_rdiv, uint32_t plldv_mfi, uint32_t pllfd_mfn)
{
	uint32_t i, dfs_on = 0, fvco;

	/*
	 * This formula is from S32R45 platform reference manual
	 * (Rev. 1, draft C), PLL programming chapter.
	 */
	fvco = refclk_freq * (plldv_mfi + (pllfd_mfn/(float)18432)) /
		(float)plldv_rdiv;

	/*
	 * VCO should have value in [ PLL_MIN_FREQ, PLL_MAX_FREQ ].
	 * Please consult the platform DataSheet in order to determine
	 * the allowed values.
	 */

	if (fvco < PLL_MIN_FREQ || fvco > PLL_MAX_FREQ)
		return -1;

	/* Disable deviders. */
	for (i = 0; i < phi_nr; i++)
		writel(0x0, PLLDIG_PLLODIV(pll, i));

	/* Disable PLL. */
	writel(PLLDIG_PLLCR_PLLPD, PLLDIG_PLLCR(pll));

	if (select_pll_source_clk(pll, refclk_freq) < 0)
		return -1;

	writel(PLLDIG_PLLDV_RDIV_SET(plldv_rdiv) | PLLDIG_PLLDV_MFI(plldv_mfi),
	       PLLDIG_PLLDV(pll));
	writel(PLLDIG_PLLFD_MFN_SET(pllfd_mfn) |
	       PLLDIG_PLLFD_SMDEN, PLLDIG_PLLFD(pll));

	/* Calculate Output Frequency Divider for required PHIn outputs. */
	for (i = 0; i < phi_nr; i++) {
		if (freq[i])
			writel(readl(PLLDIG_PLLODIV(pll, i)) |
				     PLLDIG_PLLODIV_DIV_SET(fvco / freq[i] - 1),
				     PLLDIG_PLLODIV(pll, i));
	}

	/* Enable the PLL. */
	writel(0x0, PLLDIG_PLLCR(pll));

	/* Poll until PLL acquires lock. */
	while (!(readl(PLLDIG_PLLSR(pll)) & PLLDIG_PLLSR_LOCK))
		;

	/* Enable deviders. */
	for (i = 0; i < phi_nr; i++)
		writel(PLLDIG_PLLODIV_DE | readl(PLLDIG_PLLODIV(pll, i)),
		       PLLDIG_PLLODIV(pll, i));

	/* Only ARM_PLL and PERIPH_PLL. */
	if ((pll == ARM_PLL) || (pll == PERIPH_PLL)) {
		writel(DFS_PORTRESET_PORTRESET_MAXVAL, DFS_PORTRESET(pll));
		writel(DFS_CTL_RESET, DFS_CTL(pll));
		for (i = 0; i < dfs_nr; i++) {
			if (dfs[i][0]) {
				writel(DFS_DVPORTn_MFI_SET(dfs[i][2]) |
				       DFS_DVPORTn_MFN_SET(dfs[i][1]),
				       DFS_DVPORTn(pll, i));
				dfs_on |= (dfs[i][0] << i);
			}
		}
		/* DFS clk enable programming */
		writel(~DFS_CTL_RESET, DFS_CTL(pll));

		writel(DFS_PORTRESET_PORTRESET_SET(~dfs_on),
		       DFS_PORTRESET(pll));
		while ((readl(DFS_PORTSR(pll)) & dfs_on) != dfs_on)
			;
	}

	return 0;
}

int mux_source_clk_config(uintptr_t cgm_addr, uint8_t mux, uint8_t source)
{
	uint32_t css, csc;
	/* Ongoing clock switch? */
	while (readl(CGM_MUXn_CSS(cgm_addr, mux)) & MC_CGM_MUXn_CSS_SWIP)
			;

	csc = readl(CGM_MUXn_CSC(cgm_addr, mux));
	/* Clear previous source. */
	csc &= ~(MC_CGM_MUXn_CSC_SELCTL_MASK);
	/* select the clock source and trigger the clock switch. */
	writel(csc | MC_CGM_MUXn_CSC_SELCTL(source) | MC_CGM_MUXn_CSC_CLK_SW,
	       CGM_MUXn_CSC(cgm_addr, mux));
	/* Wait for configuration bit to auto-clear. */
	while (readl(CGM_MUXn_CSC(cgm_addr, mux)) & MC_CGM_MUXn_CSC_CLK_SW)
		;

	/* Is the clock switch completed? */
	while (readl(CGM_MUXn_CSS(cgm_addr, mux)) & MC_CGM_MUXn_CSS_SWIP)
			;

	/* Chech if the switch succeeded.
	 * Check switch trigger cause and the source.
	 */
	css = readl(CGM_MUXn_CSS(cgm_addr, mux));
	if ((MC_CGM_MUXn_CSS_SWTRG(css) == MC_CGM_MUXn_CSS_SWTRG_SUCCESS) &&
	    (MC_CGM_MUXn_CSS_SELSTAT(css) == source))
		return 0;

	return -1;
}

void mux_div_clk_config(uintptr_t cgm_addr, uint8_t mux, uint8_t dc, uint8_t divider)
{
	/* set the divider */
	writel(MC_CGM_MUXn_DCm_DE | MC_CGM_MUXn_DCm_DIV(divider),
	       CGM_MUXn_DCm(cgm_addr, mux, dc));

	/* Wait for divider gets updated */
	while (MC_CGM_MUXn_DIV_UPD_STAT_DIVSTAT(readl(CGM_MUXn_DIV_UPD_STAT(cgm_addr, mux))))
		;
}

void s32gen1_setup_fxosc(void)
{
	/* According to "20.4 Initialization information" from
	 * S32S247RM_Rev1D.pdf, "Once FXOSC is turned ON, DO NOT change the
	 * any signal (on the fly) which is going to analog module input.
	 * The inputs can be changed when the analog module is OFF...When
	 * disabling the IP through Software do not change any other values in
	 * the registers for at least 4 crystal clock cycles."
	 *
	 * Just make sure that FXOSC wasn't aleady started by BootROM.
	 */
	uint32_t ctrl;
	if (readl(FXOSC_CTRL) & FXOSC_CTRL_OSCON)
		return;

	ctrl = FXOSC_CTRL_COMP_EN;
	ctrl &= ~FXOSC_CTRL_OSC_BYP;
	ctrl |= FXOSC_CTRL_EOCV(0x1);
	ctrl |= FXOSC_CTRL_GM_SEL(0x7);
	writel(ctrl, FXOSC_CTRL);

	/* Switch ON the crystal oscillator. */
	writel(FXOSC_CTRL_OSCON | readl(FXOSC_CTRL), FXOSC_CTRL);

	/* Wait until the clock is stable. */
	while (!(readl(FXOSC_STAT) & FXOSC_STAT_OSC_STAT))
		;
}


