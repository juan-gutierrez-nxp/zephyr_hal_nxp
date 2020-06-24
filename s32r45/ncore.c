// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright 2020 NXP
 */

#include <sys/util.h>

//#define uint32_t int

#define readb(a)                  (*(volatile unsigned char *)(a))
#define readw(a)                  (*(volatile unsigned short *)(a))
#define readl(a)                  (*(volatile unsigned int *)(a))
#define readq(a)                  (*(volatile unsigned long long *)(a))

#define writeb(v,a)                (*(volatile unsigned char *)(a) = (v))
#define writew(v,a)                (*(volatile unsigned short *)(a) = (v))
#define writel(v,a)                (*(volatile unsigned int *)(a) = (v))
#define writeq(v,a)                (*(volatile unsigned long long *)(a) = (v))

#define NCORE_BASE_ADDR		(0x50400000ul)

#define CSR			(NCORE_BASE_ADDR + 0xff000)
#define CSADSE(m)		(CSR + 0x40 + (m * 0x4))
#define CSSFIDR(m)		(CSR + 0xf00 + (m * 0x4))
#define CSSFIDR_TYPE_MASK	(0xf)
#define CSSFIDR_TYPE_OFFSET	(26)
#define CSID			(CSR + 0xffc)
#define CSID_NUMSFS		(0x1f << 18)
#define CSUID			(CSR + 0xff8)
#define CSUID_NUMCAIUS_MASK	(0x7f)
#define CSUID_NUMCAIUS_OFFSET	(0)
#define CSUID_NUMNCBUS_MASK	(0x3f)
#define CSUID_NUMNCBUS_OFFSET	(8)
#define CSUID_NUMDIRUS_MASK	(0x3f)
#define CSUID_NUMDIRUS_OFFSET	(16)
#define CSUID_NUMCMIUS_MASK	(0x3f)
#define CSUID_NUMCMIUS_OFFSET	(24)

#define CAIU(n)			(NCORE_BASE_ADDR + (n * 0x1000))
#define CAIUCEC(n)		(CAIU(n) + 0x100)
#define CAIUUEC(n)		(CAIU(n) + 0x140)
#define CAIUID(n)		(CAIU(n) + 0xffc)
#define CAIUID_CA		BIT(15)
#define CAIUID_TYPE_MASK	(0xf)
#define CAIUID_TYPE_OFFSET	(16)

#define NCBU(n)			(NCORE_BASE_ADDR + 0x60000 + (n * 0x1000))
#define NCBUCEC(n)		(NCBU(n) + 0x100)
#define NCBUUEC(n)		(NCBU(n) + 0x140)
#define NCBUID(n)		(NCBU(n) + 0xffc)
#define NCBUID_CA		BIT(15)
#define NCBUID_TYPE_MASK	(0xf)
#define NCBUID_TYPE_OFFSET	(16)

#define DIRU(n)			(NCORE_BASE_ADDR + 0x80000 + (n * 0x1000))
#define DIRUSFE(n)		(DIRU(n) + 0x10)
#define DIRUCASER(n,m)		(DIRU(n) + 0x40 + (m * 0x4))
#define DIRUSFMC(n)		(DIRU(n) + 0x80ul)
#define DIRUSFMC_SFID_OFFSET	(16)
#define DIRUSFMA(n)		(DIRU(n) + 0x84)
#define DIRUSFMA_MNTOPACTIV	BIT(0)
#define DIRUCEC(n)		(DIRU(n) + 0x100)
#define DIRUUEC(n)		(DIRU(n) + 0x140)

#define CMIU(n)			(NCORE_BASE_ADDR + 0xc0000 + (n * 0x1000))
#define CMIUCEC(n)		(CMIU(n) + 0x100)
#define CMIUUEC(n)		(CMIU(n) + 0x140)

#define ERRDETEN		BIT(0)
#define ERRINTEN		BIT(1)

#define A53_GPR			(0x4007c400ul)
#define GPR06			(A53_GPR + 0x18)
#define GPR06_CA53_LOCKSTEP_EN	BIT(0)

#define CPUMASK_CLUSTER0	(BIT(0) | BIT(1))
#define CPUMASK_CLUSTER1	(BIT(2) | BIT(3))

void ncore_init(uint32_t cpumask)
{
	int i, j;
	uint32_t regdata;
	uint32_t csid_numsfs;
	uint32_t csuid_numcaius, csuid_numncbus, csuid_numdirus, csuid_numcmius;
	uint32_t dirucase[4] = {0,0,0,0};
	uint32_t csadse[4] = {0,0,0,0};
	uint32_t lockstep_enabled;

	csid_numsfs = (readl(CSID) & CSID_NUMSFS) + 1;
	regdata = readl(CSUID);
	csuid_numcaius = regdata >> CSUID_NUMCAIUS_OFFSET & CSUID_NUMCAIUS_MASK;
	csuid_numncbus = regdata >> CSUID_NUMNCBUS_OFFSET & CSUID_NUMNCBUS_MASK;
	csuid_numdirus = regdata >> CSUID_NUMDIRUS_OFFSET & CSUID_NUMDIRUS_MASK;
	csuid_numcmius = regdata >> CSUID_NUMCMIUS_OFFSET & CSUID_NUMCMIUS_MASK;

	lockstep_enabled = readl(GPR06) & GPR06_CA53_LOCKSTEP_EN;

	if (((cpumask & CPUMASK_CLUSTER0) == cpumask) ||
	    ((cpumask & CPUMASK_CLUSTER1) == cpumask) ||
	    (lockstep_enabled == GPR06_CA53_LOCKSTEP_EN)) {
		csuid_numcaius  = 1;
		csuid_numncbus  = 1;
	}

	for (i = 0; i < csuid_numdirus; i++) {
		writel(0, DIRUSFMC(i));
		while (readl(DIRUSFMA(i)) & DIRUSFMA_MNTOPACTIV)
			;
	}

	for (i = 0; i < csid_numsfs; i++) {
		regdata = readl(CSSFIDR(i)) & CSSFIDR_TYPE_MASK >> CSSFIDR_TYPE_OFFSET;
		if ((regdata >= 2) && (regdata <= 7))
			for (j = 0; j < csuid_numdirus; j++)
				writel(BIT(i) << DIRUSFMC_SFID_OFFSET, DIRUSFMC(j));
	}

	for (i = 0; i < csuid_numdirus; i++)
		while (readl(DIRUSFMA(i)) & DIRUSFMA_MNTOPACTIV)
			;

	for (i=0; i<csuid_numdirus; i++) {
		writel(0xffffffff, DIRUSFE(i));
		writel(ERRDETEN | ERRINTEN, DIRUCEC(i));
		writel(ERRDETEN | ERRINTEN, DIRUUEC(i));
	}

	for (i = 0; i < csuid_numcmius; i++) {
		writel(ERRDETEN | ERRINTEN, CMIUCEC(i));
		writel(ERRDETEN | ERRINTEN, CMIUUEC(i));
	}

	for (i = 0; i < csuid_numcaius; i++)
		if (readl(CAIUID(i)) & CAIUID_CA)
			dirucase[i / 32] |= BIT(i % 32);

	for (i = 0; i < csuid_numncbus; i++)
		if (readl(NCBUID(i)) & NCBUID_CA)
			dirucase[3] |= BIT(i % 32);

	for (i = 0; i < csuid_numdirus; i++)
		for (j = 0; j < 4; j++)
			writel(dirucase[j], DIRUCASER(i, j));

	for (i = 0; i < csuid_numcaius; i++) {
		writel(ERRDETEN | ERRINTEN, CAIUCEC(i));
		writel(ERRDETEN | ERRINTEN, CAIUUEC(i));
	}

	for (i = 0; i < csuid_numncbus; i++) {
		writel(ERRDETEN | ERRINTEN, NCBUCEC(i));
		writel(ERRDETEN | ERRINTEN, NCBUUEC(i));
	}

	for (i = 0; i < csuid_numcaius; i++) {
		regdata = readl(CAIUID(i)) & CAIUID_TYPE_MASK >> CAIUID_TYPE_OFFSET;
		if (regdata < 2)
			csadse[i / 32] += BIT(i % 32);
	}

	for (i=0; i<csuid_numncbus; i++) {
		regdata = readl(NCBUID(i)) & NCBUID_TYPE_MASK >> NCBUID_TYPE_OFFSET;
		if (regdata < 2)
			csadse[3] += BIT(i % 32);
	}

	for (i = 0; i < 4; i++)
		writel(csadse[i], CSADSE(i));
}
