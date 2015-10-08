/*
 * Purna Chandra Mandal,<purna.mandal@microchip.com>
 * Copyright (C) 2014 Microchip Technology Inc.  All rights reserved.
 *
 * This program is free software; you can distribute it and/or modify it
 * under the terms of the GNU General Public License (Version 2) as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/log2.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <asm/div64.h>
#include <linux/time.h>
#include <linux/debugfs.h>
#include <asm/traps.h>

#include <asm/mips-boards/generic.h>
#include <asm/mach-pic32/pic32.h>

/* OSCCON Reg fields */
#define OSC_CUR_MASK		0x07
#define OSC_CUR_SHIFT		12
#define OSC_NEW_MASK		0x07
#define OSC_NEW_SHIFT		8
#define OSC_SWEN		0x01
#define OSC_SOSCEN_MASK		0x02
#define OSC_CLK_FAILED		0x04
#define OSC_SPLL_LOCKED		0x20 /* SPLL is locked by SCLK MUX */
#define OSC_SYS_LOCK		0x80
#define OSC_SOSC_READY		BIT(22)

/* SPLLCON Reg fields */
#define PLL_RANGE_MASK		0x07
#define PLL_RANGE_SHIFT		0
#define PLL_ICLK_MASK		0x01
#define PLL_ICLK_SHIFT		7
#define PLL_IDIV_MASK		0x07
#define PLL_IDIV_SHIFT		8
#define PLL_ODIV_MASK		0x07
#define PLL_ODIV_SHIFT		24
#define PLL_MULT_MASK		0x7F
#define PLL_MULT_SHIFT		16
#define PLL_MULT_MAX		128
#define PLL_ODIV_MIN		1
#define PLL_ODIV_MAX		5

/* Peripheral Bus Clock Reg Fields */
#define PB_DIV_MASK		0x7f
#define PB_DIV_SHIFT		0
#define PB_DIV_READY		BIT(11)
#define PB_DIV_ENABLED		BIT(15)
#define PB_DIV_MAX		128
#define PB_DIV_MIN		0

/* Reference Oscillator Control Reg fields */
#define REFO_SEL_MASK		0x0f
#define REFO_SEL_SHIFT		0
#define REFO_ACTIVE		BIT(8)
#define REFO_DIVSW_EN		BIT(9)
#define REFO_OE			BIT(12)
#define REFO_ON			BIT(15)
#define REFO_DIV_SHIFT		16
#define REFO_DIV_MASK		0x7fff

/* Reference Oscillator Trim Register Fields */
#define REFO_TRIM_REG		0x10 /* Register offset w.r.t. REFO_CON_REG */
#define REFO_TRIM_MASK		0x1ff
#define REFO_TRIM_SHIFT		23
#define REFO_TRIM_MAX		511

/* FRC postscaler */
#define OSC_FRCDIV_MASK		0x07
#define OSC_FRCDIV_SHIFT	24

/* FRC tuning */
#define OSC_FRCTUN_MASK		0x3F
#define OSC_FRCTUN_SHIFT	0

/* SLEW Control Register fields */
#define SLEW_BUSY		0x01
#define SLEW_DOWNEN		0x02
#define SLEW_UPEN		0x04
#define SLEW_DIV		0x07
#define SLEW_DIV_SHIFT		8
#define SLEW_SYSDIV		0x0f
#define SLEW_SYSDIV_SHIFT	20

/* CFGMPLL Register fields */
#define MPLL_IDIV		0x3f	/* Input divider */
#define MPLL_MULT		0xff	/* Feedback multiplier */
#define MPLL_MULT_SHIFT		8
#define MPLL_VREG_DISABLE	BIT(22) /* Voltage Regulator Disable */
#define MPLL_VREG_READY		BIT(23) /* Voltage Regulator Ready */
#define MPLL_ODIV1		0x7	/* Output divider */
#define MPLL_ODIV1_SHIFT	24
#define MPLL_ODIV2		0x7	/* Output Divider (second stage) */
#define MPLL_ODIV2_SHIFT	27
#define MPLL_DISABLE		BIT(30)	/* PLL disable */
#define MPLL_READY		BIT(31)	/* PLL Ready */

/* Common clock flags */
#define CLK_ENABLED_ALWAYS	CLK_IGNORE_UNUSED
#define CLK_DIV_FIXED		BIT(20)

/* Sys Mux clock flags */
#define SYS_MUX_POSTDIV		0x1
#define SYS_MUX_SLEW		0x2

/* System PLL clk */
struct pic32_spll {
	struct clk_hw hw;
	void __iomem *regs;
	void __iomem *status_reg;
	u32 pll_locked;
	u8 idiv; /* pll-iclk divider, treating fixed */
};

/* System Clk */
struct pic32_sclk {
	struct clk_hw hw;
	void __iomem *regs;
	void __iomem *slwreg;
	unsigned long flags;
	u32 *parent_idx;
	struct debugfs_regset32	regset;
};

/* USB Phy Clk */
struct pic32_upll {
	struct clk_hw hw;
	unsigned long flags;
	void __iomem *regs;
};

/* DDR Phy & DDR clk */
struct pic32_mpll {
	struct clk_hw hw;
	void __iomem *regs;
};

/* Reference Oscillator */
struct pic32_refosc {
	struct clk_hw hw;
	void __iomem *regs;
	u32 *parent_idx;
	struct debugfs_regset32	regset;
};

/* Peripheral Bus Clock */
struct pic32_pbclk {
	struct clk_hw hw;
	void __iomem *regs;
	u32 flags;
	struct debugfs_regset32	regset;
};

/* External SOSC(fixed gated) clock  */
struct pic32_sosc {
	struct clk_hw hw;
	void __iomem *regs;
	void __iomem *status_reg;
	unsigned long fixed_rate;
	int bitmask;
	int status_bitmask;
};

/* Soc specific clock reg-base */
static void __iomem *pic32_clk_regbase;
static struct clk *pic32_sys_clk;

static DEFINE_SPINLOCK(lock);

#define __clk_lock(flags)	spin_lock_irqsave(&lock, flags)
#define __clk_unlock(flags)	spin_unlock_irqrestore(&lock, flags)
#define pic32_devcon_sysunlock()	pic32_syskey_unlock()
#define pic32_devcon_syslock()

#define cpu_nop5()			\
do {					\
	__asm__ __volatile__("nop");	\
	__asm__ __volatile__("nop");	\
	__asm__ __volatile__("nop");	\
	__asm__ __volatile__("nop");	\
	__asm__ __volatile__("nop");	\
} while (0)

#ifdef __CLK_DEBUG
#define __clk_debug(fmt, args...) \
	pr_info("%s:%d::" fmt, __func__, __LINE__, ##args)
#else
#define __clk_debug(fmt, args...)
#endif

#define clkhw_to_spll(_hw)	container_of(_hw, struct pic32_spll, hw)
#define clkhw_to_refosc(_hw)	container_of(_hw, struct pic32_refosc, hw)
#define clkhw_to_pbclk(_hw)	container_of(_hw, struct pic32_pbclk, hw)
#define clkhw_to_sys_clk(_hw)	container_of(_hw, struct pic32_sclk, hw)
#define clkhw_to_upll(_hw)	container_of(_hw, struct pic32_upll, hw)
#define clkhw_to_sosc(_hw)	container_of(_hw, struct pic32_sosc, hw)
#define clkhw_to_mpll(_hw)	container_of(_hw, struct pic32_mpll, hw)

/* pic32_of_clk_get_parent_indices - get parent clk indices.
 *
 * This is useful for mux clocks where all of the parents are not present
 * for a particular platform. "microchip,clock-indices" defines ids
 * corresponding to each input clock.
 */
int pic32_of_clk_get_parent_indices(struct device_node *np,
				    u32 **table_p,
				    int count)
{
	struct property *prop;
	const __be32 *pv;
	u32 i, *array, ret;

	if ((!table_p) || (!count))
		return -EINVAL;

	prop = of_find_property(np, "microchip,clock-indices", NULL);
	if (!prop) {
		ret = 0;
		goto out_err;
	}

	array = kzalloc((sizeof(u32) * count), GFP_KERNEL);
	if (!array) {
		ret = -ENOMEM;
		goto out_err;
	}

	for (i = 0, pv = NULL; i < count; i++) {
		pv = of_prop_next_u32(prop, pv, &array[i]);
		if (!pv) {
			kfree(array);
			ret = -EINVAL;
			goto out_err;
		}
	}

	*table_p = array;
	return 0;
out_err:
	*table_p = NULL;
	return ret;
}
EXPORT_SYMBOL(pic32_of_clk_get_parent_indices);

static int pic32_of_clk_register_clkdev(struct device_node *np, struct clk *clk)
{
	int ret;

	ret = clk_register_clkdev(clk, NULL, __clk_get_name(clk));
	if (ret) {
		pr_err("%s: clkdev register failed, ret %d\n",
		       __clk_get_name(clk), ret);
		goto out_err;
	}

	ret = of_clk_add_provider(np, of_clk_src_simple_get, clk);

out_err:
	return ret;
}

static int pic32_of_clk_add_aliases(struct device_node *np,
				    struct clk *clk)
{
	const char *alias_name;
	int index;
	int count = of_property_count_strings(np, "clock-output-names");

	for (index = 1; index < count; index++) {
		if (of_property_read_string_index(np, "clock-output-names",
						  index, &alias_name) < 0)
			break;
		clk_register_clkdev(clk, NULL, alias_name);
	}
	return 0;
}

static int pbclk_endisable(struct clk_hw *hw, int enable)
{
	struct pic32_pbclk *pb = clkhw_to_pbclk(hw);

	if (enable)
		clk_writel(PB_DIV_ENABLED, PIC32_SET(pb->regs));
	else
		clk_writel(PB_DIV_ENABLED, PIC32_CLR(pb->regs));
	return 0;
}

static int pbclk_is_enabled(struct clk_hw *hw)
{
	u32 v;
	struct pic32_pbclk *pb = clkhw_to_pbclk(hw);

	v = clk_readl(pb->regs) & PB_DIV_ENABLED;
	return !!v;
}

static int pbclk_enable(struct clk_hw *hw)
{
	return pbclk_endisable(hw, 1);
}

static void pbclk_disable(struct clk_hw *hw)
{
	struct pic32_pbclk *pb = clkhw_to_pbclk(hw);

	if (pb->flags & CLK_ENABLED_ALWAYS)
		return;

	pbclk_endisable(hw, 0);
	cpu_relax();
}

static unsigned long calc_best_divided_rate(unsigned long rate,
					    unsigned long parent_rate,
					    u32 divider_max,
					    u32 divider_min)
{
	u32 divided_rate_up, divided_rate_down, best_rate;
	u32 divider_down, divider_up;

	/* eq. clk_rate = parent_rate / divider.
	 *
	 * Find best divider to produce closest of target divided rate.
	 */

	divider_down = parent_rate / rate;
	divider_up = divider_down + 1;
	if (divider_down >= divider_max) {
		divider_down = divider_max;
		divider_up = divider_down;
	} else if (divider_down < divider_min) {
		divider_down = divider_min;
	}
	divided_rate_up = parent_rate / divider_down;
	divided_rate_down = parent_rate / divider_up;
	if (abs(rate - divided_rate_down) < abs(rate - divided_rate_up))
		best_rate = divided_rate_down;
	else
		best_rate = divided_rate_up;

	return best_rate;
}

static inline u16 pbclk_read_pbdiv(struct pic32_pbclk *pb)
{
	u32 v = clk_readl(pb->regs);

	return ((v >> PB_DIV_SHIFT) & PB_DIV_MASK) + 1;
}

static unsigned long pbclk_recalc_rate(struct clk_hw *hw,
				       unsigned long parent_rate)
{
	struct pic32_pbclk *pb = clkhw_to_pbclk(hw);
	unsigned long div, rate;

	div = pbclk_read_pbdiv(pb);
	rate = parent_rate / div;

	__clk_debug("%s: parent_rate = %lu, div %lu, rate = %lu\n",
		    __clk_get_name(hw->clk), parent_rate, div, rate);

	return rate;
}

static long pbclk_round_rate(struct clk_hw *hw, unsigned long rate,
			     unsigned long *parent_rate)
{
	long best_rate = calc_best_divided_rate(rate, *parent_rate,
						PB_DIV_MAX, PB_DIV_MIN);

	__clk_debug("target_rate = %lu, parent_rate %lu / best_rate = %ld\n",
		    rate, *parent_rate, best_rate);

	return best_rate;
}

static int pbclk_set_rate(struct clk_hw *hw, unsigned long rate,
			  unsigned long parent_rate)
{
	struct pic32_pbclk *pb = clkhw_to_pbclk(hw);
	u16 div, new_div;
	unsigned long pbclk, flags, v;

	__clk_debug("%s, rate %lu, parent_rate %lu\n",
		    __clk_get_name(hw->clk), rate, parent_rate);

	/* fixed-div clk ? */
	if (pb->flags & CLK_DIV_FIXED)
		return -EINVAL;

	/* calculate clkdiv and best rate */
	new_div = parent_rate / rate;
	pbclk = parent_rate / new_div;

	/* check & wait for PBDIVRDY */
	for (;;) {
		v = clk_readl(pb->regs);
		if (v & PB_DIV_READY)
			break;
	}

	__clk_lock(flags);

	/* apply new pbdiv */
	v = clk_readl(pb->regs);
	v &= ~PB_DIV_MASK;
	v |= (new_div - 1);

	/* sys unlock */
	pic32_devcon_sysunlock();

	clk_writel(v, pb->regs);

	/* sys lock */
	pic32_devcon_syslock();

	__clk_unlock(flags);

	/* wait again, for pbdivready */
	for (;;) {
		v = clk_readl(pb->regs);
		if (v & PB_DIV_READY)
			break;
	}

	/* confirm that new div is applied correctly */
	div = pbclk_read_pbdiv(pb);

	__clk_debug("pbclk = %lu, pbdiv %d / div = %d\n", pbclk, new_div, div);
	return (div == new_div) ? 0 : -EPERM;
}

static struct debugfs_reg32 pbclk_regs_debug[] = {
	{ .name = "PBxDIV", .offset = 0,},
};

static int pbclk_debug_init(struct clk_hw *hw, struct dentry *dentry)
{
	struct pic32_pbclk *pb = clkhw_to_pbclk(hw);
	struct dentry *file;

	pb->regset.base = pb->regs;
	pb->regset.regs = pbclk_regs_debug;
	pb->regset.nregs = ARRAY_SIZE(pbclk_regs_debug);

	file = debugfs_create_regset32("regdump", S_IRUGO, dentry, &pb->regset);
	if (IS_ERR(file))
		return PTR_ERR(file);

	return 0;
}

/* Reference Oscillator operations */
static int roclk_endisable(struct clk_hw *hw, int enable)
{
	struct pic32_refosc *refo = clkhw_to_refosc(hw);

	if (enable)
		clk_writel(REFO_ON | REFO_OE, PIC32_SET(refo->regs));
	else
		clk_writel(REFO_ON | REFO_OE, PIC32_CLR(refo->regs));
	return 0;
}

static int roclk_is_enabled(struct clk_hw *hw)
{
	struct pic32_refosc *refo = clkhw_to_refosc(hw);

	return clk_readl(refo->regs) & REFO_ON;
}

static int roclk_enable(struct clk_hw *hw)
{
	return roclk_endisable(hw, 1);
}

static void roclk_disable(struct clk_hw *hw)
{
	roclk_endisable(hw, 0);
	cpu_relax();
}

static void roclk_init(struct clk_hw *hw)
{
	roclk_disable(hw);
}

static u8 roclk_get_parent(struct clk_hw *hw)
{
	u8 i = 0;
	struct pic32_refosc *refo = clkhw_to_refosc(hw);
	unsigned long v;

	v = clk_readl(refo->regs);
	v = (v >> REFO_SEL_SHIFT) & REFO_SEL_MASK;

	if (!refo->parent_idx)
		goto done;

	for (i = 0; i < __clk_get_num_parents(hw->clk); i++)
		if (refo->parent_idx[i] == v)
			return (u8)i;
done:
	return (u8)v;
}

static int roclk_set_parent(struct clk_hw *hw, u8 index)
{
	struct pic32_refosc *refo = clkhw_to_refosc(hw);
	unsigned long v, flags;
	u8 new_idx, cur_idx, was_disabled = 1;

	new_idx = index;
	if (refo->parent_idx && (index < __clk_get_num_parents(hw->clk)))
		new_idx = refo->parent_idx[index];

	/* sanity */
	v = clk_readl(refo->regs);
	cur_idx = v & REFO_SEL_MASK;

	if (unlikely(cur_idx == new_idx))
		return 0;

	/*
	 * Note: clk-src switching is allowed only when module is not ACTIVE.
	 * Module gets ACTIVE when enabled. So it meant set_parent() needs
	 * clk-gating across the call.
	 */
	if (roclk_is_enabled(hw)) {
		pr_warn("%s needs gated clock. Forcing.\n", __func__);
		roclk_disable(hw);
		was_disabled = 0;
	}

	/* wait until ACTIVE bit is zero */
	for (;;) {
		v = clk_readl(refo->regs);
		if ((v & REFO_ACTIVE) == 0)
			break;
	}

	__clk_lock(flags);

	/* sysunlock */
	pic32_devcon_sysunlock();

	/* Calculate REFOCON register value */
	v = clk_readl(refo->regs);
	v &= ~(REFO_SEL_MASK << REFO_SEL_SHIFT);
	v |= (new_idx << REFO_SEL_SHIFT);

	/* Apply */
	clk_writel(v, refo->regs);

	/* syslock */
	pic32_devcon_syslock();

	/* enable module */
	clk_writel(REFO_ON | REFO_OE, PIC32_SET(refo->regs));

	__clk_unlock(flags);

	/* keep it disabled, if it was */
	if (was_disabled)
		clk_writel(REFO_ON, PIC32_CLR(refo->regs));

	return 0;
}

static unsigned long roclk_calc_rate(unsigned long parent_rate,
				     u16 rodiv, u16 rotrim)
{
	u64 rate64;
	u32 N;

	N = rodiv;
	/* fout = fin / [2 * {N + (M / 512)}]
	 *	= fin * 512 / [1024 * N + 2 * M]
	 *	= fin * 256 / (512 * N + M)
	 *	= (fin << 8) / ((N << 9) + M)
	 */
	if (rotrim) {
		N = (N << 9) + rotrim;
		rate64 = parent_rate;
		rate64 <<= 8;
		do_div(rate64, N);
	} else {
		rate64 = parent_rate / (N << 1);
	}
	return (unsigned long)rate64;
}

static void roclk_calc_div_trim(unsigned long rate,
				unsigned long parent_rate,
				u16 *rodiv_p, u16 *rotrim_p)
{
	u16 div, rotrim, rodiv;
	u64 frac;

	/* Find integer approximation of floating-point arithmatic.
	 *      fout = fin / [2 * {rodiv + (rotrim / 512)}] ... (1)
	 * i.e. fout = fin / 2 * DIV
	 *      whereas DIV = rodiv + (rotrim / 512)
	 *
	 * Since kernel does not perform floating-point arithmatic so
	 * (rotrim/512) will be zero. And DIV & rodiv will result same.
	 *
	 * ie. fout = (fin * 256) / [(512 * rodiv) + rotrim]  ... from (1)
	 * ie. rotrim = ((fin * 256) / fout) - (512 * DIV)
	 */
	if (parent_rate <= rate) {
		div = 0;
		frac = 0;
		rodiv = 0;
		rotrim = 0;
	} else {
		div = parent_rate / (rate << 1);
		frac = parent_rate;
		frac <<= 8;
		do_div(frac, rate);
		frac -= (u64)(div << 9);

		rodiv = (div > REFO_DIV_MASK) ? REFO_DIV_MASK : div;
		rotrim = (frac >= REFO_TRIM_MAX) ? REFO_TRIM_MAX : (u16)frac;
	}

	if (rodiv_p)
		*rodiv_p = rodiv;

	if (rotrim_p)
		*rotrim_p = rotrim;

	__clk_debug("parent_rate %lu, rate %lu/div %d, frac %lld (rotrim %u)\n",
		    parent_rate, rate, div, frac, rotrim);
}

static unsigned long roclk_recalc_rate(struct clk_hw *hw,
				       unsigned long parent_rate)
{
	struct pic32_refosc *refo = clkhw_to_refosc(hw);
	unsigned long v;
	u16 rodiv, rotrim;

	/* get rodiv */
	v = clk_readl(refo->regs);
	rodiv = (v >> REFO_DIV_SHIFT) & REFO_DIV_MASK;

	/* get trim */
	v = clk_readl(refo->regs + REFO_TRIM_REG);
	rotrim = (v >> REFO_TRIM_SHIFT) & REFO_TRIM_MASK;

	v = roclk_calc_rate(parent_rate, rodiv, rotrim);

	__clk_debug("%s, parent_rate %lu / parent %s / rate %lu\n",
		    __clk_get_name(hw->clk), parent_rate,
		    __clk_get_name(__clk_get_parent(hw->clk)), v);
	return v;
}

static long roclk_round_rate(struct clk_hw *hw, unsigned long rate,
			     unsigned long *parent_rate)
{
	u16 rotrim, rodiv;

	/* calculate dividers for new rate */
	roclk_calc_div_trim(rate, *parent_rate, &rodiv, &rotrim);

	/* caclulate new rate (rounding) based on new rodiv & rotrim */
	return roclk_calc_rate(*parent_rate, rodiv, rotrim);
}

static long roclk_determine_rate(struct clk_hw *hw, unsigned long rate,
				 unsigned long min_rate,
				 unsigned long max_rate,
				 unsigned long *best_parent_rate_p,
				 struct clk_hw **best_parent_p)
{
	struct clk *clk = hw->clk, *parent_clk, *best_parent_clk = NULL;
	unsigned int i, delta, best_delta = -1;
	unsigned long parent_rate, best_parent_rate = 0;
	unsigned long best = 0, nearest_rate;

	/* find a parent which can generate nearest clkrate >= rate */
	for (i = 0; i < __clk_get_num_parents(clk); i++) {
		/* get parent */
		parent_clk = clk_get_parent_by_index(clk, i);
		if (!parent_clk)
			continue;

		/* skip if parent runs slower than target rate */
		parent_rate = __clk_get_rate(parent_clk);
		if (rate > parent_rate)
			continue;

		nearest_rate = roclk_round_rate(hw, rate, &parent_rate);
		delta = abs(nearest_rate - rate);
		if ((nearest_rate >= rate) && (delta < best_delta)) {
			best_parent_clk = parent_clk;
			best_parent_rate = parent_rate;
			best = nearest_rate;
			best_delta = delta;

			if (delta == 0)
				break;
		}
	}

	/* if no match found, retain old rate */
	if (!best_parent_clk) {
		pr_err("%s:%s, no parent found for rate %lu.\n",
		       __func__, __clk_get_name(clk), rate);
		best_parent_clk = clk_get_parent(clk);
		best_parent_rate = clk_get_rate(best_parent_clk);
		best = clk_get_rate(clk);
	}

	__clk_debug("%s,rate %lu /best_parent(%s, %lu) /best %lu /delta %d\n",
		    __clk_get_name(clk), rate, __clk_get_name(best_parent_clk),
		    best_parent_rate, best, best_delta);

	if (best_parent_rate_p)
		*best_parent_rate_p = best_parent_rate;

	if (best_parent_p)
		(*best_parent_p)->clk = best_parent_clk;

	return best;
}

static int roclk_set_rate_and_parent(struct clk_hw *hw,
				     unsigned long rate,
				     unsigned long parent_rate,
				     u8 index)
{
	struct pic32_refosc *refo = clkhw_to_refosc(hw);
	u16 trim, rodiv, parent_id, was_disabled = 1;
	unsigned long flags, v;

	if (unlikely(__clk_get_rate(hw->clk) == rate))
		return 0;

	/* calculate new rodiv & rotrim for new rate */
	roclk_calc_div_trim(rate, parent_rate, &rodiv, &trim);

	__clk_debug("parent_rate = %lu, rate = %lu, div = %d, trim = %d\n",
		    parent_rate, rate, rodiv, trim);

	/* Note: rosel can only be programmed when module is INACTIVE.
	 * i.e gating is required across set_parent.
	 * So disable clk, if required.
	 */
	if (roclk_is_enabled(hw)) {
		pr_err("%s: needs gating. Forcing.\n", __func__);
		roclk_disable(hw);
		was_disabled = 0;
	}

	/* check current source */
	if (refo->parent_idx)
		index = refo->parent_idx[index];

	parent_id = roclk_get_parent(hw);
	if (parent_id == index)
		goto clk_rosel_ready;

	/* wait till source change is active */
	for (;;) {
		v = clk_readl(refo->regs);
		if ((v & (REFO_DIVSW_EN | REFO_ACTIVE)) == 0)
			break;
	}

clk_rosel_ready:
	/* spinlock */
	__clk_lock(flags);
	v = clk_readl(refo->regs);

	/* sysunlock */
	pic32_devcon_sysunlock();

	/* apply parent, if required */
	if (parent_id != index) {
		v &= ~(REFO_SEL_MASK << REFO_SEL_SHIFT);
		v |= (index << REFO_SEL_SHIFT);
	}

	/* apply RODIV */
	v &= ~(REFO_DIV_MASK << REFO_DIV_SHIFT);
	v |= (rodiv << REFO_DIV_SHIFT);
	clk_writel(v, refo->regs);

	/* apply ROTRIM */
	v = clk_readl(refo->regs + REFO_TRIM_REG);
	v &= ~(REFO_TRIM_MASK << REFO_TRIM_SHIFT);
	v |= (trim << REFO_TRIM_SHIFT);
	clk_writel(v, refo->regs + REFO_TRIM_REG);

	/* enable refo module */
	clk_writel(REFO_ON | REFO_OE, PIC32_SET(refo->regs));

	/* activate divider switching */
	clk_writel(REFO_DIVSW_EN, PIC32_SET(refo->regs));

	/* syslock */
	pic32_devcon_syslock();

	/* wait till divswen is in-progress */
	for (;;) {
		v = clk_readl(refo->regs);
		if ((v & REFO_DIVSW_EN) == 0)
			break;
	}

	__clk_unlock(flags);

	/* keep it disabled if it was */
	if (was_disabled)
		clk_writel(REFO_ON, PIC32_CLR(refo->regs));

	return 0;
}

static int roclk_set_rate(struct clk_hw *hw, unsigned long rate,
			  unsigned long parent_rate)
{
	u8 index = roclk_get_parent(hw);

	return roclk_set_rate_and_parent(hw, rate, parent_rate, index);
}

static struct debugfs_reg32 roclk_regs_debug[] = {
	{ .name = "REFOxCON", .offset = 0,},
	{ .name = "REFOxTRIM", .offset = REFO_TRIM_REG,},
};

static int roclk_debug_init(struct clk_hw *hw, struct dentry *dentry)
{
	struct pic32_refosc *ro = clkhw_to_refosc(hw);
	struct dentry *file;

	ro->regset.base = ro->regs;
	ro->regset.regs = roclk_regs_debug;
	ro->regset.nregs = ARRAY_SIZE(roclk_regs_debug);

	file = debugfs_create_regset32("regdump", S_IRUGO, dentry, &ro->regset);

	return IS_ERR(file) ? PTR_ERR(file) : 0;
}

static inline u8 spll_odiv_to_divider(u8 odiv)
{
	if (odiv <= PLL_ODIV_MIN)
		odiv = PLL_ODIV_MIN;
	else if (odiv >= PLL_ODIV_MAX)
		odiv = PLL_ODIV_MAX;

	return 1 << odiv;
}

static unsigned long spll_calc_mult_div(struct pic32_spll *pll,
					unsigned long rate,
					unsigned long parent_rate,
					u8 *mult_p, u8 *odiv_p)
{
	u8 mul, div, best_mul = 1, best_div = 1;
	unsigned long new_rate, best_rate = rate;
	unsigned int best_delta = -1, delta, match_found = 0;

	parent_rate /= pll->idiv;

	for (mul = 1; mul <= PLL_MULT_MAX; mul++) {
		for (div = PLL_ODIV_MIN; div <= PLL_ODIV_MAX; div++) {
			new_rate = (parent_rate * mul) / (1 << div);
			delta = abs(rate - new_rate);
			if ((new_rate >= rate) && (delta < best_delta)) {
				best_delta = delta;
				best_rate = new_rate;
				best_mul = mul;
				best_div = div;
				match_found = 1;
			}
		}
	}

	if (!match_found) {
		pr_warn("spll: no match found\n");
		return 0;
	}

	__clk_debug("rate %lu, par_rate %lu/mult %u, div %u, best_rate %lu\n",
		    rate, parent_rate, best_mul, best_div, best_rate);

	if (mult_p)
		*mult_p = best_mul - 1;

	if (odiv_p)
		*odiv_p = best_div;

	return best_rate;
}

static unsigned long spll_clk_recalc_rate(struct clk_hw *hw,
					  unsigned long parent_rate)
{
	struct pic32_spll *pll = clkhw_to_spll(hw);
	unsigned long rate, pll_in_rate, v;
	u8 mult, odiv, div;

	v = clk_readl(pll->regs);
	odiv = ((v >> PLL_ODIV_SHIFT) & PLL_ODIV_MASK);
	mult = ((v >> PLL_MULT_SHIFT) & PLL_MULT_MASK) + 1;
	div = spll_odiv_to_divider(odiv);

	/* pll_in = parent_rate / idiv
	 * pll_out = pll_in * mult / div;
	 */
	pll_in_rate = parent_rate / pll->idiv;
	rate = pll_in_rate * mult;
	rate /= div;

	__clk_debug("parent_rate = %lu, mult %d, div %d, rate = %lu\n",
		    parent_rate, mult, div, rate);

	return (unsigned long)rate;
}

static long spll_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *parent_rate)
{
	struct pic32_spll *pll = clkhw_to_spll(hw);

	return spll_calc_mult_div(pll, rate, *parent_rate, NULL, NULL);
}

static int spll_clk_set_rate(struct clk_hw *hw, unsigned long rate,
			     unsigned long parent_rate)
{
	struct pic32_spll *pll = clkhw_to_spll(hw);
	u8 mult, odiv;
	unsigned long ret, v, loop = 1000;
	struct clk *sclk_parent;
	unsigned long flags;

	ret = spll_calc_mult_div(pll, rate, parent_rate, &mult, &odiv);
	if (!ret || (ret == rate))
		return 0;

	/* To change frequency
	 * - (a) switch sysclk-parent to FRCDIV.
	 * - (b) apply new mult & odiv.
	 * - (c) switch back to PLL
	 * - (d) wait until PLL settles down / locked.
	 */

	/* To check whether rate change is allowed we will have to ensure
	 * spll_clk is not active parent of sys_clk.
	 */
	BUG_ON(IS_ERR_OR_NULL(pic32_sys_clk));

	/* get sysclk parent */
	sclk_parent = clk_get_parent(pic32_sys_clk);

	/* does sys_clk using spll_clk as parent ? */
	if (unlikely(sclk_parent == hw->clk)) {
		pr_err("spll: set_rate() is not allowed when spll is parent of sys_clk.");
		pr_err("First reparent sys_clk to frcdiv-clk and then try.\n");
		return -EPERM;
	}

	/* lock */
	__clk_lock(flags);

	/* apply new multiplier & divisor (read-modify-write) */
	v = clk_readl(pll->regs);
	v &= ~(PLL_MULT_MASK << PLL_MULT_SHIFT);
	v &= ~(PLL_ODIV_MASK << PLL_ODIV_SHIFT);
	v |= (mult << PLL_MULT_SHIFT) | (odiv << PLL_ODIV_SHIFT);

	/* sysunlock before writing to SPLLCON register */
	pic32_devcon_sysunlock();

	clk_writel(v, pll->regs);
	cpu_relax();

	/* insert few nops (5-stage) to ensure CPU does not hang */
	cpu_nop5();
	cpu_nop5();

	/* syslock*/
	pic32_devcon_syslock();

	/* Wait until PLL is locked (maximum 100 usecs). */
	for (;;) {
		v = clk_readl(pll->status_reg);
		if (v & pll->pll_locked)
			break;

		if (--loop == 0)
			break;

		ndelay(100);
	}

	/* lock */
	__clk_unlock(flags);

	v = clk_readl(pll->status_reg);
	if (v & pll->pll_locked) {
		__clk_debug("clk & pll selection is locked.\n");
		return 0;
	}

	return 0;
}

static struct debugfs_reg32 sclk_regs_debug[] = {
	{ .name = "OSCCON", .offset = 0,},
	{ .name = "OSCTUN", .offset = 0x10,},
	{ .name = "SPLLCON", .offset = 0x20,},
};

static int sclk_debug_init(struct clk_hw *hw, struct dentry *dir)
{
	struct pic32_sclk *sclk = clkhw_to_sys_clk(hw);
	struct dentry *file;

	sclk->regset.base = sclk->regs;
	sclk->regset.regs = sclk_regs_debug;
	sclk->regset.nregs = ARRAY_SIZE(sclk_regs_debug);

	file = debugfs_create_regset32("regdump", S_IRUGO, dir, &sclk->regset);
	if (IS_ERR(file))
		return PTR_ERR(file);

	return 0;
}

static long sclk_round_rate(struct clk_hw *hw, unsigned long rate,
			    unsigned long *parent_rate)
{
	return calc_best_divided_rate(rate, *parent_rate, SLEW_SYSDIV, 1);
}

static unsigned long sclk_get_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	u32 v, div;
	struct pic32_sclk *sysclk = clkhw_to_sys_clk(hw);

	v = clk_readl(sysclk->slwreg);
	div = (v >> SLEW_SYSDIV_SHIFT) & SLEW_SYSDIV;
	div += 1; /* sys-div to divider */

	return parent_rate / div;
}

static int sclk_set_rate(struct clk_hw *hw,
			 unsigned long rate, unsigned long parent_rate)
{
	u32 v, div;
	unsigned long flags;
	struct pic32_sclk *sysclk = clkhw_to_sys_clk(hw);

	div = parent_rate / rate;

	__clk_lock(flags);

	/* sysunlock*/
	pic32_devcon_sysunlock();

	/* apply new div */
	v = clk_readl(sysclk->slwreg);
	v &= ~(SLEW_SYSDIV << SLEW_SYSDIV_SHIFT);
	v |= ((div - 1) << SLEW_SYSDIV_SHIFT);
	clk_writel(v, sysclk->slwreg);

	/* syslock*/
	pic32_devcon_syslock();

	/* wait until BUSY is cleared */
	for (;;) {
		v = clk_readl(sysclk->slwreg);
		if (!(v & SLEW_BUSY))
			break;
	}
	__clk_unlock(flags);

	return 0;
}

static u8 sclk_get_parent(struct clk_hw *hw)
{
	u8 idx, i;
	u32 v;
	struct pic32_sclk *sysclk = clkhw_to_sys_clk(hw);

	v = clk_readl(sysclk->regs);
	idx = (v >> OSC_CUR_SHIFT) & OSC_CUR_MASK;

	__clk_debug("parent = %d\n",  idx);
	if (!sysclk->parent_idx)
		goto done;

	for (i = 0; i < __clk_get_num_parents(hw->clk); i++) {
		if (sysclk->parent_idx[i] == idx) {
			idx = i;
			break;
		}
	}

done:
	return idx;
}

static int sclk_set_parent(struct clk_hw *hw, u8 index)
{
	u32 v;
	unsigned long flags, parent_rate;
	u8 nosc, cosc;
	struct pic32_sclk *sysclk = clkhw_to_sys_clk(hw);

	/* find new_osc */
	nosc = sysclk->parent_idx ? sysclk->parent_idx[index] : index;

	/* check cur_osc is not same as new_osc */
	v = clk_readl(sysclk->regs);
	cosc = (v >> OSC_CUR_SHIFT) & OSC_CUR_MASK;
	if (unlikely(cosc == nosc))
		return 0;

	parent_rate = clk_get_rate(clk_get_parent_by_index(hw->clk, index));
	__clk_debug("init clk-switch / idx: %d -> %d ....\n\n\n", cosc, nosc);

	/* spin lock */
	__clk_lock(flags);

	/* sysunlock*/
	pic32_devcon_sysunlock();

	/* set new parent */
	v = clk_readl(sysclk->regs);
	v &= ~(OSC_NEW_MASK << OSC_NEW_SHIFT);
	v |= (nosc << OSC_NEW_SHIFT);
	clk_writel(v, sysclk->regs);

	/* initate switch */
	clk_writel(OSC_SWEN, PIC32_SET(sysclk->regs));
	cpu_relax();

	/* some nop to flush pipeline(cpu-clk is in-flux) */
	cpu_nop5();

	/* syslock */
	pic32_devcon_syslock();

	/* wait for SWEN bit to clear */
	for (;;) {
		v = clk_readl(sysclk->regs);
		if (!(v & OSC_SWEN))
			break;
	}

	/* spin unlock */
	__clk_unlock(flags);

	/* SYSCLK switch logic performs sanity and maintains state machine for
	 * clock-switching. So h/w might reject clk-switch request if required
	 * conditions (like clksrc not present or unstable) aren't met.
	 * So confirm before claiming success.
	 */
	cosc = (v >> OSC_CUR_SHIFT) & OSC_CUR_MASK;
	if (unlikely(cosc != nosc)) {
		pr_err("%s: err COSC %d and NOSC %d\n",
		       __clk_get_name(hw->clk), cosc, nosc);
		return -EBUSY;
	}

	return 0;
}

static int sosc_clk_enable(struct clk_hw *hw)
{
	int loop;
	unsigned long flags;
	struct pic32_sosc *sosc = clkhw_to_sosc(hw);

	local_irq_save(flags);

	/* enable SOSC */
	pic32_devcon_sysunlock();
	clk_writel(sosc->bitmask, PIC32_SET(sosc->regs));
	pic32_devcon_syslock();

	/* Wait till warm-up period expires and ready-status is updated */
	for (loop = 1024; loop; --loop) {
		cpu_relax();
		if (clk_readl(sosc->status_reg) & sosc->status_bitmask)
			break;
	}

	local_irq_restore(flags);

	if (!loop) {
		pr_err("%s: possibly clk is not present or ready for ops\n",
		       __clk_get_name(hw->clk));
		return -EBUSY;
	}

	return 0;
}

static void sosc_clk_disable(struct clk_hw *hw)
{
	struct pic32_sosc *sosc = clkhw_to_sosc(hw);
	unsigned long flags;

	local_irq_save(flags);

	pic32_devcon_sysunlock();
	clk_writel(sosc->bitmask, PIC32_CLR(sosc->regs));
	pic32_devcon_syslock();

	local_irq_restore(flags);
}

static int sosc_clk_is_enabled(struct clk_hw *hw)
{
	struct pic32_sosc *sosc = clkhw_to_sosc(hw);
	u32 enable, status;

	/* check enable & ready-status */
	enable = clk_readl(sosc->regs) & sosc->bitmask;
	status = clk_readl(sosc->status_reg) & sosc->status_bitmask;

	return enable && status;
}

static unsigned long sosc_clk_calc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	return clkhw_to_sosc(hw)->fixed_rate;
}

static int mpll_clk_is_enable(struct clk_hw *hw)
{
	struct pic32_mpll *mpll = clkhw_to_mpll(hw);
	u32 v = clk_readl(mpll->regs);

	return (!(v & MPLL_DISABLE)) && (v & MPLL_READY);
}

static int mpll_clk_enable(struct clk_hw *hw)
{
	return 0;
}

static void mpll_clk_disable(struct clk_hw *hw)
{
}

static unsigned long mpll_clk_recalc_rate(struct clk_hw *hw,
					  unsigned long parent_rate)
{
	u64 rate;
	u32 v, idiv, odiv1, odiv2, mul;
	struct pic32_mpll *mpll = clkhw_to_mpll(hw);

	v = clk_readl(mpll->regs);
	idiv = v & MPLL_IDIV;
	mul = (v >> MPLL_MULT_SHIFT) & MPLL_MULT;
	odiv1 = (v >> MPLL_ODIV1_SHIFT) & MPLL_ODIV1;
	odiv2 = (v >> MPLL_ODIV2_SHIFT) & MPLL_ODIV2;

	rate = (parent_rate / idiv) * mul;
	do_div(rate, odiv1);
	do_div(rate, odiv2);

	return (unsigned long)rate;
}

static struct clk_ops pbclk_ops = {
	.enable		= pbclk_enable,
	.disable	= pbclk_disable,
	.is_enabled	= pbclk_is_enabled,
	.recalc_rate	= pbclk_recalc_rate,
	.round_rate	= pbclk_round_rate,
	.set_rate	= pbclk_set_rate,
	.debug_init	= pbclk_debug_init,
};

/* sysclk is a mux with post-divider.
 * get/set_parent &  get/set_rate are required operation.
 */
static struct clk_ops sclk_postdiv_ops = {
	.get_parent	= sclk_get_parent,
	.set_parent	= sclk_set_parent,
	.determine_rate = __clk_mux_determine_rate,
	.round_rate	= sclk_round_rate,
	.set_rate	= sclk_set_rate,
	.recalc_rate	= sclk_get_rate,
	.debug_init	= sclk_debug_init,
};

static struct clk_ops spll_clk_ops = {
	.recalc_rate	= spll_clk_recalc_rate,
	.round_rate	= spll_clk_round_rate,
	.set_rate	= spll_clk_set_rate,
};

static struct clk_ops mpll_clk_ops = {
	.is_enabled	= mpll_clk_is_enable,
	.enable		= mpll_clk_enable,
	.disable	= mpll_clk_disable,
	.recalc_rate	= mpll_clk_recalc_rate,
};

static struct clk_ops roclk_ops = {
	.enable			= roclk_enable,
	.disable		= roclk_disable,
	.is_enabled		= roclk_is_enabled,
	.get_parent		= roclk_get_parent,
	.set_parent		= roclk_set_parent,
	.determine_rate		= roclk_determine_rate,
	.recalc_rate		= roclk_recalc_rate,
	.round_rate		= roclk_round_rate,
	.set_rate_and_parent	= roclk_set_rate_and_parent,
	.set_rate		= roclk_set_rate,
	.init			= roclk_init,
	.debug_init		= roclk_debug_init,
};

static struct clk_ops sosc_ops = {
	.enable = sosc_clk_enable,
	.disable = sosc_clk_disable,
	.is_enabled = sosc_clk_is_enabled,
	.recalc_rate = sosc_clk_calc_rate,
};

#define init_clk_data(__initdata, __clk, __parents,	\
	__nr_parents, __flags, __ops)			\
	__initdata.name = (__clk);			\
	__initdata.ops = (__ops);			\
	__initdata.flags = (__flags);			\
	__initdata.parent_names = (__parents);		\
	__initdata.num_parents = (__nr_parents)

static struct clk *periph_clk_register(const char *name,
				       const char **parent_name,
				       void __iomem *regs, u32 flags)
{
	struct clk *clk;
	struct pic32_pbclk *pbclk;
	struct clk_init_data init;

	init_clk_data(init, name, parent_name, 1,
		      flags | CLK_IS_BASIC, &pbclk_ops);

	pbclk = kzalloc(sizeof(*pbclk), GFP_KERNEL);
	if (!pbclk)
		return ERR_PTR(-ENOMEM);

	/* init */
	pbclk->regs = regs;
	pbclk->flags = flags;
	pbclk->hw.init = &init;

	clk = clk_register(NULL, &pbclk->hw);
	if (IS_ERR(clk))
		kfree(pbclk);

	return clk;
}

static struct clk *sys_mux_clk_register(const char *name,
					const char **parents,
					const int num_parents,
					void __iomem *regs,
					void __iomem *slew_reg,
					u32 *parent_idx,
					const struct clk_ops *clkop)
{
	struct clk *clk;
	struct pic32_sclk *sysclk;
	struct clk_init_data init;

	init_clk_data(init, name, parents, num_parents,
		      CLK_IS_BASIC, clkop);

	sysclk = kzalloc(sizeof(*sysclk), GFP_KERNEL);
	if (!sysclk)
		return ERR_PTR(-ENOMEM);

	/* init sysclk data */
	sysclk->hw.init = &init;
	sysclk->regs = regs;
	sysclk->slwreg = slew_reg;
	sysclk->parent_idx = parent_idx;

	clk = clk_register(NULL, &sysclk->hw);
	if (IS_ERR(clk)) {
		kfree(sysclk);
		return clk;
	}

	/* Maintain reference to this clock;
	 * This clock will be needed in spll-rate-change.
	 */
	pic32_sys_clk = clk;

	return clk;
}

static struct clk *spll_clk_register(const char *name, const char *parents,
				     void __iomem *regs,
				     void __iomem *status_reg,
				     u32 lock_bitmask)
{
	u32 v;
	struct pic32_spll *pll;
	struct clk_init_data init;
	struct clk *clk;

	init_clk_data(init, name, &parents, 1, CLK_IS_BASIC, &spll_clk_ops);

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	/* initialize configuration */
	pll->regs = regs;
	pll->status_reg = status_reg;
	pll->pll_locked = lock_bitmask;
	pll->hw.init = &init;

	/* read and cache pll_idiv; we will use it as constant.*/
	v = clk_readl(pll->regs);
	pll->idiv = ((v >> PLL_IDIV_SHIFT) & PLL_IDIV_MASK) + 1;

	clk = clk_register(NULL, &pll->hw);
	if (IS_ERR(clk))
		kfree(pll);

	return clk;
}

static struct clk *mpll_clk_register(const char *name,
				     const char *parent, void __iomem *regs)
{
	struct clk_init_data init;
	struct clk *clk;
	struct pic32_mpll *mpll;

	init_clk_data(init, name, &parent, 1,
		      CLK_IGNORE_UNUSED | CLK_IS_BASIC, &mpll_clk_ops);

	mpll = kzalloc(sizeof(*mpll), GFP_KERNEL);
	if (!mpll)
		return ERR_PTR(-ENOMEM);

	/* init mpll */
	mpll->hw.init = &init;
	mpll->regs = regs;

	clk = clk_register(NULL, &mpll->hw);
	if (IS_ERR(clk)) {
		pr_err("mem_pll: failed to register clk\n");
		kfree(mpll);
	}

	return clk;
}

static struct clk *refo_clk_register(const char *name,
				     const char **parents,
				     u32 nr_parents,
				     void __iomem *regs,
				     u32 *parent_idx)
{
	struct pic32_refosc *refo;
	struct clk_init_data init;
	struct clk *clk;
	int clk_flags = CLK_IS_BASIC;

	init_clk_data(init, name, parents, nr_parents, clk_flags, &roclk_ops);

	refo = kmalloc(sizeof(*refo), GFP_KERNEL);
	if (!refo)
		return ERR_PTR(-ENOMEM);

	/* initialize configuration */
	refo->regs = regs;
	refo->hw.init = &init;
	refo->parent_idx = parent_idx;

	clk = clk_register(NULL, &refo->hw);
	if (IS_ERR(clk))
		kfree(refo);

	return clk;
}

static void __init of_sosc_clk_setup(struct device_node *np)
{
	u32 rate, stsmask, bitmask;
	struct pic32_sosc *sosc;
	struct clk *clk;
	struct clk_init_data init;
	void __iomem *regs, *status_reg;
	const char *name = np->name;

	if (of_property_read_u32(np, "clock-frequency", &rate))
		return;

	of_property_read_string(np, "clock-output-names", &name);

	regs = of_iomap(np, 0);
	if (!regs)
		regs = pic32_clk_regbase;

	status_reg = of_iomap(np, 1);
	if (!status_reg)
		status_reg = regs;

	if (of_property_read_u32(np, "microchip,bit-mask", &bitmask))
		bitmask = OSC_SOSCEN_MASK;

	if (of_property_read_u32(np, "microchip,status-bit-mask", &stsmask))
		stsmask = OSC_SOSC_READY;

	/* allocate fixed-rate clock */
	sosc = kzalloc(sizeof(*sosc), GFP_KERNEL);
	if (!sosc)
		return;

	init_clk_data(init, name, NULL, 0,
		      CLK_IS_BASIC | CLK_IS_ROOT, &sosc_ops);

	/* struct clk assignments */
	sosc->fixed_rate = rate;
	sosc->hw.init = &init;
	sosc->regs = regs;
	sosc->status_reg = status_reg;
	sosc->bitmask = bitmask;
	sosc->status_bitmask = stsmask;

	/* register the clock */
	clk = clk_register(NULL, &sosc->hw);
	if (IS_ERR(clk))
		kfree(sosc);
	else
		pic32_of_clk_register_clkdev(np, clk);
}

static void __init of_periph_clk_setup(struct device_node *np)
{
	const char *parent_name;
	const char *name = np->name;
	struct clk *clk;
	u32 flags = 0;
	void __iomem *regs;

	regs = of_iomap(np, 0);
	if (!regs) {
		pr_err("%s: could not get reg property\n", name);
		return;
	}

	parent_name = of_clk_get_parent_name(np, 0);
	if (!parent_name) {
		pr_err("pbclk: %s must have a parent\n", name);
		goto err_map;
	}

	if (of_find_property(np, "microchip,ignore-unused", NULL)) {
		flags |= CLK_IGNORE_UNUSED;
		pr_info("%s: ignore gating even if unused\n", name);
	}

	of_property_read_string(np, "clock-output-names", &name);

	/* register peripheral clock */
	clk = periph_clk_register(name, &parent_name, regs, flags);
	if (IS_ERR(clk)) {
		pr_err("%s: could not register clock\n", name);
		goto err_map;
	}

	pic32_of_clk_register_clkdev(np, clk);

	/* add clk aliases, if available */
	pic32_of_clk_add_aliases(np, clk);

	return;

err_map:
	iounmap(regs);
}

static void __init of_refo_clk_setup(struct device_node *np)
{
	struct clk *clk;
	int ret, i, count;
	const char **parents;
	const char *clk_name = np->name;
	void __iomem *regs;
	u32 *parent_idx, rate;

	/* get the input clock source count */
	count = of_clk_get_parent_count(np);
	if (count < 0) {
		pr_err("%s: get clock count error\n", np->name);
		return;
	}

	parents = kzalloc((sizeof(char *) * count), GFP_KERNEL);
	if (!parents)
		return;

	ret = pic32_of_clk_get_parent_indices(np, &parent_idx, count);
	if (ret)
		goto err_parent;

	for (i = 0; i < count; i++)
		parents[i] = of_clk_get_parent_name(np, i);

	/* get iobase */
	regs = of_iomap(np, 0);
	if (!regs) {
		pr_err("%s: could not get reg property\n", np->name);
		goto err_parent_idx;
	}

	of_property_read_string(np, "clock-output-names", &clk_name);

	clk = refo_clk_register(clk_name, parents, count, regs, parent_idx);
	if (IS_ERR(clk)) {
		pr_err("%s: could not register clock\n", clk_name);
		goto err_map;
	}

	/* by default refclk(s) are disabled and unconfigured until
	 * requested by client module(s) & programmaed to enable & set rate.
	 * Instead, if provided by OF, driver can program default clock rate.
	 */
	ret = of_property_read_u32(np, "clock-frequency", (u32 *)&rate);
	if (!ret) {
		/* clk get & prepare */
		__clk_get(clk);
		clk_prepare(clk);

		/* set initial rate */
		clk_set_rate(clk, rate);

		/* keep it enabled */
		clk_enable(clk);
	}

	pic32_of_clk_register_clkdev(np, clk);

	/* add clk aliases, if available */
	pic32_of_clk_add_aliases(np, clk);

	goto err_parent;

err_map:
	iounmap(regs);
err_parent_idx:
	kfree(parent_idx);
err_parent:
	kfree(parents);
}

static void __init of_sys_mux_slew_setup(struct device_node *np)
{
	struct clk *clk;
	int ret, i, count;
	const char *clk_name;
	const char **parents;
	u32 *parent_idx, slew, v;
	unsigned long flags;
	void __iomem *slew_reg;

	/* get the input clock source count */
	count = of_clk_get_parent_count(np);
	if (count < 0) {
		pr_err("%s: get clock count error\n", np->name);
		return;
	}

	parents = kzalloc((sizeof(char *) * count), GFP_KERNEL);
	if (!parents)
		return;

	ret = pic32_of_clk_get_parent_indices(np, &parent_idx, count);
	if (ret)
		goto err_name;

	for (i = 0; i < count; i++) {
		parents[i] = of_clk_get_parent_name(np, i);
		__clk_debug("%s: parent[%d] = %s, idx %d\n", np->name, i,
			    parents[i], parent_idx ? parent_idx[i] : 0);
	}

	ret = of_property_read_string_index(np, "clock-output-names",
					    0, &clk_name);
	if (ret)
		clk_name = np->name;

	/* get slew base */
	slew_reg = of_iomap(np, 0);
	if (!slew_reg) {
		pr_warn("%s: no slew register ?\n", clk_name);
		goto err_name;
	}

	/* register mux clk */
	clk = sys_mux_clk_register(clk_name, parents, count, pic32_clk_regbase,
				   slew_reg, parent_idx, &sclk_postdiv_ops);
	if (IS_ERR(clk)) {
		pr_err("%s: could not register clock\n", clk_name);
		goto err_parent_idx;
	}

	/* enable slew, if asked */
	if (!of_property_read_u32(np, "microchip,slew-step", &slew)) {
		__clk_lock(flags);

		v = clk_readl(slew_reg);
		/* Apply new slew-div and enable up/down slewing */
		v &= ~(SLEW_DIV << SLEW_DIV_SHIFT);
		v |= (slew << SLEW_DIV_SHIFT);
		v |= SLEW_DOWNEN | SLEW_UPEN;
		clk_writel(v, slew_reg);

		__clk_unlock(flags);
	}

	/* register clkdev */
	pic32_of_clk_register_clkdev(np, clk);

	goto err_name;

err_parent_idx:
	iounmap(slew_reg);
	kfree(parent_idx);
err_name:
	kfree(parents);
}

static void __init of_sys_pll_setup(struct device_node *np)
{
	int i, count;
	const char *clk_name = np->name;
	const char **parent_names;
	const char *plliclk_name = "spll_mux_clk";
	void __iomem *regs, *stat_reg;
	struct clk *clk, *mux_clk;
	u32 bitmask;

	/* get the input clock source count */
	count = of_clk_get_parent_count(np);
	if (count < 0) {
		pr_err("%s: get clock count error, %d\n", np->name, count);
		return;
	}

	parent_names = kzalloc((sizeof(char *) * count), GFP_KERNEL);
	if (!parent_names)
		return;

	for (i = 0; i < count; i++)
		parent_names[i] = of_clk_get_parent_name(np, i);

	/* get output name */
	of_property_read_string(np, "clock-output-names", &clk_name);

	/* get iobase */
	regs = of_iomap(np, 0);
	if (!regs) {
		pr_err("%s: of_iomap failed\n", np->name);
		goto err_name;
	}

	/* get status reg & status bitmask */
	stat_reg = of_iomap(np, 1);

	if (of_property_read_u32(np, "microchip,status-bit-mask", &bitmask)) {
		/* OSC_SPLL_LOCKED (bit5 of reg OSCCON) does not behave as
		 * documented. Bit is asserted if SYSCLK uses SPLL as clksrc and
		 * vice-versa. So use of this bit to detect success or failure
		 * might not be reliable.
		 */
		bitmask = OSC_SPLL_LOCKED;
		stat_reg = regs - 0x20; /* OSCCON */
	}

	/* register plliclk mux */
	mux_clk = clk_register_mux(NULL, plliclk_name, parent_names,
				   count, 0, regs,
				   PLL_ICLK_SHIFT, 1, 0, &lock);
	if (IS_ERR(mux_clk))  {
		pr_err("splliclk_mux not registered\n");
		goto err_unmap;
	}

	/* register sys-pll clock */
	clk = spll_clk_register(clk_name, plliclk_name,
				regs, stat_reg, bitmask);
	if (IS_ERR(clk)) {
		pr_err("spll_clk not registered\n");
		goto err_mux;
	}

	pic32_of_clk_register_clkdev(np, clk);
	goto err_name;

err_mux:
	clk_unregister(mux_clk);
err_unmap:
	iounmap(regs);
err_name:
	kfree(parent_names);
}

static void __init of_mem_pll_setup(struct device_node *np)
{
	void __iomem *regs;
	struct clk *clk;
	const char *parent_name;
	const char *clk_name = np->name;

	/* get parent name */
	parent_name = of_clk_get_parent_name(np, 0);

	/* get output name */
	of_property_read_string(np, "clock-output-names", &clk_name);

	/* get iobase */
	regs = of_iomap(np, 0);
	if (!regs) {
		pr_err("%s: of_iomap failed\n", np->name);
		return;
	}

	/* register memory-pll clock */
	clk = mpll_clk_register(clk_name, parent_name, regs);
	if (IS_ERR(clk)) {
		pr_err("spll_clk: not registered\n");
		goto err_unmap;
	}

	pic32_of_clk_register_clkdev(np, clk);
	return;

err_unmap:
	iounmap(regs);
}

static void __init of_frcdiv_setup(struct device_node *np)
{
	struct clk *clk;
	const char *clk_name = np->name;
	const char *parent_name;

	parent_name = of_clk_get_parent_name(np, 0);
	if (!parent_name) {
		pr_err("frcdiv: %s must have a parent\n", np->name);
		return;
	}

	/* clk name */
	of_property_read_string(np, "clock-output-names", &clk_name);

	/* divider clock register */
	clk = clk_register_divider(NULL, clk_name, parent_name,
				   0, pic32_clk_regbase,
				   OSC_FRCDIV_SHIFT, OSC_FRCDIV_MASK,
				   CLK_DIVIDER_POWER_OF_TWO, &lock);

	if (IS_ERR_OR_NULL(clk)) {
		pr_err("frcdiv_clk not registered\n");
		return;
	}

	pic32_of_clk_register_clkdev(np, clk);
}

static const struct of_device_id pic32_clk_match[] __initconst = {
	{ .compatible = "microchip,pic32-refoclk", .data = of_refo_clk_setup, },
	{ .compatible = "microchip,pic32-pbclk", .data = of_periph_clk_setup, },
	{ .compatible = "microchip,pic32-syspll", .data = of_sys_pll_setup, },
	{ .compatible = "microchip,pic32-sosc",	.data = of_sosc_clk_setup, },
	{ .compatible = "microchip,pic32-frcdivclk", .data = of_frcdiv_setup, },
	{ .compatible = "microchip,pic32-mpll", .data = of_mem_pll_setup, },
	{ .compatible = "microchip,pic32-sysclk-v2",
		.data = of_sys_mux_slew_setup,},
	{}
};

static irqreturn_t pic32_fscm_isr_handler(int irq, void *data)
{
	u32 v = clk_readl(pic32_clk_regbase);

	if (v & OSC_CLK_FAILED)
		pr_info("pic32-clk: FSCM detected clk failure.\n");

	return IRQ_HANDLED;
}

static int pic32_fscm_nmi(struct notifier_block *nb,
			  unsigned long action, void *data)
{
	pic32_fscm_isr_handler(0, NULL);
	return NOTIFY_OK;
}

static struct notifier_block failsafe_clk_notifier = {
	.notifier_call = pic32_fscm_nmi,
};

static void __init of_pic32_soc_clock_init(struct device_node *np)
{
	int ret, nmi = 0, irq;
	struct resource r;
	struct device_node *childnp;
	const struct of_device_id *clk_id;
	void (*clk_setup)(struct device_node *);

	if (of_address_to_resource(np, 0, &r))
		panic("Failed to get clk-pll memory region\n");

	if (!request_mem_region(r.start, resource_size(&r), r.name))
		panic("%s: request_region failed\n", np->name);

	pic32_clk_regbase = ioremap_nocache(r.start, resource_size(&r));
	if (!pic32_clk_regbase)
		panic("pic32-clk: failed to map registers\n");

	irq = irq_of_parse_and_map(np, 0);
	if (!irq) {
		pr_warn("pic32-clk: irq not provided for FSCM; uses nmi.\n");
		nmi = 1;
	}

	for_each_child_of_node(np, childnp) {
		clk_id = of_match_node(pic32_clk_match, childnp);
		if (!clk_id)
			continue;
		clk_setup = clk_id->data;
		clk_setup(childnp);
	}

	/* register irq/nmi */
	if (nmi) {
		register_nmi_notifier(&failsafe_clk_notifier);
	} else {
		ret = request_irq(irq, pic32_fscm_isr_handler, 0, "fscm", NULL);
		if (ret)
			pr_err("pic32-clk: fscm_irq request failed\n");
	}
}

CLK_OF_DECLARE(pic32_soc_clk, "microchip,pic32-clk", of_pic32_soc_clock_init);
