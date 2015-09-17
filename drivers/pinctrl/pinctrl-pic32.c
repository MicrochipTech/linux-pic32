/*
 * pic32 pinctrl core driver.
 *
 * Copyright (C) 2014 Microchip Technology, Inc.
 * Author: Sorin-Andrei Pistirica <andrei.pistirica@microchip.com>
 *
 * Licensed under GPLv2 or later.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>

#include <linux/pinctrl/consumer.h>

#include "core.h"
#include "pinctrl-pic32.h"

/* struct pic32_gpio_irq - pic32 gpio irq descriptor
 * @gpio_irqchip: irq chip descriptor related to pin bank
 * @domain: associated irq domain
 * @pio_irq: PIO bank hardware interrupt
 * @port_shadow: shadow port value (detect fall/rise irqs)
 * @type: type of interrupt per pin: RISE, FALL or BOTH.
 **/
struct pic32_gpio_irq {
	struct irq_chip		gpio_irqchip;
	struct irq_domain	*domain;
	int			pio_irq;

	u32			port_shadow;
	u32			type[PINS_PER_BANK];
};

/* struct pic32_gpio_chip - pic32 gpio chip descriptor
 * @chip: gpio chip descriptor
 * @pio_base: port's registers base address
 * @reg_lookup_off: registers offsets (map may vary within pic32 family chips)
 * @pio_idx: port index (0-to-10 alias A-to-J)
 * @clk: associated clock
 * @gpio_irq: pic32 gpio irq descriptor
 * @range: pin ranges as gpio in pinctrl device
 **/
struct pic32_gpio_chip {
	struct gpio_chip		chip;
	void __iomem			*pio_base;
	unsigned			*reg_lookup_off;
	unsigned			lookup_size;
	int				pio_idx;
	int				gpio_base;
	int				ngpio;
	struct clk			*clk;

	struct pic32_gpio_irq		gpio_irq;
	struct pinctrl_gpio_range	range;
};
#define to_pic32_gpio_chip(c) container_of(c, struct pic32_gpio_chip, chip)

static struct pic32_gpio_chip *gpio_chips[MAX_PIO_BANKS];

/* get a specific pic32:pio register */
static void __iomem *pic32_pio_get_reg(struct pic32_gpio_chip *pic32_chip,
				       enum pic32_pio_regs pic32_reg)
{
	void __iomem *base = pic32_chip->pio_base;
	unsigned *lookup = pic32_chip->reg_lookup_off;
	unsigned lookup_size = pic32_chip->lookup_size;
	unsigned off;

	PIC32_CHECK_REG_OFF(pic32_reg, lookup, lookup_size);

	off = lookup[pic32_reg];

	return base + off;
}

/* set pin to open-drain */
static int pic32_pinconf_open_drain(struct pic32_gpio_chip *pic32_chip,
				    unsigned pin, int value)
{
	struct pic32_reg __iomem *odc_reg = (struct pic32_reg __iomem *)
			pic32_pio_get_reg(pic32_chip, PIC32_ODC);
	u32 mask = BIT(pin);

	if (value)
		/* NOTE: I/O direction will be changed to out */
		writel(mask, &odc_reg->set);
	else
		writel(mask, &odc_reg->clr);

	dev_dbg(pic32_chip->chip.dev, "%s: OPEN-DRAIN pin (%u:%u)\n", __func__,
						pic32_chip->pio_idx, pin);

	return 0;
}

/* set pin to pull-up */
static int pic32_pinconf_pullup(struct pic32_gpio_chip *pic32_chip,
				unsigned pin, int value)
{
	struct pic32_reg __iomem *cnpu_reg = (struct pic32_reg __iomem *)
			pic32_pio_get_reg(pic32_chip, PIC32_CNPU);
	u32 mask = BIT(pin);

	if (value)
		writel(mask, &cnpu_reg->set);
	else
		writel(mask, &cnpu_reg->clr);

	dev_dbg(pic32_chip->chip.dev, "%s: PULL-UP pin (%u:%u)\n", __func__,
						pic32_chip->pio_idx, pin);

	return 0;
}

/* set pin to pull-down */
static int pic32_pinconf_pulldown(struct pic32_gpio_chip *pic32_chip,
				  unsigned pin, int value)
{
	struct pic32_reg __iomem *cnpd_reg = (struct pic32_reg __iomem *)
			pic32_pio_get_reg(pic32_chip, PIC32_CNPD);
	u32 mask = BIT(pin);

	if (value)
		writel(mask, &cnpd_reg->set);
	else
		writel(mask, &cnpd_reg->clr);

	dev_dbg(pic32_chip->chip.dev, "%s: PULL-DOWN pin (%u:%u)\n", __func__,
						pic32_chip->pio_idx, pin);

	return 0;
}

/* set pin to analog */
static int pic32_pinconf_analog(struct pic32_gpio_chip *pic32_chip,
				unsigned pin)
{
	struct pic32_reg __iomem *ansel_reg = (struct pic32_reg __iomem *)
				pic32_pio_get_reg(pic32_chip, PIC32_ANSEL);
	u32 mask = BIT(pin);

	if (pin >= pic32_chip->chip.ngpio)
		BUG();

	writel(mask, &ansel_reg->set);

	dev_dbg(pic32_chip->chip.dev, "%s: ANALOG pin (%u:%u)\n", __func__,
						pic32_chip->pio_idx, pin);

	return 0;
}

/* set pin as digital */
static int pic32_pinconf_dg(struct pic32_gpio_chip *pic32_chip, unsigned pin)
{
	struct pic32_reg __iomem *ansel_reg = (struct pic32_reg __iomem *)
				pic32_pio_get_reg(pic32_chip, PIC32_ANSEL);
	u32 mask = BIT(pin);

	if (pin >= pic32_chip->chip.ngpio)
		BUG();

	writel(mask, &ansel_reg->clr);

	dev_dbg(pic32_chip->chip.dev, "%s: DIGITAL pin (%u:%u)\n", __func__,
						pic32_chip->pio_idx, pin);

	return 0;
}

/* set pin dir according to configuration */
static int pic32_pinconf_set_dir(struct pic32_gpio_chip *pic32_chip,
				 unsigned pin, unsigned long conf)
{
	struct pin_conf *pinconf = (struct pin_conf *)&conf;
	struct pic32_reg __iomem *tris_reg = (struct pic32_reg __iomem *)
				pic32_pio_get_reg(pic32_chip, PIC32_TRIS);

	u32 mask = BIT(pin);

	if (pin >= pic32_chip->chip.ngpio)
		BUG();

	if (pinconf->dir == DIR_IN)
		writel(mask, &tris_reg->set);
	else if (pinconf->dir == DIR_OUT)
		writel(mask, &tris_reg->clr);
	else
		return -1;

	dev_dbg(pic32_chip->chip.dev, "%s: pin (%u:%u), DIR:%s\n", __func__,
			pic32_chip->pio_idx, pin,
			pinconf->dir == DIR_IN ? "IN" : "OUT");

	return 0;
}

/* exported wrappers - runtime manipulation */
static struct pic32_gpio_chip *gpio_to_pic32_gpio_chip(unsigned pin_id)
{
	unsigned bank = pin_id / PINS_PER_BANK;
	unsigned pin = pin_id % PINS_PER_BANK;

	if (pin > PINS_PER_BANK || bank >= MAX_PIO_BANKS)
		BUG();

	return gpio_chips[bank];
}

int pic32_pinconf_open_drain_runtime(unsigned pin_id, int value)
{
	struct pic32_gpio_chip *pic32_chip = gpio_to_pic32_gpio_chip(pin_id);
	unsigned pin = pin_id % PINS_PER_BANK;

	if (!pic32_chip)
		return -ENODEV;

	return pic32_pinconf_open_drain(pic32_chip, pin, value);
}
EXPORT_SYMBOL(pic32_pinconf_open_drain_runtime);

int pic32_pinconf_pullup_runtime(unsigned pin_id, int value)
{
	struct pic32_gpio_chip *pic32_chip = gpio_to_pic32_gpio_chip(pin_id);
	unsigned pin = pin_id % PINS_PER_BANK;

	if (!pic32_chip)
		return -ENODEV;

	return pic32_pinconf_pullup(pic32_chip, pin, value);
}
EXPORT_SYMBOL(pic32_pinconf_pullup_runtime);

int pic32_pinconf_pulldown_runtime(unsigned pin_id, int value)
{
	struct pic32_gpio_chip *pic32_chip = gpio_to_pic32_gpio_chip(pin_id);
	unsigned pin = pin_id % PINS_PER_BANK;

	if (!pic32_chip)
		return -ENODEV;

	return pic32_pinconf_pulldown(pic32_chip, pin, value);
}
EXPORT_SYMBOL(pic32_pinconf_pulldown_runtime);

int pic32_pinconf_analog_runtime(unsigned pin_id)
{
	struct pic32_gpio_chip *pic32_chip = gpio_to_pic32_gpio_chip(pin_id);
	unsigned pin = pin_id % PINS_PER_BANK;

	if (!pic32_chip)
		return -ENODEV;

	return pic32_pinconf_analog(pic32_chip, pin);
}
EXPORT_SYMBOL(pic32_pinconf_analog_runtime);

int pic32_pinconf_dg_runtime(unsigned pin_id)
{
	struct pic32_gpio_chip *pic32_chip = gpio_to_pic32_gpio_chip(pin_id);
	unsigned pin = pin_id % PINS_PER_BANK;

	if (!pic32_chip)
		return -ENODEV;

	return pic32_pinconf_dg(pic32_chip, pin);
}
EXPORT_SYMBOL(pic32_pinconf_dg_runtime);

static int pic32_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	int gpio = chip->base + offset;
	int bank = chip->base / chip->ngpio;

	dev_dbg(chip->dev, "%s: request GPIO-%c:%d(%d)\n", __func__,
		 'A' + bank, offset, gpio);

	return pinctrl_request_gpio(gpio);
}

static void pic32_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	int gpio = chip->base + offset;
	int bank = chip->base / chip->ngpio;

	dev_dbg(chip->dev, "%s: free GPIO-%c:%d(%d)\n", __func__,
		 'A' + bank, offset, gpio);

	pinctrl_free_gpio(gpio);
}

static void pic32_gpio_set(struct gpio_chip *chip, unsigned gpio, int val)
{
	struct pic32_gpio_chip *pic32_chip = to_pic32_gpio_chip(chip);
	struct pic32_reg __iomem *port_reg = (struct pic32_reg __iomem *)
				pic32_pio_get_reg(pic32_chip, PIC32_PORT);

	u32 mask = BIT(gpio);

	if (gpio >= chip->ngpio)
		BUG();

	if (val)
		writel(mask, &port_reg->set);
	else
		writel(mask, &port_reg->clr);
}

static int pic32_gpio_get(struct gpio_chip *chip, unsigned gpio)
{
	struct pic32_gpio_chip *pic32_chip = to_pic32_gpio_chip(chip);
	struct pic32_reg __iomem *port_reg = (struct pic32_reg __iomem *)
				pic32_pio_get_reg(pic32_chip, PIC32_PORT);

	u32 mask = BIT(gpio);

	if (gpio >= chip->ngpio)
		BUG();

	return readl(&port_reg->val) & mask;
}

static int pic32_gpio_get_dir(struct gpio_chip *chip, unsigned offset)
{
	struct pic32_gpio_chip *pic32_chip = to_pic32_gpio_chip(chip);
	struct pic32_reg __iomem *tris_reg = (struct pic32_reg __iomem *)
				pic32_pio_get_reg(pic32_chip, PIC32_TRIS);

	u32 mask = BIT(offset);

	return readl(&tris_reg->val) & mask;
}

static int pic32_gpio_set_dir(struct gpio_chip *chip, unsigned gpio, int dir)
{
	struct pic32_gpio_chip *pic32_chip = to_pic32_gpio_chip(chip);
	struct pic32_reg __iomem *tris_reg = (struct pic32_reg __iomem *)
				pic32_pio_get_reg(pic32_chip, PIC32_TRIS);

	u32 mask = BIT(gpio);

	if (gpio >= chip->ngpio)
		BUG();

	/* clear analog selection when digital input is required! */
	pic32_pinconf_dg(pic32_chip, gpio);

	if (dir == DIR_IN)
		writel(mask, &tris_reg->set);
	else if (dir == DIR_OUT)
		writel(mask, &tris_reg->clr);

	return 0;
}

static int pic32_gpio_dir_in(struct gpio_chip *chip, unsigned gpio)
{
	return pic32_gpio_set_dir(chip, gpio, DIR_IN);
}

static int pic32_gpio_dir_out(struct gpio_chip *chip,
			      unsigned gpio, int value)
{
	struct pic32_gpio_chip *pic32_chip = to_pic32_gpio_chip(chip);
	struct pic32_reg __iomem *cnpu_reg = (struct pic32_reg __iomem *)
				pic32_pio_get_reg(pic32_chip, PIC32_CNPU);
	struct pic32_reg __iomem *cnpd_reg = (struct pic32_reg __iomem *)
				pic32_pio_get_reg(pic32_chip, PIC32_CNPD);

	u32 mask = BIT(gpio);

	if (gpio >= chip->ngpio)
		BUG();

	/* clear open-drain for value of 1 */
	if (value)
		pic32_pinconf_open_drain(pic32_chip, gpio, 0);

	/* set required value */
	pic32_gpio_set(chip, gpio, value);

	/* clear when digital output is required! */
	writel(mask, &cnpu_reg->clr); /* pull-up */
	writel(mask, &cnpd_reg->clr); /* pull-down */

	return pic32_gpio_set_dir(chip, gpio, DIR_OUT);
}

static int pic32_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct pic32_gpio_chip *pic32_chip = to_pic32_gpio_chip(chip);
	struct pic32_gpio_irq *gpio_irq = &pic32_chip->gpio_irq;
	int virq;

	if (offset < chip->ngpio)
		virq = irq_create_mapping(gpio_irq->domain, offset);
	else
		virq = -ENXIO;

	dev_dbg(chip->dev, "%s: request IRQ for GPIO:%d, return:%d\n",
				__func__, offset + chip->base, virq);
	return virq;
}

#ifdef CONFIG_DEBUG_FS
static void pic32_gpio_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
	int i;

	for (i = 0; i < chip->ngpio; i++) {
		/*unsigned pin = chip->base + i;*/
		const char *gpio_label;

		gpio_label = gpiochip_is_requested(chip, i);
		if (!gpio_label)
			continue;

		seq_printf(s, "%s: GPIO-%s:%d\n", gpio_label, chip->label, i);
	}
}
#else
#define pic32_gpio_dbg_show	NULL
#endif

static void pic32_gpio_ranges_setup(struct platform_device *pdev,
			     struct pic32_gpio_chip *pic32_chip)
{
	struct device_node *np = pdev->dev.of_node;
	struct pinctrl_gpio_range *range;
	struct of_phandle_args args;
	int ret;

	ret = of_parse_phandle_with_fixed_args(np, "gpio-ranges", 3, 0, &args);
	pic32_chip->gpio_base = (ret == 0) ? args.args[1] + args.args[0] :
					pic32_chip->pio_idx * PINS_PER_BANK;
	pic32_chip->ngpio = (ret == 0) ? args.args[2] - args.args[0] :
					PINS_PER_BANK;

	range = &pic32_chip->range;
	range->name = dev_name(&pdev->dev);
	range->id = pic32_chip->pio_idx;
	range->pin_base = range->base = pic32_chip->gpio_base;

	range->npins = pic32_chip->ngpio;
	range->gc = &pic32_chip->chip;

	dev_dbg(&pdev->dev, "%s: GPIO-%c ranges: (%d,%d)\n", __func__,
				'A' + range->id,
				pic32_chip->gpio_base, pic32_chip->ngpio);

	return;
}

static unsigned int gpio_irq_startup(struct irq_data *d)
{
	struct pic32_gpio_chip *pic32_chip = irq_data_get_irq_chip_data(d);
	unsigned pin = d->hwirq;
	int ret;

	ret = gpiochip_lock_as_irq(&pic32_chip->chip, pin);
	if (ret) {
		dev_err(pic32_chip->chip.dev, "unable to lock pind %lu IRQ\n",
			d->hwirq);
		return ret;
	}

	dev_dbg(pic32_chip->chip.dev, "%s: irq lock pin:%u\n", __func__, pin);

	return 0;
}

static void gpio_irq_shutdown(struct irq_data *d)
{
	struct pic32_gpio_chip *pic32_chip = irq_data_get_irq_chip_data(d);
	unsigned pin = d->hwirq;

	gpiochip_unlock_as_irq(&pic32_chip->chip, pin);

	dev_dbg(pic32_chip->chip.dev, "%s: irq unlock:%u\n", __func__, pin);
}

static int gpio_irq_type(struct irq_data *d, unsigned type)
{
	struct pic32_gpio_chip *pic32_chip = irq_data_get_irq_chip_data(d);
	struct pic32_gpio_irq *gpio_irq = &pic32_chip->gpio_irq;
	unsigned pin = d->hwirq;

	dev_dbg(pic32_chip->chip.dev, "%s: irq type:%u\n", __func__, type);

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
	case IRQ_TYPE_EDGE_FALLING:
	case IRQ_TYPE_EDGE_BOTH:
		gpio_irq->type[pin] = type;
		return IRQ_SET_MASK_OK;
	default:
		gpio_irq->type[pin] = IRQ_TYPE_NONE;
		return -EINVAL;
	}
}

static inline void
pic32_gpio_irq_shadow_set(struct pic32_gpio_chip *pic32_chip,
			  u32 shadow_val)
{
	struct pic32_gpio_irq *gpio_irq = &pic32_chip->gpio_irq;

	gpio_irq->port_shadow = shadow_val;
}

static inline u32
pic32_gpio_irq_shadow_get(struct pic32_gpio_chip *pic32_chip)
{
	struct pic32_gpio_irq *gpio_irq = &pic32_chip->gpio_irq;
	u32 shadow;

	shadow = gpio_irq->port_shadow;
	return shadow;
}

/* map virtual irq on hw irq: domain translation */
static int pic32_gpio_irq_map(struct irq_domain *d,
			      unsigned int virq,
			      irq_hw_number_t hw)
{
	struct pic32_gpio_chip *pic32_chip = d->host_data;
	struct pic32_gpio_irq *gpio_irq = &pic32_chip->gpio_irq;
	struct irq_chip *irqchip = &gpio_irq->gpio_irqchip;
	struct pic32_reg __iomem *port_reg = (struct pic32_reg __iomem *)
				pic32_pio_get_reg(pic32_chip, PIC32_PORT);
	struct pic32_reg __iomem *cncon_reg = (struct pic32_reg __iomem *)
				pic32_pio_get_reg(pic32_chip, PIC32_CNCON);
	struct pic32_reg __iomem *cnen_reg = (struct pic32_reg __iomem *)
				pic32_pio_get_reg(pic32_chip, PIC32_CNEN);
	u32 cn_en = BIT(PIC32_CNCON_BIT);
	u32 cnpin_mask = BIT(virq); /* virq is actually the pin */
	unsigned long flags;

	dev_dbg(pic32_chip->chip.dev, "%s: GPIO-%c:%d map virq:%u\n", __func__,
				'A' + pic32_chip->pio_idx, virq, virq);

	/* enable irq on pin 'virq' and protect shadow */
	local_irq_save(flags);

	/* enable CN module */
	writel(cn_en, &cncon_reg->set);

	/* enable PIN */
	writel(cnpin_mask, &cnen_reg->set);

	/* set shadow to diferentiate between RISE and FALL edges */
	pic32_gpio_irq_shadow_set(pic32_chip, readl(&port_reg->val));

	local_irq_restore(flags);

	/* set the gpioX chip */
	irq_set_chip(virq, irqchip);
	irq_set_chip_data(virq, pic32_chip);
	irq_set_handler(virq, handle_simple_irq);

	return 0;
}

/* decode irq number: base + pin */
static int pic32_gpio_irq_domain_xlate(struct irq_domain *d,
				       struct device_node *ctrlr,
				       const u32 *intspec,
				       unsigned int intsize,
				       irq_hw_number_t *out_hwirq,
				       unsigned int *out_type)
{
	struct pic32_gpio_chip *pic32_chip = d->host_data;
	int pin = pic32_chip->chip.base + intspec[0];
	int ret;

	if (WARN_ON(intsize < 2))
		return -EINVAL;

	ret = gpio_request(pin, ctrlr->full_name);
	if (ret)
		return ret;

	*out_hwirq = intspec[0];
	*out_type = intspec[1] & IRQ_TYPE_SENSE_MASK;

	/* pic32_gpio_dir_in */
	ret = gpio_direction_input(pin);
	if (ret)
		return ret;

	dev_dbg(pic32_chip->chip.dev, "%s: xlate pin:%d\n", __func__, pin);
	return 0;
}

static struct irq_domain_ops pic32_gpio_irqd_ops = {
	.map	= pic32_gpio_irq_map,
	.xlate	= pic32_gpio_irq_domain_xlate,
};

static unsigned long pic32_gpio_to_isr(struct pic32_gpio_chip *pic32_chip,
				       int cnstat, int portval)
{
	struct pic32_gpio_irq *gpio_irq = &pic32_chip->gpio_irq;
	u32 shadow = pic32_gpio_irq_shadow_get(pic32_chip);
	unsigned long isr = (unsigned long)cnstat;
	unsigned long visr = 0;
	int pin;

	/* for each change that occured, match with irq type and
	 *   set it accordingly.
	 */
	visr = 0;
	for_each_set_bit(pin, &isr, BITS_PER_BYTE * sizeof(u32)) {
		u32 mask = BIT(pin);
		bool type_rise = gpio_irq->type[pin] == IRQ_TYPE_EDGE_RISING;
		bool type_fall = gpio_irq->type[pin] == IRQ_TYPE_EDGE_FALLING;
		bool type_both = gpio_irq->type[pin] == IRQ_TYPE_EDGE_BOTH;
		bool rise = !(shadow & mask) && (portval & mask);
		bool fall = (shadow & mask) && !(portval & mask);

		if ((type_rise && rise) || (type_fall && fall) || type_both)
			visr |= mask;
	}

	pic32_gpio_irq_shadow_set(pic32_chip, portval);

	return visr;
}

static void gpio_irq_handler(unsigned irq, struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct irq_data *idata = irq_desc_get_irq_data(desc);
	struct pic32_gpio_chip *pic32_chip = irq_data_get_irq_chip_data(idata);
	struct pic32_gpio_irq *gpio_irq = &pic32_chip->gpio_irq;
	struct pic32_reg __iomem *cnstat_reg = (struct pic32_reg __iomem *)
				pic32_pio_get_reg(pic32_chip, PIC32_CNSTAT);
	struct pic32_reg __iomem *port_reg = (struct pic32_reg __iomem *)
				pic32_pio_get_reg(pic32_chip, PIC32_PORT);
	int cnstat;
	int portval;
	unsigned long isr;
	unsigned long flags;
	int n;

	/* protect shadow */
	local_irq_save(flags);

	/* read & clear port's change notificaiton register */
	cnstat = readl(&cnstat_reg->val);
	portval = readl(&port_reg->val);

	dev_dbg(pic32_chip->chip.dev, "%s: cnstat:0x%x port:0x%x pioirq:%u\n",
		 __func__, cnstat, portval, gpio_irq->pio_irq);

	chained_irq_enter(chip, desc);

	/* set a logical isr based on type and status */
	isr = pic32_gpio_to_isr(pic32_chip, cnstat, portval);

	/* for each interrupt, call handle */
	for_each_set_bit(n, &isr, BITS_PER_LONG) {
		unsigned int irq = irq_find_mapping(gpio_irq->domain, n);

		generic_handle_irq(irq);
	}

	chained_irq_exit(chip, desc);

	local_irq_restore(flags);

	return;
}

static int pic32_gpio_of_irq_setup(struct platform_device *pdev,
				   struct pic32_gpio_chip *pic32_chip)
{
	struct device_node *node = pdev->dev.of_node;
	struct pic32_gpio_irq *gpio_irq = &pic32_chip->gpio_irq;
	struct irq_chip *irqchip = &gpio_irq->gpio_irqchip;
	int base_irq;

	/* set irqchip */
	irqchip->name = kasprintf(GFP_KERNEL, "GPIO-%c",
						pic32_chip->pio_idx + 'A');
	irqchip->irq_startup	= gpio_irq_startup;
	irqchip->irq_shutdown	= gpio_irq_shutdown;
	irqchip->irq_set_type	= gpio_irq_type;

	base_irq = platform_get_irq(pdev, 0);
	if (base_irq < 0)
		return base_irq;

	gpio_irq->pio_irq = base_irq;

	/* Setup irq domain of ngpio lines */
	gpio_irq->domain = irq_domain_add_linear(
					node,
					pic32_chip->chip.ngpio,
					&pic32_gpio_irqd_ops, pic32_chip);
	if (!gpio_irq->domain) {
		dev_err(pic32_chip->chip.dev, "Couldn't allocate IRQ domain\n");
		return -ENXIO;
	}

	dev_dbg(&pdev->dev, "%s: irq GPIO-%c, base_irq:%d, domain:%d\n",
					 __func__, pic32_chip->pio_idx + 'A',
					base_irq, pic32_chip->chip.ngpio);

	/* setup chained handler */
	irq_set_chip_data(gpio_irq->pio_irq, pic32_chip);
	irq_set_chained_handler(gpio_irq->pio_irq, gpio_irq_handler);

	return 0;
}

static struct gpio_chip gpio_template = {
	.request		= pic32_gpio_request,
	.free			= pic32_gpio_free,
	.get_direction		= pic32_gpio_get_dir,
	.direction_input	= pic32_gpio_dir_in,
	.direction_output	= pic32_gpio_dir_out,
	.get			= pic32_gpio_get,
	.set			= pic32_gpio_set,
	.to_irq			= pic32_gpio_to_irq,
	.dbg_show		= pic32_gpio_dbg_show,
	.ngpio			= PINS_PER_BANK,
	.can_sleep		= false,
};

int pic32_gpio_probe(struct platform_device *pdev,
		     unsigned (*reg_lookup_off)[],
		     unsigned lookup_size)
{
	struct device_node *np = pdev->dev.of_node;
	int alias_idx = of_alias_get_id(np, "gpio");
	struct pic32_gpio_chip *pic32_chip = NULL;
	struct gpio_chip *chip;
	struct resource *r;
	int ret = 0;

	dev_dbg(&pdev->dev, "%s: probing...\n", __func__);

	if (!np)
		return -ENODEV;

	BUG_ON(alias_idx >= ARRAY_SIZE(gpio_chips));
	if (gpio_chips[alias_idx]) {
		dev_err(&pdev->dev, "Failure %i for GPIO %i\n", ret, alias_idx);
		return -EBUSY;
	}

	/* pic32 gpio chip - private data */
	pic32_chip = devm_kzalloc(&pdev->dev, sizeof(*pic32_chip),
								GFP_KERNEL);
	if (!pic32_chip)
		return -ENOMEM;

	/* base address of pio(alias_idx) registers */
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		ret = -EINVAL;
		goto probe_err;
	}
	pic32_chip->pio_base = devm_ioremap_nocache(&pdev->dev, r->start,
							resource_size(r));
	if (IS_ERR(pic32_chip->pio_base)) {
		ret = PTR_ERR(pic32_chip->pio_base);
		goto probe_err;
	}

	/* clocks */
	pic32_chip->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(pic32_chip->clk)) {
		ret = PTR_ERR(pic32_chip->clk);
		dev_err(&pdev->dev, "clk get failed\n");
		goto probe_err;
	}

	ret = clk_prepare_enable(pic32_chip->clk);
	if (ret) {
		dev_err(&pdev->dev, "clk enable failed\n");
		goto probe_err;
	}

	pic32_chip->reg_lookup_off = *reg_lookup_off;
	pic32_chip->lookup_size = lookup_size;
	pic32_chip->pio_idx = alias_idx;
	pic32_chip->chip = gpio_template;
	pic32_gpio_ranges_setup(pdev, pic32_chip);/* pin_ranges: unsupported */
	pic32_gpio_of_irq_setup(pdev, pic32_chip);/* irq domain */

	/* gpio chip */
	chip		= &pic32_chip->chip;
	chip->of_node	= np;
	chip->label	= dev_name(&pdev->dev);
	chip->dev	= &pdev->dev;
	chip->owner	= THIS_MODULE;
	chip->base	= pic32_chip->gpio_base;
	chip->ngpio	= pic32_chip->ngpio;

	ret = gpiochip_add(chip);
	if (ret)
		goto probe_err;
	gpio_chips[alias_idx] = pic32_chip;

	dev_info(&pdev->dev, "(%d) driver initialized.\n", alias_idx);
	return 0;

probe_err:
	if (pic32_chip->pio_base)
		devm_iounmap(&pdev->dev, pic32_chip->pio_base);
	if (pic32_chip) {
		devm_kfree(&pdev->dev, pic32_chip);
		gpio_chips[alias_idx] = NULL;
	}
	return ret;
}

static struct pic32_pinctrl_prop pinctrl_props[PIC32_PINCTRL_PROP_LAST] = {
	[PIC32_PINCTRL_PROP_SINGLE_PINS] =
		PIC32_PINCTRL_DT_PROP("pic32,single-pins", 3),
	[PIC32_PINCTRL_PROP_PINS] =
		PIC32_PINCTRL_DT_PROP("pic32,pins", 3),
};

/* struct pin_function - pic32 pin function descriptor
 * @name: the name of the pinmux function
 * @ngroups: number of groups related to this function
 * @groups: groups related to this function
 **/
struct pin_function {
	const char	*name;
	unsigned	ngroups;
	const char	**groups;
};

/* struct gspin - pic32 single-pins descriptor
 * @bank: the pin's bank(PORT)
 * @pin: the pin's id within bank(PORT)
 * @conf: the configuration of the pin: PULL_UP, MULTIDRIVE etc...
 **/
struct gspin {
	unsigned	bank;
	unsigned	pin;
	unsigned long	conf;
};

/* struct gpins - pic32 pinmux descriptor
 * @dir: pin direction (in or out)
 * @bucket: remappable pin bucket (A(0):D(3))
 * @bank: remappable pin's bank (A(0):K(9))
 * @pin: the remappable pin in the @bank
 * @ppin: peripheral pin mapped
 * @cod: pinmux code
 * @conf: the configuration of the pin: PULL_UP, MULTIDRIVE etc...
 **/
struct gpins {
	u8		dir;
	u8		bucket;
	u8		bank;
	u8		pin;
	u8		ppin;
	u16		cod;
	unsigned long	conf;
};

/* struct pin_group - pic32 pin group descriptor
 * @name: the name of the pin group
 * @npins: number of pins in this group (no. single-pins + no. pinmux)
 * @pins: array of pin ids (pinctrl forces to maintain such an array)
 * @ngspins: number of single pins within group
 * @gspins: descriptor of single pins within group
 * @ngpins: number of pinmux pins within group
 * @gpins: descriptor of pinmux pins within group
 **/
struct pin_group {
	const char	*name;

	unsigned	npins;
	unsigned int	*pins;

	unsigned	ngspins;
	struct gspin	*gspins;
	unsigned	ngpins;
	struct gpins	*gpins;
};

/* struct pic32_pinctrl_data - pic32 pinctrl descriptor
 * @dev: related device
 * @pctl: related pinctrl device
 * @ppsin_base: peripheral pin select input base address
 * @ppsout_base: peripheral pin select output base address
 * @pps_off: pps registers mapping
 * @clk: associated clock
 * @nbanks: number of banks (ports)
 * @npins: number pf pins
 * @ngroups: number of groups
 * @groups: pinctrl's groups (related to all functions)
 * @nfunctions: number of functions
 * @functions: pinctrl's functions
 **/
struct pic32_pinctrl_data {
	struct device			*dev;
	struct pinctrl_dev		*pctl;

	void __iomem			*ppsin_base;
	void __iomem			*ppsout_base;
	struct pic32_pps_off		*pps_off;
	struct clk			*clk;

	struct pic32_caps		caps;

	unsigned			nbanks;
	unsigned			npins;

	unsigned			ngroups;
	struct pin_group		*groups;

	unsigned			nfunctions;
	struct pin_function		*functions;
};

/* check pinconf capabilities and set the configuration */
static int pic32_pinconf_set(struct pic32_pinctrl_data *data,
			     struct pic32_gpio_chip *pic32_chip,
			     unsigned long conf, unsigned pin)
{
	struct pic32_caps *caps = &data->caps;

	/* check */
	PIC32_CHECK_PINCONF_CAPS(data->dev, conf, caps);

	/* set direction */
	pic32_pinconf_set_dir(pic32_chip, pin, conf);

	/* set configuration */
	switch (conf) {
	case PIC32_PIN_CONF_OD:
	case PIC32_PIN_CONF_OD_OUT:
		return pic32_pinconf_open_drain(pic32_chip, pin, 1);
	case PIC32_PIN_CONF_PU:
	case PIC32_PIN_CONF_PU_IN:
		return pic32_pinconf_pullup(pic32_chip, pin, 1);
	case PIC32_PIN_CONF_PD:
	case PIC32_PIN_CONF_PD_IN:
		return pic32_pinconf_pulldown(pic32_chip, pin, 1);
	case PIC32_PIN_CONF_AN:
	case PIC32_PIN_CONF_AN_IN:
		return pic32_pinconf_analog(pic32_chip, pin);
	case PIC32_PIN_CONF_DG:
	case PIC32_PIN_CONF_DG_IN:
	case PIC32_PIN_CONF_DG_OUT:
		return pic32_pinconf_dg(pic32_chip, pin);
	default:
		goto _out;
	}

_out:
	return 0;
}

/* is pin open-drain? */
static u32 pic32_pinconf_is_open_drain(struct pic32_gpio_chip *pic32_chip,
				       unsigned pin)
{
	struct pic32_reg __iomem *odc_reg = (struct pic32_reg __iomem *)
			pic32_pio_get_reg(pic32_chip, PIC32_ODC);
	u32 mask = BIT(pin);

	return readl(&odc_reg->val) & mask;
}

/* is pin pull-up? */
static u32 pic32_pinconf_is_pullup(struct pic32_gpio_chip *pic32_chip,
				   unsigned pin)
{
	struct pic32_reg __iomem *cnpu_reg = (struct pic32_reg __iomem *)
			pic32_pio_get_reg(pic32_chip, PIC32_CNPU);
	u32 mask = BIT(pin);

	return readl(&cnpu_reg->val) & mask;
}

/* is pin pull-down? */
static u32 pic32_pinconf_is_pulldown(struct pic32_gpio_chip *pic32_chip,
				     unsigned pin)
{
	struct pic32_reg __iomem *cnpd_reg = (struct pic32_reg __iomem *)
			pic32_pio_get_reg(pic32_chip, PIC32_CNPD);
	u32 mask = BIT(pin);

	return readl(&cnpd_reg->set) & mask;
}

/* is pin analog? */
static u32 pic32_pinconf_is_analog(struct pic32_gpio_chip *pic32_chip,
				   unsigned pin)
{
	struct pic32_reg __iomem *ansel_reg = (struct pic32_reg __iomem *)
				pic32_pio_get_reg(pic32_chip, PIC32_ANSEL);
	u32 mask = BIT(pin);

	return readl(&ansel_reg->set) & mask;
}

/* get pin's configuration */
static unsigned long pic32_pinconf_get(struct pic32_gpio_chip *pic32_chip,
				       unsigned pin)
{
	u32 pinconf = 0;

	pinconf |= pic32_pinconf_is_open_drain(pic32_chip, pin);
	pinconf |= pic32_pinconf_is_pullup(pic32_chip, pin);
	pinconf |= pic32_pinconf_is_pulldown(pic32_chip, pin);
	pinconf |= pic32_pinconf_is_analog(pic32_chip, pin);

	return (unsigned long)pinconf;
}

/* get the config of a certain pin */
static int pic32_pin_config_get(struct pinctrl_dev *pctldev,
				unsigned pin_id, unsigned long *config)
{
	struct pic32_pinctrl_data *data = pinctrl_dev_get_drvdata(pctldev);
	unsigned bank = pin_id / PINS_PER_BANK;
	unsigned pin = pin_id % PINS_PER_BANK;
	struct pic32_gpio_chip *pic32_chip = gpio_chips[bank];

	if (pin_id > PINS_PER_BANK * MAX_PIO_BANKS)
		BUG();

	*config = pic32_pinconf_get(pic32_chip, pin);

	dev_dbg(data->dev, "%s: get config:0x%lx of pin (%u,%u)\n", __func__,
							*config, bank, pin);

	return 0;
}

/* configure an individual pin */
static int pic32_pin_config_set(struct pinctrl_dev *pctldev,
				unsigned pin_id, unsigned long *configs,
				unsigned num_configs)
{
	struct pic32_pinctrl_data *data = pinctrl_dev_get_drvdata(pctldev);
	struct pic32_gpio_chip *pic32_chip;
	unsigned bank = pin_id / PINS_PER_BANK;
	unsigned pin = pin_id % PINS_PER_BANK;
	int i;

	if (pin > PINS_PER_BANK || bank >= MAX_PIO_BANKS)
		BUG();

	pic32_chip = gpio_chips[bank];

	for (i = 0; i < num_configs; i++) {
		unsigned long config = configs[i];

		if (config == PIC32_PIN_CONF_NONE)
			continue;

		dev_dbg(data->dev, "%s: set config:0x%lx to pin (%u,%u)\n",
						__func__, config, bank, pin);

		/* check caps and set a particular mux function */
		pic32_pinconf_set(data, pic32_chip, config, pin);
	}

	return 0;
}

#ifdef CONFIG_DEBUG_FS
/* debugfs: info for a certain pin */
static void pic32_pinconf_dbg_show(struct pinctrl_dev *pctldev,
				   struct seq_file *s, unsigned pin)
{
	struct pic32_pinctrl_data *data = pinctrl_dev_get_drvdata(pctldev);

	dev_dbg(data->dev, "%s: NULL\n", __func__);
}

/* debugfs: info for a certain pin group */
static void pic32_pinconf_group_dbg_show(struct pinctrl_dev *pctldev,
					 struct seq_file *s, unsigned group)
{
	struct pic32_pinctrl_data *data = pinctrl_dev_get_drvdata(pctldev);

	dev_dbg(data->dev, "%s: NULL\n", __func__);
}
#else
#define pic32_pinconf_dbg_show NULL
#define pic32_pinconf_group_dbg_show NULL
#endif

/* pin config operations */
static const struct pinconf_ops pic32_pinconf_ops = {
	.pin_config_get = pic32_pin_config_get,
	.pin_config_set = pic32_pin_config_set,
	.pin_config_dbg_show = pic32_pinconf_dbg_show,
	.pin_config_group_dbg_show = pic32_pinconf_group_dbg_show,
};

/* returns number of selectable functions */
static int pic32_pinmux_get_funcs_count(struct pinctrl_dev *pctldev)
{
	struct pic32_pinctrl_data *data = pinctrl_dev_get_drvdata(pctldev);

	dev_dbg(data->dev, "%s: nfunction=%u\n", __func__, data->nfunctions);

	return data->nfunctions;
}

/* return the function name */
static const char *pic32_pinmux_get_func_name(struct pinctrl_dev *pctldev,
					      unsigned selector)
{
	struct pic32_pinctrl_data *data = pinctrl_dev_get_drvdata(pctldev);

	dev_dbg(data->dev, "%s: get:%s\n", __func__,
					data->functions[selector].name);

	return data->functions[selector].name;
}

/* return an array of groups names */
static int pic32_pinmux_get_func_groups(struct pinctrl_dev *pctldev,
					unsigned selector,
					const char * const **groups,
					unsigned * const num_groups)
{
	struct pic32_pinctrl_data *data = pinctrl_dev_get_drvdata(pctldev);

	*groups = data->functions[selector].groups;
	*num_groups = data->functions[selector].ngroups;

	dev_dbg(data->dev, "%s: get groups(%u) of function=%u\n", __func__,
							*num_groups, selector);

	return 0;
}

/* get offset within PPS-OUT register for a particular re-mappable pin */
static unsigned pic32_get_ppsout_offset(struct pic32_pinctrl_data *data,
					unsigned bank, unsigned pin)
{
	struct pic32_pps_off *pps_off = data->pps_off;

	PIC32_CHECK_PPSOUT_OFF(bank, pin, (*pps_off->ppsout_lookup_off));

	return (*pps_off->ppsout_lookup_off)[bank][pin];
}

/* set a particular OUT mux function for a particular
 *   re-mappable pin (bank,pin) */
static void pic32_ppsout_set(struct pic32_pinctrl_data *data,
			     struct gpins *gpins,
			     bool en)
{
	void __iomem *base = data->ppsout_base;
	unsigned bank = gpins->bank;
	unsigned pin = gpins->pin;
	unsigned off = pic32_get_ppsout_offset(data, bank, pin);

	if (en)
		writel(gpins->cod, base + off);
	else
		writel(0x0, base + off);

	dev_dbg(data->dev, "%s: en=%d bank=%u pin=%u off=%u cod=0x%x\n",
				__func__, en, bank, pin, off, gpins->cod);
}

/* get offset within PPS-IN register for a particular peripheral pin */
static unsigned pic32_get_ppsin_offset(struct pic32_pinctrl_data *data,
				       unsigned ppin)
{
	struct pic32_pps_off *pps_off = data->pps_off;

	PIC32_CHECK_PPSIN_OFF(ppin, (*pps_off->ppsin_lookup_off));

	return (*pps_off->ppsin_lookup_off)[ppin];
}

/* set a particular IN mux function for a particular
 *   peripheral pin (bank,pin) */
static void pic32_ppsin_set(struct pic32_pinctrl_data *data,
			    struct gpins *gpins, bool en)
{
	unsigned ppin = gpins->ppin;
	unsigned off = pic32_get_ppsin_offset(data, ppin);
	void __iomem *base = data->ppsin_base;

	if (en)
		writel(gpins->cod, base + off);
	else
		writel(0x0, base + off);

	dev_dbg(data->dev, "%s: en=%d ppin=%u off=%u cod=0x%x\n",
				__func__, en, ppin, off, gpins->cod);
}

/* set a particular pinmux function */
static int pic32_pinmux_muxen(struct pic32_pinctrl_data *data,
			      struct gpins *gpins,
			      bool en)
{
	if (gpins->dir == DIR_IN)
		pic32_ppsin_set(data, gpins, en);
	else if (gpins->dir == DIR_OUT)
		pic32_ppsout_set(data, gpins, en);
	else
		return -EINVAL;

	return 0;
}

/* enable a certain pinmux function within a pin group and
 *   within a muxing function */
static int pic32_pinmux_set_mux(struct pinctrl_dev *pctldev,
			       unsigned selector, unsigned group)
{
	struct pic32_pinctrl_data *data = pinctrl_dev_get_drvdata(pctldev);
	struct pin_group *grp = &data->groups[group];
	unsigned ngpins = grp->ngpins;
	struct gpins *gpins = data->groups[group].gpins;
	int i;

	dev_dbg(data->dev, "enable function %s for group %s\n",
			data->functions[selector].name, grp->name);

	for (i = 0; i < ngpins; i++) {
		struct gpins *pins = &gpins[i];

		pic32_pinmux_muxen(data, pins, true);
	}

	return 0;
}

/* enable a pin to work in GPIO mode */
static int pic32_gpio_request_enable(struct pinctrl_dev *pctldev,
				     struct pinctrl_gpio_range *range,
				     unsigned offset)
{
	struct pic32_pinctrl_data *data = pinctrl_dev_get_drvdata(pctldev);
	struct gpio_chip *chip = range->gc;
	struct pic32_gpio_chip *pic32_chip = to_pic32_gpio_chip(chip);
	struct pic32_reg __iomem *ansel_reg = (struct pic32_reg __iomem *)
				pic32_pio_get_reg(pic32_chip, PIC32_ANSEL);
	int pin = offset - chip->base;
	u32 mask = BIT(pin);

	if (!range) {
		dev_err(data->dev, "invalid range\n");
		return -EINVAL;
	}
	if (!range->gc) {
		dev_err(data->dev, "missing GPIO chip in range\n");
		return -EINVAL;
	}

	/* clear analog selection when digital input is required! */
	writel(mask, &ansel_reg->clr);

	dev_dbg(data->dev, "%s: enable pin %u as GPIO-%c:%d\n", __func__,
			offset, 'A' + range->id, pin);

	return 0;
}

/* disable a pin from GPIO mode */
static void pic32_gpio_disable_free(struct pinctrl_dev *pctldev,
				    struct pinctrl_gpio_range *range,
				    unsigned offset)
{
	struct pic32_pinctrl_data *data = pinctrl_dev_get_drvdata(pctldev);
	struct gpio_chip *chip = range->gc;

	if (!range) {
		dev_err(data->dev, "invalid range\n");
		return;
	}
	if (!range->gc) {
		dev_err(data->dev, "missing GPIO chip in range\n");
		return;
	}

	dev_dbg(data->dev, "%s: free pin %u as GPIO-%c:%d\n", __func__,
			offset, 'A' + range->id, offset - chip->base);

	return;
}

/* pinmux operations */
static const struct pinmux_ops pic32_pinmux_ops = {
	.get_functions_count = pic32_pinmux_get_funcs_count,
	.get_function_name = pic32_pinmux_get_func_name,
	.get_function_groups = pic32_pinmux_get_func_groups,
	.set_mux = pic32_pinmux_set_mux,
	.gpio_request_enable = pic32_gpio_request_enable,
	.gpio_disable_free = pic32_gpio_disable_free,
};

static const struct pin_group *pic32_pinctrl_find_group_by_name(
				const struct pic32_pinctrl_data *data,
				const char *name)
{
	const struct pin_group *grp = NULL;
	int i;

	for (i = 0; i < data->ngroups; i++) {
		if (strcmp(data->groups[i].name, name))
			continue;

		grp = &data->groups[i];
		dev_dbg(data->dev, "%s: found group:%s\n", __func__, name);
		break;
	}

	return grp;
}

static int pic32_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct pic32_pinctrl_data *data = pinctrl_dev_get_drvdata(pctldev);

	dev_dbg(data->dev, "%s: ngroups:%u\n", __func__, data->ngroups);

	return data->ngroups;
}

static const char *pic32_get_group_name(struct pinctrl_dev *pctldev,
					unsigned selector)
{
	struct pic32_pinctrl_data *data = pinctrl_dev_get_drvdata(pctldev);

	dev_dbg(data->dev, "%s: selector=%u group=%s\n", __func__,
					selector, data->groups[selector].name);

	return data->groups[selector].name;
}

static int pic32_get_group_pins(struct pinctrl_dev *pctldev, unsigned selector,
				const unsigned **pins, unsigned *npins)
{
	struct pic32_pinctrl_data *data = pinctrl_dev_get_drvdata(pctldev);

	*pins = data->groups[selector].pins;
	*npins = data->groups[selector].npins;

	dev_dbg(data->dev, "%s: get pins(%u) for group=%u\n", __func__,
							*npins, selector);

	return 0;
}

static void pic32_pin_dbg_show(struct pinctrl_dev *pctldev,
			       struct seq_file *s, unsigned offset)
{
	struct pic32_pinctrl_data *data = pinctrl_dev_get_drvdata(pctldev);

	seq_printf(s, "%s", dev_name(data->dev));
}

/* construct a config map for pins(single and pinmux) within a group */
static int pic32_dt_node_to_map(struct pinctrl_dev *pctldev,
				struct device_node *np,
				struct pinctrl_map **map,
				unsigned *num_maps)
{
	struct pic32_pinctrl_data *data = pinctrl_dev_get_drvdata(pctldev);
	const struct pin_group *grp;
	struct pinctrl_map *new_map;
	struct device_node *parent;
	int map_num = 1;
	int i;

	/* first find the group of this node and check if we need create
	 *   config maps for pins */
	grp = pic32_pinctrl_find_group_by_name(data, np->name);
	if (!grp) {
		dev_err(data->dev, "unable to find group for node %s\n",
			np->name);
		return -EINVAL;
	}

	map_num += (grp->ngspins + grp->ngpins);
	new_map = devm_kzalloc(pctldev->dev,
				sizeof(*new_map) * map_num, GFP_KERNEL);
	if (!new_map)
		return -ENOMEM;

	*map = new_map;
	*num_maps = map_num;

	/* create mux map */
	parent = of_get_parent(np);
	if (!parent) {
		devm_kfree(pctldev->dev, new_map);
		return -EINVAL;
	}
	new_map[0].type = PIN_MAP_TYPE_MUX_GROUP;
	new_map[0].data.mux.function = parent->name;
	new_map[0].data.mux.group = np->name;
	of_node_put(parent);

	/* single-pins */
	new_map++;
	for (i = 0; i < grp->ngspins; i++) {
		unsigned pin = grp->gspins[i].bank * PINS_PER_BANK +
							grp->gspins[i].pin;

		new_map[i].type = PIN_MAP_TYPE_CONFIGS_PIN;
		new_map[i].data.configs.group_or_pin =
				pin_get_name(pctldev, pin);
		new_map[i].data.configs.configs = &grp->gspins[i].conf;
		new_map[i].data.configs.num_configs = 1;

	}

	/* pin-mux */
	for (i = 0; i < grp->ngpins; i++) {
		unsigned pin = grp->gpins[i].bank * PINS_PER_BANK +
							grp->gpins[i].pin;

		new_map[grp->ngspins + i].type = PIN_MAP_TYPE_CONFIGS_PIN;
		new_map[grp->ngspins + i].data.configs.group_or_pin =
				pin_get_name(pctldev, pin);
		new_map[grp->ngspins + i].data.configs.configs =
				&grp->gpins[i].conf;
		new_map[grp->ngspins + i].data.configs.num_configs = 1;

	}

	dev_dbg(pctldev->dev, "%s: maps: function:%s group:%s num:%d\n",
	__func__, (*map)->data.mux.function, (*map)->data.mux.group, map_num);

	return 0;
}

static void pic32_dt_free_map(struct pinctrl_dev *pctldev,
			      struct pinctrl_map *map, unsigned num_maps)
{
	struct pic32_pinctrl_data *data = pinctrl_dev_get_drvdata(pctldev);

	dev_dbg(data->dev, "%s: free map=%p\n", __func__, map);

	devm_kfree(pctldev->dev, map);
}

static const struct pinctrl_ops pic32_pinctrl_ops = {
	.get_groups_count = pic32_get_groups_count,
	.get_group_name = pic32_get_group_name,
	.get_group_pins = pic32_get_group_pins,
	.pin_dbg_show = pic32_pin_dbg_show,
	.dt_node_to_map = pic32_dt_node_to_map,
	.dt_free_map = pic32_dt_free_map,
};

static struct pinctrl_desc pic32_pinctrl_desc = {
	.pctlops = &pic32_pinctrl_ops,
	.pmxops = &pic32_pinmux_ops,
	.confops = &pic32_pinconf_ops,
	.owner = THIS_MODULE,
};

/* pinctrl mechanism force to keep a pin array, therfore the pin array will
 *   contain single-pinss and pinmux as well. */
static int pic32_pinctrl_cout_pins(struct platform_device *pdev,
				   struct pin_group *grp)
{
	int i;

	grp->npins = grp->ngspins + grp->ngpins;
	grp->pins = devm_kzalloc(&pdev->dev, grp->npins * sizeof(unsigned int),
				 GFP_KERNEL);
	if (!grp->pins)
		return -ENOMEM;

	/* the single-pinss */
	for (i = 0; i < grp->ngspins; i++) {
		grp->pins[i] =
			grp->gspins[i].bank * PINS_PER_BANK +
							grp->gspins[i].pin;
	}

	/* the pinmux */
	for (i = 0; i < grp->ngpins; i++) {
		grp->pins[grp->ngspins + i] =
			grp->gpins[i].bank * PINS_PER_BANK +
							grp->gpins[i].pin;
	}

	return 0;
}

/* return the idx of a property within the array of properties */
static int get_pic32_pinctrl_propidx(struct property *prop)
{
	int i;

	for (i = 0; i < PIC32_PINCTRL_PROP_LAST; i++) {
		if (0 == strcmp(pinctrl_props[i].name, prop->name))
			return i;
	}

	return -EINVAL;
}

/* parse single-pins entry within a group */
static int pic32_pinctrl_parse_single_pins(struct platform_device *pdev,
					   struct device_node *np,
					   struct pin_group *grp,
					   struct pic32_pinctrl_data *data,
					   int idx)
{
	struct property *prop;
	int length;
	int u32array_size;
	u32 *u32array;
	int i;
	int ret = 0;

	/* sanity check */
	prop = of_find_property(np, pinctrl_props[idx].name, &length);
	if (!prop) {
		ret = -EINVAL;
		goto out_err;
	}

	/* get single-pins entries */
	u32array_size = length/sizeof(u32);
	if (u32array_size % pinctrl_props[idx].narg) {
		ret = -EINVAL;
		goto out_err;
	}

	u32array = kzalloc(u32array_size*sizeof(u32), GFP_KERNEL);
	if (!u32array) {
		ret = -ENOMEM;
		goto out_err;
	}

	ret = of_property_read_u32_array(np, pinctrl_props[idx].name,
						u32array, u32array_size);
	if (ret)
		goto clean_out_err;

	grp->ngspins = u32array_size/pinctrl_props[idx].narg;
	grp->gspins = devm_kzalloc(&pdev->dev,
				grp->ngspins * sizeof(struct gspin),
				GFP_KERNEL);

	dev_dbg(data->dev, "%s: parse %u pins\n", __func__,
							grp->ngspins);
	/* single-pins configuration entry:
	 *   <pin-bank> <pin-idx> <pin-conf> */
	for (i = 0; i < grp->ngspins; i++) {
		u32 *bank = (i == 0 ? u32array : (++u32array));
		u32 *pin = (++u32array);
		struct pin_conf *pinconf = (struct pin_conf *)(++u32array);

		grp->gspins[i].bank = *bank;
		grp->gspins[i].pin = *pin;

		/* we need the direction for differentiate between
		 *   PPS-in and PPS-out */
		grp->gspins[i].conf = (CONF_DIR(pinconf->dir) |
				       CONF_COD(pinconf->conf));
	}

	return 0;
clean_out_err:
	kfree(u32array);
out_err:
	dev_err(data->dev, "wrong single-pins entry\n");
	return ret;
}

/* parse pinmux entry within a group */
static int pic32_pinctrl_parse_pins(struct platform_device *pdev,
				    struct device_node *np,
				    struct pin_group *grp,
				    struct pic32_pinctrl_data *data,
				    int idx)
{
	struct property *prop;
	int length;
	int u32array_size;
	u32 *u32array;
	int i;
	int ret = 0;

	/* sanity check */
	prop = of_find_property(np, pinctrl_props[idx].name, &length);
	if (!prop) {
		ret = -EINVAL;
		goto out_err;
	}

	u32array_size = length/sizeof(u32);
	if (u32array_size % pinctrl_props[idx].narg) {
		ret = -EINVAL;
		goto out_err;
	}


	/* get pinmux entries */
	u32array = kzalloc(u32array_size*sizeof(u32), GFP_KERNEL);
	if (!u32array) {
		ret = -ENOMEM;
		goto out_err;
	}

	ret = of_property_read_u32_array(np, pinctrl_props[idx].name,
						u32array, u32array_size);
	if (ret)
		goto out_clean_err;

	grp->ngpins = u32array_size/pinctrl_props[idx].narg;
	grp->gpins = devm_kzalloc(&pdev->dev,
				grp->ngpins * sizeof(struct gpins),
				GFP_KERNEL);

	dev_dbg(data->dev, "%s: parse %u pins\n", __func__,
							grp->ngpins);
	/* pinmux configuration entry:
	 *   <pin-code> <peripheral-pin-code> <pin-conf> */
	for (i = 0; i < grp->ngpins; i++) {
		struct rpin_cod *pincod = (struct rpin_cod *)
					  (i == 0 ? u32array : (++u32array));
		struct ppin_cod *ppincod = (struct ppin_cod *)(++u32array);
		struct pin_conf *pinconf = (struct pin_conf *)(++u32array);

		dev_dbg(data->dev, "rpin(%x:%x:%x:%x:%x)\n",
			pincod->bank, pincod->pin, pincod->bucket,
			pincod->dir, pincod->cod);
		dev_dbg(data->dev, "ppin(%x:%x:%x:%x)\n",
			ppincod->pin, ppincod->bucket,
			ppincod->dir, ppincod->cod);
		dev_dbg(data->dev, "rpin-conf(%x:%x)\n",
			pinconf->dir, pinconf->conf);

		/* syntax check:
		 * 1) if the function is not available for bucket or
		 * 2) if the function is not availbale for direction */
		if (((pincod->bucket & ppincod->bucket) == 0) ||
		    (pincod->dir != ppincod->dir)) {
			ret = -EINVAL;
			goto out_clean_err;
		}

		grp->gpins[i].dir = pincod->dir;
		grp->gpins[i].bucket = pincod->bucket;
		grp->gpins[i].bank = pincod->bank;
		grp->gpins[i].pin = pincod->pin;
		grp->gpins[i].ppin = ppincod->pin;
		if (grp->gpins[i].dir == DIR_OUT)
			grp->gpins[i].cod = ppincod->cod;
		else
			grp->gpins[i].cod = pincod->cod;

		grp->gpins[i].conf = (CONF_DIR(pinconf->dir) |
				      CONF_COD(pinconf->conf));
	}

	return 0;
out_clean_err:
	kfree(u32array);
out_err:
	dev_err(data->dev, "wrong pins entry\n");
	return ret;

}

/* parse group's properties */
static int pic32_pinctrl_parse_group_prop(struct platform_device *pdev,
					  struct device_node *np,
					  struct pin_group *grp,
					  struct pic32_pinctrl_data *data,
					  int idx)
{
	switch (idx) {
	case PIC32_PINCTRL_PROP_SINGLE_PINS:
		return pic32_pinctrl_parse_single_pins(pdev, np, grp,
						       data, idx);
	break;
	case PIC32_PINCTRL_PROP_PINS:
		return pic32_pinctrl_parse_pins(pdev, np, grp,
						data, idx);
	break;
	default:
		return -EINVAL;
	}

	return -EINVAL;
}

/* parse groups within a function */
static int pic32_pinctrl_parse_groups(struct platform_device *pdev,
				      struct device_node *np,
				      struct pin_group *grp,
				      struct pic32_pinctrl_data *data,
				      u32 gidx)
{
	struct property *prop;
	int ret = 0;

	dev_dbg(data->dev, "%s: group(%d): %s\n", __func__, gidx, np->name);

	grp->name = np->name;

	/* parse group's properties */
	for_each_property_of_node(np, prop) {
		int prop_idx;

		if (!strcmp(prop->name, "name") ||
		    !strcmp(prop->name, "phandle") ||
		    !strcmp(prop->name, "linux,phandle"))
			continue;

		prop_idx = get_pic32_pinctrl_propidx(prop);
		if (prop_idx == -EINVAL) {
			dev_err(data->dev, "property (%s) not found!\n",
								prop->name);
			return -EINVAL;
		}
		dev_dbg(data->dev, "property found: %s\n",
						pinctrl_props[prop_idx].name);

		ret = pic32_pinctrl_parse_group_prop(pdev, np, grp,
						     data, prop_idx);
		if (ret) {
			dev_err(data->dev, "wrong property (%s)!\n",
								prop->name);
			return ret;
		}
	}

	return pic32_pinctrl_cout_pins(pdev, grp);
}

/* parse pinctrl's functions */
static int pic32_pinctrl_parse_functions(struct platform_device *pdev,
					 struct device_node *np,
					 struct pic32_pinctrl_data *data,
					 u32 fidx)
{
	struct device_node *child;
	struct pin_function *func;
	struct pin_group *grp;
	static unsigned gidx; /* group index index related to pinctrl */
	unsigned f_gidx = 0;  /* group index related to function */
	int ret;

	dev_dbg(data->dev, "%s: parse function(%d): %s\n", __func__,
							fidx, np->name);

	func = &data->functions[fidx];

	/* initialise function */
	func->name = np->name;
	func->ngroups = of_get_child_count(np);
	if (func->ngroups <= 0) {
		dev_err(data->dev, "no groups defined\n");
		return -EINVAL;
	}
	func->groups = devm_kzalloc(data->dev,
			func->ngroups * sizeof(char *), GFP_KERNEL);
	if (!func->groups)
		return -ENOMEM;

	/* parse groups within this function */
	f_gidx = 0;
	for_each_child_of_node(np, child) {
		func->groups[f_gidx] = child->name;
		grp = &data->groups[gidx++];
		ret = pic32_pinctrl_parse_groups(pdev, child,
						 grp, data, f_gidx++);
		if (ret)
			return ret;
	}

	return 0;
}

/* probe(parse) pinctrl's device tree node */
static int pic32_pinctrl_probe_dt(struct platform_device *pdev,
				  struct pic32_pinctrl_data *data)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *child;
	const char *pio_compat = "microchip,pic32-gpio";
	struct pinctrl_pin_desc *pdesc;
	int f_idx = 0;
	int i, kbank, kpio;
	int ret = 0;

	child = of_get_next_child(np, NULL);
	if (!child) {
		dev_err(&pdev->dev, "no group is defined\n");
		return -ENOENT;
	}

	/* count total functions and groups */
	for_each_child_of_node(np, child) {
		if (of_device_is_compatible(child, pio_compat)) {
			data->nbanks++;
		} else {
			data->nfunctions++;
			data->ngroups += of_get_child_count(child);
		}
	}

	if (data->nbanks < 1 || data->nbanks > MAX_PIO_BANKS) {
		dev_err(&pdev->dev,
			"gpio-controllers between 1 to %u.\n", MAX_PIO_BANKS);
		return -EINVAL;
	}
	data->npins = data->nbanks * PINS_PER_BANK;

	dev_dbg(&pdev->dev, "%s:(nbanks:%u, nfunctions:%u, ngroups:%u)\n",
		__func__, data->nbanks, data->nfunctions, data->ngroups);

	data->functions = devm_kzalloc(&pdev->dev, data->nfunctions *
				   sizeof(*data->functions), GFP_KERNEL);
	if (!data->functions)
		return -ENOMEM;

	data->groups = devm_kzalloc(&pdev->dev, data->ngroups *
				   sizeof(*data->groups), GFP_KERNEL);
	if (!data->groups)
		return -ENOMEM;

	/* parse all functions */
	f_idx = 0;
	for_each_child_of_node(np, child) {
		if (of_device_is_compatible(child, pio_compat))
			continue;

		ret = pic32_pinctrl_parse_functions(pdev, child,
						      data, f_idx++);
		if (ret) {
			dev_err(&pdev->dev, "failed to parse function\n");
			return ret;
		}
	}

	pic32_pinctrl_desc.name = dev_name(&pdev->dev);
	pic32_pinctrl_desc.npins = data->npins;
	pic32_pinctrl_desc.pins = pdesc = devm_kzalloc(&pdev->dev,
						sizeof(*pdesc) * data->npins,
						GFP_KERNEL);
	if (!pic32_pinctrl_desc.pins)
		return -ENOMEM;

	/* populate pinctrl pin descriptor */
	kbank = 0;
	for_each_child_of_node(np, child) {
		if (of_device_is_compatible(child, pio_compat)) {
			kpio = 0;
			for (i = 0; i < PINS_PER_BANK; i++) {
				pdesc->number = (kbank*PINS_PER_BANK) + kpio;
				pdesc->name = kasprintf(GFP_KERNEL, "%s%c%d",
						child->name,
						kbank + 'A',
						kpio);
				pdesc->drv_data = NULL; /* unused */

				pdesc++;
				kpio++;
			}
			kbank++;
		}
	}

	return 0;
}

/* probe pinctrl device */
int pic32_pinctrl_probe(struct platform_device *pdev,
			struct pic32_pps_off *pps_off,
			struct pic32_caps *caps)
{
	struct device_node *np = pdev->dev.of_node;
	struct pic32_pinctrl_data *data = NULL;
	struct resource *r;
	int i;
	int ret = 0;

	dev_dbg(&pdev->dev, "%s: probing...\n", __func__);

	if (!np)
		return -ENODEV;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->dev = &pdev->dev;
	data->caps.pinconf_incaps = caps->pinconf_incaps;
	data->caps.pinconf_outcaps = caps->pinconf_outcaps;

	/* base address of pps(in, out) registers */
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		ret = -EINVAL;
		goto probe_err;
	}
	data->ppsin_base = devm_ioremap_nocache(&pdev->dev, r->start,
							resource_size(r));
	if (IS_ERR(data->ppsin_base)) {
		ret = PTR_ERR(data->ppsin_base);
		goto probe_err;
	}

	r = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!r) {
		ret = -EINVAL;
		goto probe_err;
	}
	data->ppsout_base = devm_ioremap_nocache(&pdev->dev, r->start,
							resource_size(r));
	if (IS_ERR(data->ppsout_base)) {
		ret = PTR_ERR(data->ppsout_base);
		goto probe_err;
	}
	data->pps_off = pps_off;

	/* clocks */
	data->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(data->clk)) {
		ret = PTR_ERR(data->clk);
		dev_err(&pdev->dev, "clk get failed\n");
		goto probe_err;
	}

	ret = clk_prepare_enable(data->clk);
	if (ret) {
		dev_err(&pdev->dev, "clk enable failed\n");
		goto probe_err;
	}

	/* probe(parse) pinctrl device tree node */
	ret = pic32_pinctrl_probe_dt(pdev, data);
	if (ret) {
		dev_err(&pdev->dev, "dt probe failed: %d\n", ret);
		goto probe_err;
	}

	/* PIO driver must be probed first */
	for (i = 0; i < data->nbanks; i++) {
		if (!gpio_chips[i]) {
			dev_warn(&pdev->dev,
				"GPIO chip %d not registered\n", i);
			ret = -EPROBE_DEFER;
			goto probe_defer;
		}
	}

	platform_set_drvdata(pdev, data);
	data->pctl = pinctrl_register(&pic32_pinctrl_desc, &pdev->dev, data);
	if (!data->pctl) {
		dev_err(&pdev->dev, "Couldn't register pic32 pinctrl driver\n");
		ret = -EINVAL;
		goto probe_err;
	}

	/* Add gpio pin ranges - defined by GPIO nodes */
	for (i = 0; i < data->nbanks; i++)
		pinctrl_add_gpio_range(data->pctl, &gpio_chips[i]->range);

	dev_info(&pdev->dev, "pic32 pinctrl driver initialized.\n");
	return 0;

probe_defer:
probe_err:
	if (data->ppsin_base)
		devm_iounmap(&pdev->dev, data->ppsin_base);
	if (data->ppsout_base)
		devm_iounmap(&pdev->dev, data->ppsout_base);
	if (data)
		devm_kfree(&pdev->dev, data);
	return ret;
}
