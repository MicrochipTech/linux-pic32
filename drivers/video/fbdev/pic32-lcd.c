/*
 * PIC32 LCD Framebuffer Driver
 *
 * Joshua Henderson, <joshua.henderson@microchip.com>
 * Copyright (C) 2015 Microchip Technology Inc.  All rights reserved.
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <video/of_display_timing.h>
#include <linux/dma-mapping.h>
#include <video/videomode.h>
#include <linux/clk.h>
#include <asm/mach-pic32/pic32.h>
#include <linux/platform_data/lcd-pic32.h>

#include "pic32-lcd.h"

#define PIC32_LCD_PALETTE_OFFSET	0x400
#define PIC32_LCD_PALETTE_COLORS	256
#define PIC32_LCD_ACCEL			0x54736930

#define pic32_readl(base, offset)	__raw_readl((base) + (offset))
#define pic32_writel(base, offset, val) __raw_writel((val), (base) + (offset))

#define XY16TOREG32(x, y)	((x) << 16 | ((y) & 0xffff))
#define CLAMP255(i)		(((i) < 0) ? 0 : ((i) > 255) ? 255 : (i))

/* LCD Controller info data structure, stored in device platform_data */
struct pic32_lcd_info {
	u32			pseudo_palette[PIC32_LCD_PALETTE_COLORS];
	u32			mode;
	wait_queue_head_t	wait_vsync;
	u64			vblank_count;
	struct fb_info		*info;
	void __iomem		*mmio;
	int			irq;
	unsigned int		smem_len;
	struct platform_device	*pdev;
	struct pic32_lcd_pdata	*pdata;
	struct clk		*bus_clk;
	struct clk		*lcdc_clk;
};

static const struct fb_videomode *pic32_lcd_choose_mode(
	struct fb_var_screeninfo *var,
	struct fb_info *info)
{
	struct fb_videomode varfbmode;
	const struct fb_videomode *fbmode = NULL;

	fb_var_to_videomode(&varfbmode, var);
	fbmode = fb_find_nearest_mode(&varfbmode, &info->modelist);
	if (fbmode)
		fb_videomode_to_var(var, fbmode);
	return fbmode;
}

static int pic32_lcd_check_var(struct fb_var_screeninfo *var,
			       struct fb_info *info)
{
	struct device *dev = info->device;
	struct pic32_lcd_info *sinfo = info->par;
	unsigned long clk_value_khz;

	clk_value_khz = clk_get_rate(sinfo->lcdc_clk) / 1000;

	if (!(var->pixclock && var->bits_per_pixel)) {
		/* choose a suitable mode if possible */
		if (!pic32_lcd_choose_mode(var, info)) {
			dev_err(dev, "needed value not specified\n");
			return -EINVAL;
		}
	}

	dev_dbg(dev, "  resolution: %ux%u\n", var->xres, var->yres);
	dev_dbg(dev, "  pixclk:     %lu KHz\n", PICOS2KHZ(var->pixclock));
	dev_dbg(dev, "  bpp:        %u\n", var->bits_per_pixel);
	dev_dbg(dev, "  clk:        %lu KHz\n", clk_value_khz);

	if (PICOS2KHZ(var->pixclock) > clk_value_khz) {
		dev_err(dev, "%lu KHz pixel clock is too fast\n",
			PICOS2KHZ(var->pixclock));
		return -EINVAL;
	}

	/*  FB_VMODE_CONUPDATE and FB_VMODE_SMOOTH_XPAN are equal!
	 *  as FB_VMODE_SMOOTH_XPAN is only used internally */
	if (var->vmode & FB_VMODE_CONUPDATE) {
		var->vmode |= FB_VMODE_YWRAP;
		var->xoffset = info->var.xoffset;
		var->yoffset = info->var.yoffset;
	}

	/* some very basic checks */
	if (!var->xres)
		var->xres = 1;
	if (!var->yres)
		var->yres = 1;
	if (var->xres > var->xres_virtual)
		var->xres_virtual = var->xres;
	if (var->yres > var->yres_virtual)
		var->yres_virtual = var->yres;

	if (var->bits_per_pixel <= 8)
		var->bits_per_pixel = 8;
	else if (var->bits_per_pixel <= 16)
		var->bits_per_pixel = 16;
	else if (var->bits_per_pixel <= 24)
		var->bits_per_pixel = 24;
	else if (var->bits_per_pixel <= 32)
		var->bits_per_pixel = 32;
	else
		return -EINVAL;

	if (var->xres_virtual < var->xoffset + var->xres)
		var->xres_virtual = var->xoffset + var->xres;
	if (var->yres_virtual < var->yoffset + var->yres)
		var->yres_virtual = var->yoffset + var->yres;

	if (info->fix.smem_len) {
		unsigned int smem_len = (var->xres_virtual * var->yres_virtual
					 * ((var->bits_per_pixel + 7) / 8));
		if (smem_len > info->fix.smem_len) {
			dev_err(dev,
				"Framebuffer too small (%u) need at least %u\n",
				info->fix.smem_len, smem_len);
			return -EINVAL;
		}
	}

	switch (var->bits_per_pixel) {
	case 1:
	case 2:
	case 4:
	case 8:
		if (var->grayscale || var->red.offset == 0) {
			/* LUT8 && L8 */
			var->red    = (struct fb_bitfield) { 0, 8, 0 };
			var->green  = (struct fb_bitfield) { 0, 8, 0 };
			var->blue   = (struct fb_bitfield) { 0, 8, 0 };
			var->transp = (struct fb_bitfield) { 0, 0, 0 };
		} else {
			/* RGB332 */
			var->red    = (struct fb_bitfield) { 5, 3, 0 };
			var->green  = (struct fb_bitfield) { 2, 3, 0 };
			var->blue   = (struct fb_bitfield) { 0, 2, 0 };
			var->transp = (struct fb_bitfield) { 0, 0, 0 };
		}

		break;
	case 16:
		if (var->transp.length) {
			/* RGBA5551 */
			var->red    = (struct fb_bitfield) { 11, 5, 0 };
			var->green  = (struct fb_bitfield) {  6, 5, 0 };
			var->blue   = (struct fb_bitfield) {  1, 5, 0 };
			var->transp = (struct fb_bitfield) {  0, 1, 0 };
		} else {
			/* RGB565 */
			var->red    = (struct fb_bitfield) { 11, 5, 0 };
			var->green  = (struct fb_bitfield) {  5, 6, 0 };
			var->blue   = (struct fb_bitfield) {  0, 5, 0 };
			var->transp = (struct fb_bitfield) {  0, 0, 0 };
		}

		break;
	case 24:
		/* RGB888 */
		var->red    = (struct fb_bitfield) { 16, 8, 0 };
		var->green  = (struct fb_bitfield) {  8, 8, 0 };
		var->blue   = (struct fb_bitfield) {  0, 8, 0 };
		var->transp = (struct fb_bitfield) {  0, 0, 0 };

		break;
	case 32:
		if (var->transp.offset || var->red.offset == 16) {
			/* ARGB8888 */
			var->transp = (struct fb_bitfield) { 24, 8, 0 };
			var->red    = (struct fb_bitfield) { 16, 8, 0 };
			var->green  = (struct fb_bitfield) {  8, 8, 0 };
			var->blue   = (struct fb_bitfield) {  0, 8, 0 };
		} else {
			/* RGBA8888 */
			var->red    = (struct fb_bitfield) { 24, 8, 0 };
			var->green  = (struct fb_bitfield) { 16, 8, 0 };
			var->blue   = (struct fb_bitfield) {  8, 8, 0 };
			var->transp = (struct fb_bitfield) {  0, 8, 0 };
		}

		break;
	default:
		dev_err(dev, "color depth %d not supported\n",
			var->bits_per_pixel);
		return -EINVAL;
	}

	return 0;
}

static int pic32_lcd_set_par(struct fb_info *info)
{
	struct pic32_lcd_info *sinfo = info->par;
	unsigned long value;
	unsigned long clk_value_khz;
	u32 mode_reg;
	u32 clkcon_reg;
	unsigned long pix_factor = 2;

	if (!sinfo)
		return -EINVAL;

	dev_dbg(info->device, "%s:\n", __func__);
	dev_dbg(info->device, "  * resolution: %ux%u (%ux%u virtual)\n",
		info->var.xres, info->var.yres,
		info->var.xres_virtual, info->var.yres_virtual);

	mode_reg = pic32_readl(sinfo->mmio, PIC32_LCD_REG_MODE);
	pic32_writel(sinfo->mmio, PIC32_LCD_REG_MODE, mode_reg &
		     ~PIC32_LCD_CONFIG_ENABLE);

	pic32_writel(sinfo->mmio, PIC32_LCD_REG_LAYER0_RESXY,
		     XY16TOREG32(info->var.xres, info->var.yres));
	pic32_writel(sinfo->mmio, PIC32_LCD_REG_RESXY,
		     XY16TOREG32(info->var.xres, info->var.yres));
	pic32_writel(sinfo->mmio, PIC32_LCD_REG_LAYER0_STARTXY,
		     XY16TOREG32(0, 0));
	pic32_writel(sinfo->mmio, PIC32_LCD_REG_LAYER0_SIZEXY,
		     XY16TOREG32(info->var.xres, info->var.yres));

	pic32_writel(sinfo->mmio, PIC32_LCD_REG_FRONTPORCHXY,
		     XY16TOREG32(info->var.xres + info->var.right_margin,
				 info->var.yres + info->var.lower_margin));
	pic32_writel(sinfo->mmio, PIC32_LCD_REG_BLANKINGXY,
		     XY16TOREG32(info->var.xres + info->var.right_margin +
				 info->var.hsync_len,
				 info->var.yres + info->var.lower_margin +
				 info->var.vsync_len));
	pic32_writel(sinfo->mmio, PIC32_LCD_REG_BACKPORCHXY,
		     XY16TOREG32(info->var.xres + info->var.right_margin +
				 info->var.hsync_len + info->var.left_margin,
				 info->var.yres + info->var.lower_margin +
				 info->var.vsync_len + info->var.upper_margin));

	if (info->var.bits_per_pixel <= 8)
		info->fix.visual = info->var.red.offset ?
			FB_VISUAL_TRUECOLOR : FB_VISUAL_PSEUDOCOLOR;
	else
		info->fix.visual = FB_VISUAL_TRUECOLOR;

	/* try to guess our mode based on color sizes and offsets */
	switch (info->var.bits_per_pixel) {
	case  8:
		sinfo->mode = info->var.grayscale ? PIC32_LCD_MODE_L8 :
		(info->var.red.offset ?
			PIC32_LCD_MODE_RGB332 : PIC32_LCD_MODE_LUT8);
		info->fix.line_length = info->var.xres_virtual;
		break;
	case 16:
		sinfo->mode = info->var.transp.length ?
			PIC32_LCD_MODE_RGBA5551 : PIC32_LCD_MODE_RGB565;
		info->fix.line_length = info->var.xres_virtual << 1;
		break;
	case 24:
		sinfo->mode = PIC32_LCD_MODE_RGB888;
		info->fix.line_length = info->var.xres_virtual * 3;
		break;
	case 32:
		sinfo->mode = (info->var.transp.length ||
			info->var.red.offset == 16) ?
			PIC32_LCD_MODE_ARGB8888 : PIC32_LCD_MODE_RGBA8888;
		info->fix.line_length = info->var.xres_virtual << 2;
		break;
	default:
		return -EINVAL;
	}

	switch (info->var.bits_per_pixel) {
	case 8:
	case 16:
		if (sinfo->pdata && sinfo->pdata->set_mode)
			sinfo->pdata->set_mode(1);
		break;
	case 24:
	case 32:
		if (sinfo->pdata && sinfo->pdata->set_mode)
			sinfo->pdata->set_mode(0);
		break;
	}

	dev_dbg(info->device, "Detected color mode is %u\n", sinfo->mode);

	clk_value_khz = clk_get_rate(sinfo->lcdc_clk) / 1000;

	value = DIV_ROUND_UP(clk_value_khz, PICOS2KHZ(info->var.pixclock));
	clkcon_reg = pic32_readl(sinfo->mmio, PIC32_LCD_REG_CLKCON);

	if (value < pix_factor) {
		dev_dbg(info->device, "Bypassing pixel clock divider\n");
		pic32_writel(sinfo->mmio, PIC32_LCD_REG_CLKCON, clkcon_reg & ~0x3F);
	} else {
		value = (value / pix_factor) - 1;
		dev_dbg(info->device, "  * programming CLKVAL = 0x%08lx\n",
			value);
		pic32_writel(sinfo->mmio, PIC32_LCD_REG_CLKCON,
			     (value & ~0x3F) | value);
		info->var.pixclock =
			KHZ2PICOS(clk_value_khz / (pix_factor * (value + 1)));
		dev_dbg(info->device, "  updated pixclk:     %lu KHz\n",
			PICOS2KHZ(info->var.pixclock));
	}

	pic32_writel(sinfo->mmio, PIC32_LCD_REG_LAYER0_STRIDE,
		     info->fix.line_length);

	pic32_writel(sinfo->mmio, PIC32_LCD_REG_LAYER0_MODE, 0
		     | (0xff << 16)
		     | sinfo->mode
		     | (1 << 8)
		     | PIC32_LCD_CONFIG_ENABLE);

	/* set mode in registers */
	pic32_writel(sinfo->mmio, PIC32_LCD_REG_MODE,
		     mode_reg | PIC32_LCD_CONFIG_ENABLE);

	return 0;
}

static int pic32_lcd_setcolreg(unsigned int regno, unsigned int red,
			       unsigned int green, unsigned int blue,
			       unsigned int transp, struct fb_info *info)
{
	struct pic32_lcd_info *sinfo = info->par;
	u32 out_val;

	if (regno >= PIC32_LCD_PALETTE_COLORS)
		return -EINVAL;

	/* grayscale = 0.30*R + 0.59*G + 0.11*B */
	if (info->var.grayscale) {
		blue = (red * 77 + green * 151 + blue * 28) >> 8;
		red = blue;
		green = blue;
	}

	if (info->fix.visual == FB_VISUAL_TRUECOLOR ||
	    info->fix.visual == FB_VISUAL_DIRECTCOLOR) {
		red = (red >> (8 + 8 - info->var.red.length));
		green = (green >> (8 + 8 - info->var.green.length));
		blue = (blue >> (8 + 8 - info->var.blue.length));
		transp = (transp >> (8 + 8 - info->var.transp.length));

		out_val = (red << info->var.red.offset) |
			(green << info->var.green.offset) |
			(blue << info->var.blue.offset);
	} else {
		red >>= 8;
		green >>= 8;
		blue >>= 8;
		transp >>= 8;

		out_val = (red << 16) | (green << 8) | (blue << 0);
	}

	/*
	 * the pseudo_palette expects color values in screen format,
	 * computed as seen above
	 */
	sinfo->pseudo_palette[regno] = out_val;

	/* the hardware always expects an RGB888 value */
	pic32_writel(sinfo->mmio, PIC32_LCD_PALETTE_OFFSET + regno * 4,
		     (red << 16) | (green << 8) | (blue << 0));

	return 0;
}

static int pic32_lcd_pan_display(struct fb_var_screeninfo *var,
				 struct fb_info *info)
{
	struct pic32_lcd_info *sinfo = info->par;
	unsigned long address;

	/* check bounds */
	if (var->vmode & FB_VMODE_YWRAP ||
	    var->xoffset + var->xres > info->var.xres_virtual ||
	    var->yoffset + var->yres > info->var.yres_virtual)
		return -EINVAL;

	info->var.xoffset = var->xoffset;
	info->var.yoffset = var->yoffset;

	/* compute new base address */
	address = info->fix.smem_start +
		(var->yoffset * info->fix.line_length + var->xoffset) *
		(var->bits_per_pixel >> 3);

	pic32_writel(sinfo->mmio, PIC32_LCD_REG_LAYER0_BASEADDR, address);

	return 0;
}

#ifdef PIC32_LCD_CURSOR
static void pic32_lcd_load_cursor_image(int width, int height, u8 *data,
					struct pic32_lcd_info *sinfo)
{
	u32 __iomem *p_addr = (u32 __iomem *)(sinfo->mmio +
					PIC32_LCD_CURSOR_IMAGE);
	u32 p;
	int i, j, k;
	u8 b, mod = width % 8;

	/* for all vertical lines... */
	for (i = height; i--; ) {
		/* for every whole byte... */
		for (j = 0; j < width / 8; j++) {
			/* fetch a new byte, ...*/
			b = *data++, p = 0;

			/* set the color respective to every bit to 0 or 1
			   (4 bits are needed) */
			for (k = 0; k < 8; k += 1)
				p |= (b & (1 << 7) ? 1 : 0) << (4 * k), b <<= 1;

			/* reverse the bytes if we must */
			pic32_writel(p_addr, 4 * j, cpu_to_be32(p));
		}

		/* for the remaining bits... */
		if (mod) {
			/* fetch a new byte, ... */
			b = *data++, p = 0;

			/* set the color respective to every bit to 0 or 1
			   (4 bits are needed) */
			for (k = 0; k < mod; k += 1)
				p |= (b & (1 << 7) ? 1 : 0) << (4 * k), b <<= 1;

			/* reverse the bytes if we must */
			pic32_writel(p_addr, 4 * j, cpu_to_be32(p));
		}

		/* move some words ahead in device memory */
		p_addr += PIC32_LCD_CURSOR_WIDTH >> 3;
	}
}

static int pic32_lcd_cursor(struct fb_info *info, struct fb_cursor *cursor)
{
	struct pic32_lcd_info *sinfo = info->par;
	struct device *dev = &sinfo->pdev->dev;
	unsigned int i;
	u32 fg_idx, bg_idx, fg, bg, size;
	u8 *data;

	if (cursor->image.width > PIC32_LCD_CURSOR_WIDTH ||
	    cursor->image.height > PIC32_LCD_CURSOR_HEIGHT)
		return -ENXIO;

	/* first of all, disable the cursor */
	pic32_writel(sinfo->mmio, PIC32_LCD_REG_MODE,
		     pic32_readl(sinfo->mmio, PIC32_LCD_REG_MODE) &
		     ~PIC32_LCD_CONFIG_CURSOR);

	if (cursor->set & FB_CUR_SETPOS) {
		dev_dbg(dev, "pic32_lcd_cursor: SETPOS x:%u y:%u\n",
			cursor->image.dx, cursor->image.dy);
		pic32_writel(sinfo->mmio, PIC32_LCD_REG_CURSORXY,
			     XY16TOREG32(cursor->image.dx, cursor->image.dy));
	}

	if (cursor->set & FB_CUR_SETCMAP) {
		dev_dbg(dev, "pic32_lcd_cursor: SET_CMAP fg:%u bg:%u\n",
			cursor->image.fg_color, cursor->image.bg_color);

		/* fetch the requested colors from the color map and program the
		 * device accordingly; only 2 colors are always used
		 */
		bg_idx = cursor->image.bg_color;
		fg_idx = cursor->image.fg_color;

		fg = (info->cmap.red[fg_idx] << 16)
			| (info->cmap.green[fg_idx] << 8)
			| (info->cmap.blue[fg_idx] << 0);
		bg = (info->cmap.red[bg_idx] << 16)
			| (info->cmap.green[bg_idx] << 8)
			| (info->cmap.blue[bg_idx] << 0);

		pic32_writel(sinfo->mmio, PIC32_LCD_CURSOR_CLUT + 4 * 1, fg);
		pic32_writel(sinfo->mmio, PIC32_LCD_CURSOR_CLUT + 4 * 0, bg);
	}

	if (cursor->set & (FB_CUR_SETSIZE |
				FB_CUR_SETSHAPE | FB_CUR_SETIMAGE)) {
		dev_dbg(dev,
			"pic32_lcd_cursor: w:%u h:%u\n",
			cursor->image.width, cursor->image.height);

		/* zero out the hardware cursor */
		for (i = 0; i < PIC32_LCD_CURSOR_WIDTH *
			     PIC32_LCD_CURSOR_HEIGHT / 8; i++)
			pic32_writel(sinfo->mmio,
				     PIC32_LCD_CURSOR_IMAGE + 4 * i, 0);

		/* allocate memory to apply the mask in software */
		size = ((cursor->image.width + 7) >> 3) * cursor->image.height;
		data = kmalloc(size, GFP_ATOMIC);

		if (!data)
			return -ENOMEM;

		/* apply the mask */
		switch (cursor->rop) {
		case ROP_XOR:
			for (i = 0; i < size; i++)
				data[i] =
					cursor->image.data[i] ^ cursor->mask[i];

			break;

		case ROP_COPY:
		default:
			for (i = 0; i < size; i++)
				data[i] =
					cursor->image.data[i] & cursor->mask[i];

			break;
		}

		/* load the image to the device */
		pic32_lcd_load_cursor_image(cursor->image.width,
					    cursor->image.height, data, sinfo);

		/* free the memory */
		kfree(data);
	}

	/* enable the cursor if we must */
	if (cursor->enable)
		pic32_writel(sinfo->mmio, PIC32_LCD_REG_MODE,
			     pic32_readl(sinfo->mmio, PIC32_LCD_REG_MODE) |
			     PIC32_LCD_CONFIG_CURSOR);

	return 0;
}
#endif

static int pic32_lcd_blank(int blank_mode, struct fb_info *info)
{
	struct pic32_lcd_info *sinfo = info->par;
	u32 mode_reg;

	/* blank out the screen by setting or clearing PIC32_LCD_MODE bit 31 */
	mode_reg = pic32_readl(sinfo->mmio, PIC32_LCD_REG_MODE);

	switch (blank_mode) {
	case FB_BLANK_UNBLANK:
	case FB_BLANK_NORMAL:
		pic32_writel(sinfo->mmio, PIC32_LCD_REG_MODE,
			     mode_reg | PIC32_LCD_CONFIG_ENABLE);
		break;
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
		break;
	case FB_BLANK_POWERDOWN:
		pic32_writel(sinfo->mmio, PIC32_LCD_REG_MODE,
			     mode_reg & ~PIC32_LCD_CONFIG_ENABLE);
		break;
	default:
		return -EINVAL;
	}

	/* let fbcon do a soft blank for us */
	return ((blank_mode == FB_BLANK_NORMAL) ? 1 : 0);
}

static irqreturn_t pic32_lcd_vsync_interrupt(int irq, void *dev_id)
{
	struct fb_info *info = dev_id;
	struct pic32_lcd_info *sinfo = info->par;

	/* clear the interrupt */
	pic32_writel(sinfo->mmio, PIC32_CLR(PIC32_LCD_REG_INTERRUPT), 1);

	/* update stats, also needed as a condition to unblock */
	sinfo->vblank_count++;

	/* wake up any threads waiting */
	wake_up_interruptible(&sinfo->wait_vsync);

	return IRQ_HANDLED;
}

static int pic32_lcd_vsync(struct fb_info *info)
{
	struct pic32_lcd_info *sinfo = info->par;
	u64 count;

	/* enable vsync interrupt; it will be cleared on arrival */
	count = sinfo->vblank_count;
	pic32_writel(sinfo->mmio, PIC32_SET(PIC32_LCD_REG_INTERRUPT), 1);

	/* wait for it for a while */
	if (!wait_event_interruptible_timeout(sinfo->wait_vsync,
					      count != sinfo->vblank_count,
					      HZ / 10))
		return -ETIMEDOUT;

	return 0;
}

static int pic32_lcd_ioctl(struct fb_info *info, unsigned int cmd,
			   unsigned long arg)
{
	struct pic32_lcd_info *sinfo = info->par;
	struct device *dev = &sinfo->pdev->dev;
	int i, mode;
	int red, green, blue;

	switch (cmd) {
	case FBIO_WAITFORVSYNC:
		return pic32_lcd_vsync(info);
	case FBIO_RAMP_SET:

		mode = pic32_readl(sinfo->mmio, PIC32_LCD_REG_MODE);

		if (arg == 0) {
			pic32_writel(sinfo->mmio, PIC32_LCD_REG_MODE,
				     mode & ~PIC32_LCD_CONFIG_GAMMA);
		} else {
			for (i = 0; i < 256; i++) {
				red   = CLAMP255(i + arg);
				green = CLAMP255(i + arg);
				blue  = CLAMP255(i + arg);
				pic32_writel(sinfo->mmio,
					     PIC32_LCD_PALETTE_OFFSET + i * 4,
					     (red << 16) | (green << 8) |
					     (blue << 0));
			}

			pic32_writel(sinfo->mmio, PIC32_LCD_REG_MODE,
				     mode | PIC32_LCD_CONFIG_GAMMA);
		}

		return 1;
	default:
		dev_err(dev, "Unknown ioctl 0x%08x has been requested\n", cmd);
		return -EINVAL;
	}

	return 0;
}

static inline void pic32_lcd_free_video_memory(struct pic32_lcd_info *sinfo)
{
	struct fb_info *info = sinfo->info;

	dma_free_coherent(info->device, info->fix.smem_len,
			  info->screen_base, info->fix.smem_start);
}

static int pic32_lcd_alloc_video_memory(struct pic32_lcd_info *sinfo)
{
	struct fb_info *info = sinfo->info;
	struct fb_var_screeninfo *var = &info->var;
	unsigned int smem_len;

	smem_len = (var->xres_virtual * var->yres_virtual
		    * ((var->bits_per_pixel + 7) / 8));

	info->fix.smem_len = max(smem_len, sinfo->smem_len);

	info->screen_base =
		dma_alloc_coherent(info->device, info->fix.smem_len,
				   (dma_addr_t *)&info->fix.smem_start,
				   GFP_KERNEL);

	if (!info->screen_base)
		return -ENOMEM;

	memset(info->screen_base, 0, info->fix.smem_len);

	return 0;
}

static int pic32_lcd_of_init(struct pic32_lcd_info *sinfo)
{
	struct fb_info *info = sinfo->info;
	struct fb_var_screeninfo *var = &info->var;
	struct device *dev = &sinfo->pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *timings_np;
	struct display_timings *timings;
	int ret = -ENOENT;
	int i;
	u32 value;

	ret = of_property_read_u32(np, "bits-per-pixel", &var->bits_per_pixel);
	if (ret < 0) {
		dev_err(dev, "failed to get property bits-per-pixel\n");
		goto put_display_node;
	}

	ret = of_property_read_u32(np, "pic32,grayscale", &value);
	if (ret >= 0)
		info->var.grayscale = value;

	ret = of_property_read_u32(np, "pic32,transp-length", &value);
	if (ret >= 0)
		info->var.transp.length = value;

	ret = of_property_read_u32(np, "pic32,red-offset", &value);
	if (ret >= 0)
		info->var.red.offset = value;

	timings = of_get_display_timings(np);
	if (!timings) {
		dev_err(dev, "failed to get display timings\n");
		ret = -EINVAL;
		goto put_display_node;
	}

	timings_np = of_find_node_by_name(np, "display-timings");
	if (!timings_np) {
		dev_err(dev, "failed to find display-timings node\n");
		ret = -ENODEV;
		goto put_display_node;
	}

	for (i = 0; i < of_get_child_count(timings_np); i++) {
		struct videomode vm;
		struct fb_videomode fb_vm;

		ret = videomode_from_timings(timings, &vm, i);
		if (ret < 0)
			goto put_timings_node;
		ret = fb_videomode_from_videomode(&vm, &fb_vm);
		if (ret < 0)
			goto put_timings_node;

		fb_add_videomode(&fb_vm, &info->modelist);
	}

	return 0;

put_timings_node:
	of_node_put(timings_np);
put_display_node:
	return ret;
}

static int __init pic32_lcd_init_fbinfo(struct pic32_lcd_info *sinfo)
{
	struct fb_info *info = sinfo->info;
	int ret = 0;

	info->var.nonstd = 0;
	info->var.height = -1;
	info->var.width = -1;

	info->var.activate |= FB_ACTIVATE_FORCE | FB_ACTIVATE_NOW;
	info->var.vmode = FB_VMODE_NONINTERLACED;

	dev_info(info->device,
		 "%luKiB framebuffer at %08lx (mapped at %p)\n",
		 (unsigned long)info->fix.smem_len / 1024,
		 (unsigned long)info->fix.smem_start,
		 info->screen_base);

	/* Allocate colormap */
	ret = fb_alloc_cmap(&info->cmap, 256, 0);
	if (ret < 0)
		dev_err(info->device, "Alloc color map failed\n");

	return ret;
}

static void pic32_lcd_start_clock(struct pic32_lcd_info *sinfo)
{
	clk_prepare_enable(sinfo->bus_clk);
	clk_prepare_enable(sinfo->lcdc_clk);
}

static void pic32_lcd_stop_clock(struct pic32_lcd_info *sinfo)
{
	clk_disable_unprepare(sinfo->bus_clk);
	clk_disable_unprepare(sinfo->lcdc_clk);
}

static struct fb_ops pic32_lcd_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= pic32_lcd_check_var,
	.fb_set_par	= pic32_lcd_set_par,
	.fb_setcolreg	= pic32_lcd_setcolreg,
	.fb_pan_display	= pic32_lcd_pan_display,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_blank	= pic32_lcd_blank,
	.fb_ioctl	= pic32_lcd_ioctl,
#ifdef PIC32_LCD_CURSOR
	.fb_cursor	= pic32_lcd_cursor,
#endif
};

static struct fb_fix_screeninfo pic32_lcd_fix = {
	.type		= FB_TYPE_PACKED_PIXELS,
	.visual		= FB_VISUAL_TRUECOLOR,
	.xpanstep	= 1,
	.ypanstep	= 1,
	.ywrapstep	= 0,
	.accel		= PIC32_LCD_ACCEL,
};

static int pic32_lcd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fb_info *info;
	struct pic32_lcd_info *sinfo;
	struct resource *regs = NULL;
	struct resource *map = NULL;
	struct fb_modelist *modelist;
	int ret;

	ret = -ENOMEM;
	info = framebuffer_alloc(sizeof(struct pic32_lcd_info), dev);
	if (!info) {
		dev_err(dev,
			"failed to allocate framebuffer\n");
		goto out;
	}

	sinfo = info->par;
	sinfo->pdev = pdev;
	sinfo->info = info;

	INIT_LIST_HEAD(&info->modelist);

	if (pdev->dev.of_node) {
		ret = pic32_lcd_of_init(sinfo);
		if (ret)
			goto free_info;
	} else {
		dev_err(dev, "cannot get default configuration\n");
		goto free_info;
	}

	info->flags = FBINFO_FLAG_DEFAULT
		| FBINFO_HWACCEL_XPAN
		| FBINFO_HWACCEL_YPAN
		| FBINFO_PARTIAL_PAN_OK;
	info->pseudo_palette = sinfo->pseudo_palette;
	info->fbops = &pic32_lcd_ops;

	info->fix = pic32_lcd_fix;
	strcpy(info->fix.id, sinfo->pdev->name);

	/* Enable LCDC Clocks */
	sinfo->bus_clk = clk_get(dev, "sys_clk");
	if (IS_ERR(sinfo->bus_clk)) {
		ret = PTR_ERR(sinfo->bus_clk);
		goto free_info;
	}
	sinfo->lcdc_clk = clk_get(dev, "lcd_clk");
	if (IS_ERR(sinfo->lcdc_clk)) {
		ret = PTR_ERR(sinfo->lcdc_clk);
		goto put_bus_clk;
	}
	pic32_lcd_start_clock(sinfo);

	modelist = list_first_entry(&info->modelist,
				    struct fb_modelist, list);
	fb_videomode_to_var(&info->var, &modelist->mode);

	pic32_lcd_check_var(&info->var, info);

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(dev, "resources unusable\n");
		ret = -ENXIO;
		goto stop_clk;
	}

	sinfo->irq = platform_get_irq(pdev, 0);
	if (sinfo->irq < 0) {
		dev_err(dev, "unable to get irq\n");
		ret = sinfo->irq;
		goto stop_clk;
	}

	/* Initialize video memory */
	map = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (map) {
		/* use a pre-allocated memory buffer */
		info->fix.smem_start = map->start;
		info->fix.smem_len = resource_size(map);
		if (!request_mem_region(info->fix.smem_start,
					info->fix.smem_len, pdev->name)) {
			ret = -EBUSY;
			goto stop_clk;
		}

		info->screen_base = ioremap(info->fix.smem_start,
					info->fix.smem_len);
		if (!info->screen_base) {
			ret = -ENOMEM;
			goto release_intmem;
		}

		/*
		 * Don't clear the framebuffer -- someone may have set
		 * up a splash image.
		 */
	} else {
		/* allocate memory buffer */
		ret = pic32_lcd_alloc_video_memory(sinfo);
		if (ret < 0) {
			dev_err(dev, "cannot allocate framebuffer: %d\n", ret);
			goto stop_clk;
		}
	}

	/* LCDC registers */
	info->fix.mmio_start = regs->start;
	info->fix.mmio_len = resource_size(regs);

	if (!request_mem_region(info->fix.mmio_start,
				info->fix.mmio_len, pdev->name)) {
		ret = -EBUSY;
		goto free_fb;
	}

	sinfo->mmio = ioremap(info->fix.mmio_start, info->fix.mmio_len);
	if (!sinfo->mmio) {
		dev_err(dev, "cannot map LCDC registers\n");
		ret = -ENOMEM;
		goto release_mem;
	}

	/* initialize the wait object for interrupt */
	init_waitqueue_head(&sinfo->wait_vsync);

	/* interrupt */
	ret = devm_request_irq(dev, sinfo->irq,
			       pic32_lcd_vsync_interrupt, 0,
			       pdev->name, info);
	if (ret) {
		dev_err(dev, "request_irq failed: %d\n", ret);
		goto unmap_mmio;
	}

	/* initialize register file */
	pic32_writel(sinfo->mmio, PIC32_LCD_REG_MODE, 0);
	pic32_writel(sinfo->mmio, PIC32_LCD_REG_CLKCON, 0 | (5 << 8));
	pic32_writel(sinfo->mmio, PIC32_LCD_REG_LAYER0_BASEADDR,
		     info->fix.smem_start);
	pic32_writel(sinfo->mmio, PIC32_LCD_REG_BGCOLOR, 0);
	pic32_writel(sinfo->mmio, PIC32_LCD_REG_INTERRUPT, 1 << 31);

	sinfo->pdata = dev->platform_data;
	if (sinfo->pdata && sinfo->pdata->enable)
		sinfo->pdata->enable();

	ret = pic32_lcd_init_fbinfo(sinfo);
	if (ret < 0) {
		dev_err(dev, "init fbinfo failed: %d\n", ret);
		goto unregister_irqs;
	}

	ret = pic32_lcd_set_par(info);
	if (ret < 0) {
		dev_err(dev, "set par failed: %d\n", ret);
		goto unregister_irqs;
	}

	dev_set_drvdata(dev, info);

	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(dev, "failed to register framebuffer: %d\n", ret);
		goto reset_drvdata;
	}

	dev_info(dev,
		 "fb%d: %s framebuffer at 0x%08lx (mapped at %p), irq %d\n",
		 info->node, info->fix.id, info->fix.mmio_start,
		 sinfo->mmio, sinfo->irq);

	return 0;

reset_drvdata:
	dev_set_drvdata(dev, NULL);
	fb_dealloc_cmap(&info->cmap);
unregister_irqs:
	free_irq(sinfo->irq, info);
unmap_mmio:
	iounmap(sinfo->mmio);
release_mem:
	release_mem_region(info->fix.mmio_start, info->fix.mmio_len);
free_fb:
	if (map)
		iounmap(info->screen_base);
	else
		pic32_lcd_free_video_memory(sinfo);

release_intmem:
	if (map)
		release_mem_region(info->fix.smem_start, info->fix.smem_len);
stop_clk:
	pic32_lcd_stop_clock(sinfo);
	clk_put(sinfo->lcdc_clk);
put_bus_clk:
	clk_put(sinfo->bus_clk);
free_info:
	framebuffer_release(info);
out:
	dev_dbg(dev, "%s FAILED\n", __func__);

	return ret;
}

static int pic32_lcd_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fb_info *info = dev_get_drvdata(dev);
	struct pic32_lcd_info *sinfo;

	if (!info || !info->par)
		return 0;
	sinfo = info->par;

	unregister_framebuffer(info);
	pic32_lcd_stop_clock(sinfo);
	fb_dealloc_cmap(&info->cmap);
	iounmap(sinfo->mmio);
	release_mem_region(info->fix.mmio_start, info->fix.mmio_len);
	pic32_lcd_free_video_memory(sinfo);

	framebuffer_release(info);
	if (sinfo->pdata && sinfo->pdata->disable)
		sinfo->pdata->disable();

	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id pic32_dt_ids[] = {
	{ .compatible = "microchip,pic32-lcd" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, pic32_dt_ids);
#endif

static struct platform_driver pic32_lcd_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = "pic32-lcd",
		.of_match_table	= of_match_ptr(pic32_dt_ids),
	},

	.remove = pic32_lcd_remove,
};

module_platform_driver_probe(pic32_lcd_driver, pic32_lcd_probe);

MODULE_DESCRIPTION("Microchip PIC32 LCD Framebuffer Driver");
MODULE_AUTHOR("Joshua Henderson <joshua.henderson@microchip.com>");
MODULE_LICENSE("GPL");
