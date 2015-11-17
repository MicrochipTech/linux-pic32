/*
 * PIC32 Integrated Serial Driver.
 *
 * Copyright (C) 2015 Microchip Technology, Inc.
 *
 * Authors:
 *   Steve Scott <steve.scott@microchip.com>,
 *   Sorin-Andrei Pistirica <andrei.pistirica@microchip.com>
 *
 * Licensed under GPLv2 or later.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/console.h>
#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/sysrq.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <uapi/linux/serial_core.h>
#include <linux/delay.h>

#include "pic32_uart.h"

/* UART name and device definitions */
#define PIC32_DEV_NAME		"pic32-usart"
#define PIC32_MAX_UARTS		6

#define PIC32_SDEV_NAME		"ttyS"
#define PIC32_SDEV_MAJOR	TTY_MAJOR
#define PIC32_SDEV_MINOR	64

/* pic32_sport pointer for console use */
static struct pic32_sport *pic32_sports[PIC32_MAX_UARTS];

static inline int pic32_enable_clock(struct pic32_sport *sport)
{
	sport->ref_clk++;

	return clk_prepare_enable(sport->clk);
}

static inline void pic32_disable_clock(struct pic32_sport *sport)
{
	sport->ref_clk--;
	clk_disable_unprepare(sport->clk);
}

/* serial core request to check if uart tx buffer is empty */
static unsigned int pic32_uart_tx_empty(struct uart_port *port)
{
	struct pic32_sport *sport = to_pic32_sport(port);
	u32 val = pic32_uart_read(sport, PIC32_UART_STA);

	return (val & PIC32_UART_STA_TRMT) ? 1 : 0;
}

/* serial core request to set UART outputs */
static void pic32_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct pic32_sport *sport = to_pic32_sport(port);

	/* set loopback mode */
	if (mctrl & TIOCM_LOOP)
		pic32_uart_rset(PIC32_UART_MODE_LPBK, sport, PIC32_UART_MODE);
	else
		pic32_uart_rclr(PIC32_UART_MODE_LPBK, sport, PIC32_UART_MODE);
}

/* get the state of CTS input pin for this port */
static unsigned int get_cts_state(struct pic32_sport *sport)
{
	/* default state must be asserted */
	int val = 1;

	/* read and invert UxCTS */
	if (gpio_is_valid(sport->cts_gpio))
		val = !gpio_get_value(sport->cts_gpio);

	return val;
}

/* serial core request to return the state of misc UART input pins */
static unsigned int pic32_uart_get_mctrl(struct uart_port *port)
{
	struct pic32_sport *sport = to_pic32_sport(port);
	unsigned int mctrl = 0;

	if (!sport->hw_flow_ctrl) {
		mctrl |= TIOCM_CTS;
		goto ret;
	}

	if (get_cts_state(sport))
		mctrl |= TIOCM_CTS;

ret:
	/* DSR and CD are not supported in PIC32, so return 1
	 * RI is not supported in PIC32, so return 0
	 */
	mctrl |= TIOCM_CD;
	mctrl |= TIOCM_DSR;

	return mctrl;
}

/* stop tx and start tx are not called in pairs, therefore a flag indicates
 * the status of irq to control the irq-depth.
 */
static inline void pic32_uart_irqtxen(struct pic32_sport *sport, u8 en)
{
	if (en && !tx_irq_enabled(sport)) {
		enable_irq(sport->irq_tx);
		tx_irq_enabled(sport) = 1;
	} else if (!en && tx_irq_enabled(sport)) {
		/* use disable_irq_nosync() and not disable_irq() to avoid self
		 * imposed deadlock by not waiting for irq handler to end,
		 * since this callback is called from interrupt context.
		 */
		disable_irq_nosync(sport->irq_tx);
		tx_irq_enabled(sport) = 0;
	}
}

/* serial core request to disable tx ASAP (used for flow control) */
static void pic32_uart_stop_tx(struct uart_port *port)
{
	struct pic32_sport *sport = to_pic32_sport(port);

	if (!(pic32_uart_read(sport, PIC32_UART_MODE) & PIC32_UART_MODE_ON))
		return;

	if (!(pic32_uart_read(sport, PIC32_UART_STA) & PIC32_UART_STA_UTXEN))
		return;

	/* wait for tx empty */
	while (!(pic32_uart_read(sport, PIC32_UART_STA) & PIC32_UART_STA_TRMT))
		udelay(1);

	pic32_uart_rclr(PIC32_UART_STA_UTXEN, sport, PIC32_UART_STA);
	pic32_uart_irqtxen(sport, 0);
}

/* serial core request to (re)enable tx */
static void pic32_uart_start_tx(struct uart_port *port)
{
	struct pic32_sport *sport = to_pic32_sport(port);

	pic32_uart_irqtxen(sport, 1);
	pic32_uart_rset(PIC32_UART_STA_UTXEN, sport, PIC32_UART_STA);
}

/* serial core request to stop rx, called before port shutdown */
static void pic32_uart_stop_rx(struct uart_port *port)
{
	struct pic32_sport *sport = to_pic32_sport(port);

	/* disable rx interrupts */
	disable_irq(sport->irq_rx);

	/* receiver Enable bit OFF */
	pic32_uart_rclr(PIC32_UART_STA_URXEN, sport, PIC32_UART_STA);
}

/* serial core request to start/stop emitting break char */
static void pic32_uart_break_ctl(struct uart_port *port, int ctl)
{
	struct pic32_sport *sport = to_pic32_sport(port);
	unsigned long flags = 0;

	spin_lock_irqsave(&port->lock, flags);

	if (ctl)
		pic32_uart_rset(PIC32_UART_STA_UTXBRK, sport, PIC32_UART_STA);
	else
		pic32_uart_rclr(PIC32_UART_STA_UTXBRK, sport, PIC32_UART_STA);

	spin_unlock_irqrestore(&port->lock, flags);
}

/* get port type in string format */
static const char *pic32_uart_type(struct uart_port *port)
{
	return (port->type == PORT_PIC32) ? PIC32_DEV_NAME : NULL;
}

/* read all chars in rx fifo and send them to core */
static void pic32_uart_do_rx(struct uart_port *port)
{
	struct pic32_sport *sport = to_pic32_sport(port);
	struct tty_port *tty;
	unsigned int max_count;

	/* limit number of char read in interrupt, should not be
	 * higher than fifo size anyway since we're much faster than
	 * serial port
	 */
	max_count = PIC32_UART_RX_FIFO_DEPTH;

	spin_lock(&port->lock);

	tty = &port->state->port;

	do {
		u32 sta_reg, c;
		char flag;

		/* get overrun/fifo empty information from status register */
		sta_reg = pic32_uart_read(sport, PIC32_UART_STA);
		if (unlikely(sta_reg & PIC32_UART_STA_OERR)) {

			/* fifo reset is required to clear interrupt */
			pic32_uart_rclr(PIC32_UART_STA_OERR, sport,
							PIC32_UART_STA);

			port->icount.overrun++;
			tty_insert_flip_char(tty, 0, TTY_OVERRUN);
		}

		/* Can at least one more character can be read? */
		if (!(sta_reg & PIC32_UART_STA_URXDA))
			break;

		/* read the character and increment the rx counter */
		c = pic32_uart_read(sport, PIC32_UART_RX);

		port->icount.rx++;
		flag = TTY_NORMAL;
		c &= 0xff;

		if (unlikely((sta_reg & PIC32_UART_STA_PERR) ||
			     (sta_reg & PIC32_UART_STA_FERR))) {

			/* do stats first */
			if (sta_reg & PIC32_UART_STA_PERR)
				port->icount.parity++;
			if (sta_reg & PIC32_UART_STA_FERR)
				port->icount.frame++;

			/* update flag wrt read_status_mask */
			sta_reg &= port->read_status_mask;

			if (sta_reg & PIC32_UART_STA_FERR)
				flag = TTY_FRAME;
			if (sta_reg & PIC32_UART_STA_PERR)
				flag = TTY_PARITY;
		}

		if (uart_handle_sysrq_char(port, c))
			continue;

		if ((sta_reg & port->ignore_status_mask) == 0)
			tty_insert_flip_char(tty, c, flag);

	} while (--max_count);

	spin_unlock(&port->lock);

	tty_flip_buffer_push(tty);
}

/* fill tx fifo with chars to send, stop when fifo is about to be full
 * or when all chars have been sent.
 */
static void pic32_uart_do_tx(struct uart_port *port)
{
	struct pic32_sport *sport = to_pic32_sport(port);
	struct circ_buf *xmit = &port->state->xmit;
	unsigned int max_count = PIC32_UART_TX_FIFO_DEPTH;

	if (port->x_char) {
		pic32_uart_write(port->x_char, sport, PIC32_UART_TX);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}

	if (uart_tx_stopped(port)) {
		pic32_uart_stop_tx(port);
		return;
	}

	if (uart_circ_empty(xmit))
		goto txq_empty;

	/* keep stuffing chars into uart tx buffer
	 * 1) until uart fifo is full
	 * or
	 * 2) until the circ buffer is empty
	 * (all chars have been sent)
	 * or
	 * 3) until the max count is reached
	 * (prevents lingering here for too long in certain cases)
	 */
	while (!(PIC32_UART_STA_UTXBF &
		pic32_uart_rval(sport, PIC32_UART_STA))) {
		unsigned int c = xmit->buf[xmit->tail];

		pic32_uart_write(c, sport, PIC32_UART_TX);

		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		--max_count;
		if (uart_circ_empty(xmit))
			break;
		if (max_count == 0)
			break;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit))
		goto txq_empty;

	return;

txq_empty:
	pic32_uart_irqtxen(sport, 0);
}

/* RX interrupt handler */
static irqreturn_t pic32_uart_rx_interrupt(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;

	pic32_uart_do_rx(port);

	return IRQ_HANDLED;
}

/* TX interrupt handler */
static irqreturn_t pic32_uart_tx_interrupt(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	pic32_uart_do_tx(port);
	spin_unlock_irqrestore(&port->lock, flags);

	return IRQ_HANDLED;
}

/* FAULT interrupt handler */
static irqreturn_t pic32_uart_fault_interrupt(int irq, void *dev_id)
{
	/* do nothing: pic32_uart_do_rx() handles faults. */
	return IRQ_HANDLED;
}

/* enable rx & tx operation on uart */
static void pic32_uart_en_and_unmask(struct uart_port *port)
{
	struct pic32_sport *sport = to_pic32_sport(port);

	pic32_uart_rset(PIC32_UART_STA_UTXEN | PIC32_UART_STA_URXEN,
			sport, PIC32_UART_STA);
	pic32_uart_rset(PIC32_UART_MODE_ON, sport, PIC32_UART_MODE);
}

/* disable rx & tx operation on uart */
static void pic32_uart_dsbl_and_mask(struct uart_port *port)
{
	struct pic32_sport *sport = to_pic32_sport(port);

	pic32_uart_rclr(PIC32_UART_MODE_ON, sport, PIC32_UART_MODE);
	pic32_uart_rclr(PIC32_UART_STA_UTXEN | PIC32_UART_STA_URXEN,
			sport, PIC32_UART_STA);
}

/* serial core request to initialize uart and start rx operation */
static int pic32_uart_startup(struct uart_port *port)
{
	struct pic32_sport *sport = to_pic32_sport(port);
	u32 dflt_baud = ((port->uartclk / PIC32_UART_DFLT_BRATE) / 16) - 1;
	unsigned long flags;
	int ret = 0;

	local_irq_save(flags);

	ret = pic32_enable_clock(sport);
	if (ret)
		goto out_unlock;

	/* clear status and mode registers */
	pic32_uart_write(0, sport, PIC32_UART_MODE);
	pic32_uart_write(0, sport, PIC32_UART_STA);

	/* disable uart and mask all interrupts */
	pic32_uart_dsbl_and_mask(port);

	/* set default baud */
	pic32_uart_write(dflt_baud, sport, PIC32_UART_BRG);

	local_irq_restore(flags);

	/* Each UART of a PIC32 has three interrupts therefore,
	 * we setup driver to register the 3 irqs for the device.
	 *
	 * For each irq request_irq() is called with interrupt disabled.
	 * And the irq is enabled as soon as we are ready to handle them.
	 */
	tx_irq_enabled(sport) = 0;

	sport->irq_fault_name = kasprintf(GFP_KERNEL, "%s%d-fault",
					  pic32_uart_type(port),
					  sport->idx);
	irq_set_status_flags(sport->irq_fault, IRQ_NOAUTOEN);
	ret = request_irq(sport->irq_fault, pic32_uart_fault_interrupt,
			  sport->irqflags_fault, sport->irq_fault_name, port);
	if (ret) {
		dev_err(port->dev, "%s: request irq(%d) err! ret:%d name:%s\n",
			__func__, sport->irq_fault, ret,
			pic32_uart_type(port));
		goto out_done;
	}

	sport->irq_rx_name = kasprintf(GFP_KERNEL, "%s%d-rx",
				       pic32_uart_type(port),
				       sport->idx);
	irq_set_status_flags(sport->irq_rx, IRQ_NOAUTOEN);
	ret = request_irq(sport->irq_rx, pic32_uart_rx_interrupt,
			  sport->irqflags_rx, sport->irq_rx_name, port);
	if (ret) {
		dev_err(port->dev, "%s: request irq(%d) err! ret:%d name:%s\n",
			__func__, sport->irq_rx, ret,
			pic32_uart_type(port));
		goto out_done;
	}

	sport->irq_tx_name = kasprintf(GFP_KERNEL, "%s%d-tx",
				       pic32_uart_type(port),
				       sport->idx);
	irq_set_status_flags(sport->irq_tx, IRQ_NOAUTOEN);
	ret = request_irq(sport->irq_tx, pic32_uart_tx_interrupt,
			  sport->irqflags_tx, sport->irq_tx_name, port);
	if (ret) {
		dev_err(port->dev, "%s: request irq(%d) err! ret:%d name:%s\n",
			__func__, sport->irq_tx, ret,
			pic32_uart_type(port));
		goto out_done;
	}

	local_irq_save(flags);

	/* set rx interrupt on first receive */
	pic32_uart_rclr(PIC32_UART_STA_URXISEL1 | PIC32_UART_STA_URXISEL0,
							sport, PIC32_UART_STA);

	/* set interrupt on empty */
	pic32_uart_rclr(PIC32_UART_STA_UTXISEL1, sport, PIC32_UART_STA);

	/* enable all interrupts and eanable uart */
	pic32_uart_en_and_unmask(port);

	enable_irq(sport->irq_fault);
	enable_irq(sport->irq_rx);

out_unlock:
	local_irq_restore(flags);

out_done:
	return ret;
}

/* serial core request to flush & disable uart */
static void pic32_uart_shutdown(struct uart_port *port)
{
	struct pic32_sport *sport = to_pic32_sport(port);
	unsigned long flags;

	/* disable uart */
	spin_lock_irqsave(&port->lock, flags);
	pic32_uart_dsbl_and_mask(port);
	spin_unlock_irqrestore(&port->lock, flags);
	pic32_disable_clock(sport);

	/* free all 3 interrupts for this UART */
	free_irq(sport->irq_fault, port);
	free_irq(sport->irq_tx, port);
	free_irq(sport->irq_rx, port);
}

/* serial core request to change current uart setting */
static void pic32_uart_set_termios(struct uart_port *port,
				   struct ktermios *new,
				   struct ktermios *old)
{
	struct pic32_sport *sport = to_pic32_sport(port);
	unsigned int baud;
	unsigned int quot;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	/* disable uart and mask all interrupts while changing speed */
	pic32_uart_dsbl_and_mask(port);

	/* stop bit options */
	if (new->c_cflag & CSTOPB)
		pic32_uart_rset(PIC32_UART_MODE_STSEL, sport, PIC32_UART_MODE);
	else
		pic32_uart_rclr(PIC32_UART_MODE_STSEL, sport, PIC32_UART_MODE);

	/* parity options */
	if (new->c_cflag & PARENB) {
		if (new->c_cflag & PARODD) {
			pic32_uart_rset(PIC32_UART_MODE_PDSEL1, sport,
					PIC32_UART_MODE);
			pic32_uart_rclr(PIC32_UART_MODE_PDSEL0, sport,
					PIC32_UART_MODE);
		} else {
			pic32_uart_rset(PIC32_UART_MODE_PDSEL0, sport,
					PIC32_UART_MODE);
			pic32_uart_rclr(PIC32_UART_MODE_PDSEL1, sport,
					PIC32_UART_MODE);
		}
	} else {
		pic32_uart_rclr(PIC32_UART_MODE_PDSEL1 | PIC32_UART_MODE_PDSEL0,
				sport, PIC32_UART_MODE);
	}
	/* if hw flow ctrl, then the pins must be specified in device tree */
	if ((new->c_cflag & CRTSCTS) && sport->hw_flow_ctrl) {
		/* enable hardware flow control */
		pic32_uart_rset(PIC32_UART_MODE_UEN1, sport, PIC32_UART_MODE);
		pic32_uart_rclr(PIC32_UART_MODE_UEN0, sport, PIC32_UART_MODE);
		pic32_uart_rclr(PIC32_UART_MODE_RTSMD, sport, PIC32_UART_MODE);
	} else {
		/* disable hardware flow control */
		pic32_uart_rclr(PIC32_UART_MODE_UEN1, sport, PIC32_UART_MODE);
		pic32_uart_rclr(PIC32_UART_MODE_UEN0, sport, PIC32_UART_MODE);
		pic32_uart_rclr(PIC32_UART_MODE_RTSMD, sport, PIC32_UART_MODE);
	}

	/* update baud */
	baud = uart_get_baud_rate(port, new, old, 0, port->uartclk / 16);
	quot = uart_get_divisor(port, baud) - 1;
	pic32_uart_write(quot, sport, PIC32_UART_BRG);
	uart_update_timeout(port, new->c_cflag, baud);

	/* enable uart */
	pic32_uart_en_and_unmask(port);

	spin_unlock_irqrestore(&port->lock, flags);
}

/* serial core request to claim uart iomem */
static int pic32_uart_request_port(struct uart_port *port)
{
	struct platform_device *pdev = to_platform_device(port->dev);
	struct resource *res_mem;
	unsigned int res_size;

	res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!res_mem))
		return -EINVAL;
	res_size = resource_size(res_mem);

	if (!request_mem_region(port->mapbase, res_size, "pic32_uart_mem")) {
		dev_err(port->dev, "Memory region busy\n");
		return -EBUSY;
	}

	port->membase = devm_ioremap_nocache(port->dev,
					     port->mapbase, res_size);
	if (!port->membase) {
		dev_err(port->dev, "Unable to map registers\n");
		release_mem_region(port->mapbase, res_size);
		return -ENOMEM;
	}

	return 0;
}

/* serial core request to release uart iomem */
static void pic32_uart_release_port(struct uart_port *port)
{
	struct platform_device *pdev = to_platform_device(port->dev);
	struct resource *res_mem;
	unsigned int res_size;

	res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!res_mem))
		return;
	res_size = resource_size(res_mem);

	release_mem_region(port->mapbase, res_size);
	devm_iounmap(port->dev, port->membase);
}

/* serial core request to do any port required auto-configuration */
static void pic32_uart_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		if (pic32_uart_request_port(port))
			return;
		port->type = PORT_PIC32;
	}
}

/* serial core request to check that port information in serinfo are suitable */
static int pic32_uart_verify_port(struct uart_port *port,
				  struct serial_struct *serinfo)
{
	if (port->type != PORT_PIC32)
		return -EINVAL;
	if (port->irq != serinfo->irq)
		return -EINVAL;
	if (port->iotype != serinfo->io_type)
		return -EINVAL;
	if (port->mapbase != (unsigned long)serinfo->iomem_base)
		return -EINVAL;

	return 0;
}

/* serial core callbacks */
static const struct uart_ops pic32_uart_ops = {
	.tx_empty	= pic32_uart_tx_empty,
	.get_mctrl	= pic32_uart_get_mctrl,
	.set_mctrl	= pic32_uart_set_mctrl,
	.start_tx	= pic32_uart_start_tx,
	.stop_tx	= pic32_uart_stop_tx,
	.stop_rx	= pic32_uart_stop_rx,
	.break_ctl	= pic32_uart_break_ctl,
	.startup	= pic32_uart_startup,
	.shutdown	= pic32_uart_shutdown,
	.set_termios	= pic32_uart_set_termios,
	.type		= pic32_uart_type,
	.release_port	= pic32_uart_release_port,
	.request_port	= pic32_uart_request_port,
	.config_port	= pic32_uart_config_port,
	.verify_port	= pic32_uart_verify_port,
};

#ifdef CONFIG_SERIAL_PIC32_CONSOLE
/* output given char */
static void pic32_console_putchar(struct uart_port *port, int ch)
{
	struct pic32_sport *sport = to_pic32_sport(port);

	if (!(pic32_uart_read(sport, PIC32_UART_MODE) & PIC32_UART_MODE_ON))
		return;

	if (!(pic32_uart_read(sport, PIC32_UART_STA) & PIC32_UART_STA_UTXEN))
		return;

	/* wait for tx empty */
	while (!(pic32_uart_read(sport, PIC32_UART_STA) & PIC32_UART_STA_TRMT))
		udelay(1);

	pic32_uart_write(ch & 0xff, sport, PIC32_UART_TX);
}

/* console core request to output given string */
static void pic32_console_write(struct console *co, const char *s,
				unsigned int count)
{
	struct pic32_sport *sport = pic32_sports[co->index];
	struct uart_port *port = pic32_get_port(sport);

	/* call uart helper to deal with \r\n */
	uart_console_write(port, s, count, pic32_console_putchar);
}

/* console core request to setup given console, find matching uart
 * port and setup it.
 */
static int pic32_console_setup(struct console *co, char *options)
{
	struct pic32_sport *sport;
	struct uart_port *port = NULL;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	int ret = 0;

	if (unlikely(co->index < 0 || co->index >= PIC32_MAX_UARTS))
		return -ENODEV;

	sport = pic32_sports[co->index];
	if (!sport)
		return -ENODEV;
	port = pic32_get_port(sport);

	ret = pic32_enable_clock(sport);
	if (ret)
		return ret;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct uart_driver pic32_uart_driver;
static struct console pic32_console = {
	.name		= PIC32_SDEV_NAME,
	.write		= pic32_console_write,
	.device		= uart_console_device,
	.setup		= pic32_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &pic32_uart_driver,
};
#define PIC32_SCONSOLE (&pic32_console)

static int __init pic32_console_init(void)
{
	register_console(&pic32_console);
	return 0;
}
console_initcall(pic32_console_init);

static inline bool is_pic32_console_port(struct uart_port *port)
{
	return (port->cons && port->cons->index == port->line);
}

/*
 * Late console initialization.
 */
static int __init pic32_late_console_init(void)
{
	if (!(pic32_console.flags & CON_ENABLED))
		register_console(&pic32_console);

	return 0;
}

core_initcall(pic32_late_console_init);

#else
#define PIC32_SCONSOLE NULL
#endif

static struct uart_driver pic32_uart_driver = {
	.owner			= THIS_MODULE,
	.driver_name		= PIC32_DEV_NAME,
	.dev_name		= PIC32_SDEV_NAME,
	.major			= PIC32_SDEV_MAJOR,
	.minor			= PIC32_SDEV_MINOR,
	.nr			= PIC32_MAX_UARTS,
	.cons			= PIC32_SCONSOLE,
};

static int pic32_uart_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct pic32_sport *sport;
	int uart_idx = 0;
	struct resource *res_mem;
	struct uart_port *port;
	int ret = 0;

	uart_idx = of_alias_get_id(np, "serial");
	if (uart_idx < 0 || uart_idx >= PIC32_MAX_UARTS)
		return -EINVAL;

	res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res_mem)
		return -EINVAL;

	sport = devm_kzalloc(&pdev->dev, sizeof(*sport), GFP_KERNEL);
	if (!sport)
		return -ENOMEM;

	sport->idx		= uart_idx;
	sport->irq_fault	= irq_of_parse_and_map(np, 0);
	sport->irqflags_fault	= IRQF_NO_THREAD;
	sport->irq_rx		= irq_of_parse_and_map(np, 1);
	sport->irqflags_rx	= IRQF_NO_THREAD;
	sport->irq_tx		= irq_of_parse_and_map(np, 2);
	sport->irqflags_tx	= IRQF_NO_THREAD;
	sport->clk		= devm_clk_get(&pdev->dev, NULL);
	sport->cts_gpio		= -EINVAL;
	sport->dev		= &pdev->dev;

	ret = pic32_enable_clock(sport);
	if (ret) {
		dev_err(&pdev->dev, "clk enable ?\n");
		goto err;
	}

	sport->hw_flow_ctrl = of_property_read_bool(np,
					"microchip,uart-has-rtscts");
	if (!sport->hw_flow_ctrl)
		goto uart_no_flow_ctrl;

	/* Hardware flow control: gpios
	 * !Note: Basically, CTS is needed for reading the status.
	 */
	sport->cts_gpio = of_get_named_gpio(np, "cts-gpios", 0);
	if (gpio_is_valid(sport->cts_gpio)) {
		ret = devm_gpio_request(sport->dev,
					sport->cts_gpio, "CTS");
		if (ret) {
			dev_err(&pdev->dev,
				"error requesting CTS GPIO\n");
			goto err_disable_clk;
		}

		ret = gpio_direction_input(sport->cts_gpio);
		if (ret) {
			dev_err(&pdev->dev, "error setting CTS GPIO\n");
			goto err_disable_clk;
		}
	}

uart_no_flow_ctrl:
	pic32_sports[uart_idx] = sport;
	port = &sport->port;
	memset(port, 0, sizeof(*port));
	port->iotype	= UPIO_MEM;
	port->mapbase	= res_mem->start;
	port->ops	= &pic32_uart_ops;
	port->flags	= UPF_BOOT_AUTOCONF;
	port->dev	= &pdev->dev;
	port->fifosize	= PIC32_UART_TX_FIFO_DEPTH;
	port->uartclk	= clk_get_rate(sport->clk);
	port->line	= uart_idx;

	ret = uart_add_one_port(&pic32_uart_driver, port);
	if (ret) {
		port->membase = 0;
		dev_err(port->dev, "%s: uart add port error!\n", __func__);
		goto err_disable_clk;
	}

#ifdef CONFIG_SERIAL_PIC32_CONSOLE
	if (is_pic32_console_port(port) &&
	    (pic32_console.flags & CON_ENABLED)) {
		/* The peripheral clock has been enabled by console_setup,
		 * so disable it till the port is used.
		 */
		pic32_disable_clock(sport);
	}
#endif

	platform_set_drvdata(pdev, port);

	dev_info(&pdev->dev, "%s: uart(%d) driver initialized.\n",
		 __func__, uart_idx);
	ret = 0;

err_disable_clk:
	/* disable clock till the port is used. */
	pic32_disable_clock(sport);
err:
	/* automatic unroll of sport and gpios */
	return ret;
}

static int pic32_uart_remove(struct platform_device *pdev)
{
	struct uart_port *port = platform_get_drvdata(pdev);
	struct pic32_sport *sport = to_pic32_sport(port);

	uart_remove_one_port(&pic32_uart_driver, port);
	pic32_disable_clock(sport);
	platform_set_drvdata(pdev, NULL);
	pic32_sports[sport->idx] = NULL;

	/* automatic unroll of sport and gpios */
	return 0;
}

static const struct of_device_id pic32_serial_dt_ids[] = {
	{ .compatible = "microchip,pic32-usart" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, pic32_serial_dt_ids);

static struct platform_driver pic32_uart_platform_driver = {
	.probe		= pic32_uart_probe,
	.remove		= pic32_uart_remove,
	.driver		= {
		.name	= PIC32_DEV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(pic32_serial_dt_ids),
	},
};

static int __init pic32_uart_init(void)
{
	int ret;

	ret = uart_register_driver(&pic32_uart_driver);
	if (ret) {
		pr_err("failed to register %s:%d\n",
		       pic32_uart_driver.driver_name, ret);
		return ret;
	}

	ret = platform_driver_register(&pic32_uart_platform_driver);
	if (ret) {
		pr_err("fail to register pic32 uart\n");
		uart_unregister_driver(&pic32_uart_driver);
	}

	return ret;
}
arch_initcall(pic32_uart_init);

static void __exit pic32_uart_exit(void)
{
#ifdef CONFIG_SERIAL_PIC32_CONSOLE
	unregister_console(&pic32_console);
#endif
	platform_driver_unregister(&pic32_uart_platform_driver);
	uart_unregister_driver(&pic32_uart_driver);
}
module_exit(pic32_uart_exit);

MODULE_AUTHOR("Steve Scott <steve.scott@microchip.com>");
MODULE_DESCRIPTION("Microchip PIC32 integrated serial port driver");
MODULE_LICENSE("GPL v2");
