/*
 * PIC32 Integrated Serial Driver.
 *
 * Copyright (C) 2015 Microchip Technology, Inc.
 *
 * Authors:
 *   Sorin-Andrei Pistirica <andrei.pistirica@microchip.com>
 *
 * Licensed under GPLv2 or later.
 */
#ifndef __DT_PIC32_UART_H__
#define __DT_PIC32_UART_H__

#define PIC32_UART_DFLT_BRATE		(9600)
#define PIC32_UART_TX_FIFO_DEPTH	(8)
#define PIC32_UART_RX_FIFO_DEPTH	(8)

struct pic32_console_opt {
	int baud;
	int parity;
	int bits;
	int flow;
};
/* struct pic32_sport - pic32 serial port descriptor
 * @port: uart port descriptor
 * @idx: port index
 * @irq_fault: virtual fault interrupt number
 * @irqflags_fault: flags related to fault irq
 * @irq_fault_name: irq fault name
 * @irq_rx: virtual rx interrupt number
 * @irqflags_rx: flags related to rx irq
 * @irq_rx_name: irq rx name
 * @irq_tx: virtual tx interrupt number
 * @irqflags_tx: : flags related to tx irq
 * @irq_tx_name: irq tx name
 * @cts_gpio: clear to send gpio
 * @rts_gpio: ready to sent gpio
 * @dev: device descriptor
 **/
struct pic32_sport {
	struct uart_port port;
	struct pic32_console_opt opt;
	int idx;

	int irq_fault;
	int irqflags_fault;
	const char *irq_fault_name;
	int irq_rx;
	int irqflags_rx;
	const char *irq_rx_name;
	int irq_tx;
	int irqflags_tx;
	const char *irq_tx_name;
	u8 enable_tx_irq;

	bool hw_flow_ctrl;
	int cts_gpio;

	int ref_clk;
	struct clk *clk;

	struct device *dev;
};
#define to_pic32_sport(c) container_of(c, struct pic32_sport, port)
#define pic32_get_port(sport) (&sport->port)
#define pic32_get_opt(sport) (&sport->opt)
#define tx_irq_enabled(sport) (sport->enable_tx_irq)

struct pic32_reg {
	u32 val;
	u32 clr;
	u32 set;
	u32 inv;
} __packed;
#define PIC32_REGS 4
#define PIC32_REG_SIZE 4

enum pic32_uart_regs {
	PIC32_UART_UNKNOWN	= 0,
	PIC32_UART_MODE		= 1,
	PIC32_UART_STA		= 2,
	PIC32_UART_TX		= 3,
	PIC32_UART_RX		= 4,
	PIC32_UART_BRG		= 5,

	/* add above this line */
	PIC32_UART_LAST
};

/* uart register offsets */
static u32 pic32_uart_lookup_reg[PIC32_UART_LAST] = {
	[PIC32_UART_MODE]	= 0 * PIC32_REGS * PIC32_REG_SIZE,
	[PIC32_UART_STA]	= 1 * PIC32_REGS * PIC32_REG_SIZE,
	[PIC32_UART_TX]		= 2 * PIC32_REGS * PIC32_REG_SIZE,
	[PIC32_UART_RX]		= 3 * PIC32_REGS * PIC32_REG_SIZE,
	[PIC32_UART_BRG]	= 4 * PIC32_REGS * PIC32_REG_SIZE,
};

static inline void __iomem *pic32_uart_get_reg(struct pic32_sport *sport,
					       enum pic32_uart_regs reg)
{
	struct uart_port *port = pic32_get_port(sport);

	return port->membase + pic32_uart_lookup_reg[reg];
}

static inline u32 pic32_uart_rval(struct pic32_sport *sport,
				  enum pic32_uart_regs reg)
{
	void __iomem *addr = pic32_uart_get_reg(sport, reg);
	struct pic32_reg __iomem *reg_addr = (struct pic32_reg __iomem *)addr;

	return readl(&reg_addr->val);
}

static inline void pic32_uart_rset(u32 val,
				   struct pic32_sport *sport,
				   enum pic32_uart_regs reg)
{
	void __iomem *addr = pic32_uart_get_reg(sport, reg);
	struct pic32_reg __iomem *reg_addr = (struct pic32_reg __iomem *)addr;

	writel(val, &reg_addr->set);
}

static inline void pic32_uart_rclr(u32 val,
				   struct pic32_sport *sport,
				   enum pic32_uart_regs reg)
{
	void __iomem *addr = pic32_uart_get_reg(sport, reg);
	struct pic32_reg __iomem *reg_addr = (struct pic32_reg __iomem *)addr;

	writel(val, &reg_addr->clr);
}

static inline void pic32_uart_rinv(u32 val,
				   struct pic32_sport *sport,
				   enum pic32_uart_regs reg)
{
	void __iomem *addr = pic32_uart_get_reg(sport, reg);
	struct pic32_reg __iomem *reg_addr = (struct pic32_reg __iomem *)addr;

	writel(val, &reg_addr->inv);
}

static inline void pic32_uart_write(u32 val,
				    struct pic32_sport *sport,
				    enum pic32_uart_regs reg)
{
	void __iomem *addr = pic32_uart_get_reg(sport, reg);

	writel(val, addr);
}

static inline u32 pic32_uart_read(struct pic32_sport *sport,
				  enum pic32_uart_regs reg)
{
	void __iomem *addr = pic32_uart_get_reg(sport, reg);

	return readl(addr);
}

/* pic32 uart mode register bits */
#define PIC32_UART_MODE_ON        (1 << 15)
#define PIC32_UART_MODE_FRZ       (1 << 14)
#define PIC32_UART_MODE_SIDL      (1 << 13)
#define PIC32_UART_MODE_IREN      (1 << 12)
#define PIC32_UART_MODE_RTSMD     (1 << 11)
#define PIC32_UART_MODE_RESV1     (1 << 10)
#define PIC32_UART_MODE_UEN1      (1 << 9)
#define PIC32_UART_MODE_UEN0      (1 << 8)
#define PIC32_UART_MODE_WAKE      (1 << 7)
#define PIC32_UART_MODE_LPBK      (1 << 6)
#define PIC32_UART_MODE_ABAUD     (1 << 5)
#define PIC32_UART_MODE_RXINV     (1 << 4)
#define PIC32_UART_MODE_BRGH      (1 << 3)
#define PIC32_UART_MODE_PDSEL1    (1 << 2)
#define PIC32_UART_MODE_PDSEL0    (1 << 1)
#define PIC32_UART_MODE_STSEL     (1 << 0)

/* pic32 uart status register bits */
#define PIC32_UART_STA_UTXISEL1   (1 << 15)
#define PIC32_UART_STA_UTXISEL0   (1 << 14)
#define PIC32_UART_STA_UTXINV     (1 << 13)
#define PIC32_UART_STA_URXEN      (1 << 12)
#define PIC32_UART_STA_UTXBRK     (1 << 11)
#define PIC32_UART_STA_UTXEN      (1 << 10)
#define PIC32_UART_STA_UTXBF      (1 << 9)
#define PIC32_UART_STA_TRMT       (1 << 8)
#define PIC32_UART_STA_URXISEL1   (1 << 7)
#define PIC32_UART_STA_URXISEL0   (1 << 6)
#define PIC32_UART_STA_ADDEN      (1 << 5)
#define PIC32_UART_STA_RIDLE      (1 << 4)
#define PIC32_UART_STA_PERR       (1 << 3)
#define PIC32_UART_STA_FERR       (1 << 2)
#define PIC32_UART_STA_OERR       (1 << 1)
#define PIC32_UART_STA_URXDA      (1 << 0)

#endif /* __DT_PIC32_UART_H__ */
