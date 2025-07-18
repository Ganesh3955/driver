// SPDX-License-Identifier: GPL-2.0
/*
 * mfocserial.c: MFOC Serial port driver
 *
 * Copyright (C) 2024 Marcus Folkesson <marcus.folkesson@gmail.com>
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/bitfield.h>
#include <linux/console.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>

#define PORT_MFOCSERIAL		0x57
#define MSERIAL_NAME		"ttyMFOC"
#define MSERIAL_MAJOR		204
#define MSERIAL_MINOR		187
#define MSERIAL_NR_UARTS	2

#define MSERIAL_POLLRATE	(50000000)
#define MSERIAL_CLOCK		(12000000)
#define MSERIAL_BR_MIN		(MSERIAL_CLOCK / (8 * 0x3ff))
#define MSERIAL_BR_MAX		(MSERIAL_CLOCK / (8 * 0x001))

#define UART_AUTOSUSPEND_TIMEOUT	3000	/* ms */

/* ---------------------------------------------------------------------
 * Register definitions
 */

#define MSERIAL_REGION		10
#define MSERIAL_CONSTAT		0x00
#define MSERIAL_TXDATA		0x04
#define MSERIAL_RXDATA		0x08
#define MSERIAL_BR			0x0c

#define MSERIAL_CONSTAT_ENUART	0x0001
#define MSERIAL_CONSTAT_PAR		0x0002
#define MSERIAL_CONSTAT_ODD		0x0004
#define MSERIAL_CONSTAT_RXEMPTY	0x0100
#define MSERIAL_CONSTAT_RXFULL	0x0200
#define MSERIAL_CONSTAT_OVR		0x0800
#define MSERIAL_CONSTAT_FERR	0x1000
#define MSERIAL_CONSTAT_PARERR	0x2000
#define MSERIAL_CONSTAT_TXFULL	0x4000
#define MSERIAL_CONSTAT_TXC		0x8000
#define MSERIAL_CONSTAT_ERRMASK	\
	(MSERIAL_CONSTAT_OVR | MSERIAL_CONSTAT_FERR | MSERIAL_CONSTAT_PARERR)


struct mserial_data {
	struct uart_port	*port;
	unsigned int baud;
	tcflag_t cflags;
	struct hrtimer		timer;
	ktime_t				pollrate;
};


static struct uart_port mserial_ports[MSERIAL_NR_UARTS];
static struct uart_driver mserial_uart_driver;

/* ---------------------------------------------------------------------
 * Core UART driver operations
 */


static int mserial_receive(struct uart_port *port, int stat)
{
	struct tty_port *tport = &port->state->port;
	unsigned char ch = 0;
	char flag = TTY_NORMAL;

	if (!(stat & (MSERIAL_CONSTAT_OVR | MSERIAL_CONSTAT_FERR)) &&
		(stat & MSERIAL_CONSTAT_RXEMPTY))
		return 0;

	if (!(stat & MSERIAL_CONSTAT_RXEMPTY)) {
		port->icount.rx++;
		ch = ioread8(port->membase + MSERIAL_RXDATA);
	}

	port->icount.parity  += (stat & MSERIAL_CONSTAT_PARERR) ? 1 : 0;
	port->icount.overrun += (stat & MSERIAL_CONSTAT_OVR) ? 1 : 0;
	port->icount.frame   += (stat & MSERIAL_CONSTAT_FERR) ? 1 : 0;

	if (!(stat & MSERIAL_CONSTAT_RXEMPTY))
		tty_insert_flip_char(tport, ch, flag);
	if (stat & MSERIAL_CONSTAT_OVR)
		tty_insert_flip_char(tport, 0, TTY_OVERRUN);
	if (stat & MSERIAL_CONSTAT_FERR)
		tty_insert_flip_char(tport, 0, TTY_FRAME);

	return 1;
}


static int mserial_transmit(struct uart_port *port, int stat)
{
	struct circ_buf *xmit  = &port->state->xmit;

	if (stat & MSERIAL_CONSTAT_TXFULL)
		return 0;


	if (port->x_char) {
		iowrite8(port->x_char, port->membase + MSERIAL_TXDATA);
		port->x_char = 0;
		port->icount.tx++;
		return 1;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(port))
		return 0;

	iowrite8(xmit->buf[xmit->tail], port->membase + MSERIAL_TXDATA);
	xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE-1);
	port->icount.tx++;

	/* wake up */
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	return 1;
}

static enum hrtimer_restart mserial_poll(struct hrtimer *timer)
{
	struct mserial_data *ss = container_of(timer, struct mserial_data, timer);
	struct uart_port *port = ss->port;
	int rx_or_tx, n = 0;
	u16 cs;

	do {
		cs = ioread32(port->membase + MSERIAL_CONSTAT);
		iowrite32(cs & ~MSERIAL_CONSTAT_ERRMASK, port->membase + MSERIAL_CONSTAT);

		rx_or_tx  = mserial_receive(port, cs);
		rx_or_tx |= mserial_transmit(port, cs);
		n++;
	} while (rx_or_tx);

	if (n > 1)
		tty_flip_buffer_push(&port->state->port);

	hrtimer_start(&ss->timer, ss->pollrate, HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static unsigned int mserial_tx_empty(struct uart_port *port)
{
	unsigned long flags;
	unsigned int ret;

	spin_lock_irqsave(&port->lock, flags);
	ret = ioread32(port->membase + MSERIAL_CONSTAT);
	spin_unlock_irqrestore(&port->lock, flags);

	return (ret & MSERIAL_CONSTAT_TXC) ? TIOCSER_TEMT : 0;
}

static unsigned int mserial_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void mserial_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* N/A */
}

static void mserial_stop_tx(struct uart_port *port)
{
}

static void mserial_start_tx(struct uart_port *port)
{
	mserial_transmit(port, ioread32(port->membase + MSERIAL_CONSTAT));
}

static void mserial_stop_rx(struct uart_port *port)
{
	/* don't forward any more data (like !CREAD) */
	port->ignore_status_mask = MSERIAL_CONSTAT_ERRMASK;
}

static void mserial_break_ctl(struct uart_port *port, int ctl)
{
	/* N/A */
}

static int mserial_startup(struct uart_port *port)
{
	struct mserial_data *pdata = port->private_data;

	hrtimer_start(&pdata->timer, pdata->pollrate, HRTIMER_MODE_REL);

	iowrite32(ioread32(port->membase + MSERIAL_CONSTAT) | MSERIAL_CONSTAT_ENUART,
			port->membase + MSERIAL_CONSTAT);
	return 0;
}

static void mserial_shutdown(struct uart_port *port)
{
	struct mserial_data *pdata = port->private_data;

	iowrite32(ioread32(port->membase + MSERIAL_CONSTAT) & ~MSERIAL_CONSTAT_ENUART,
			port->membase + MSERIAL_CONSTAT);

	hrtimer_cancel(&pdata->timer);
}
/*
static void mserial_set_termios(struct uart_port *port,
			      struct ktermios *termios,
			      const struct ktermios *old)
{
	unsigned long flags;
	unsigned int baud = 0;
	u16 div;

	spin_lock_irqsave(&port->lock, flags);

	baud = uart_get_baud_rate(
			port, termios, old, MSERIAL_BR_MIN, MSERIAL_BR_MAX);

	if (baud)	{
		div = MSERIAL_CLOCK / (8 * baud);
		iowrite32(div, port->membase + MSERIAL_BR);
		uart_update_timeout(port, termios->c_cflag, baud);
	}
	spin_unlock_irqrestore(&port->lock, flags);

}
*/
static const char *mserial_type(struct uart_port *port)
{
	return port->type == PORT_MFOCSERIAL ? "mfocserial" : NULL;
}

static void mserial_release_port(struct uart_port *port)
{
	release_mem_region(port->mapbase, MSERIAL_REGION);
	iounmap(port->membase);
	port->membase = NULL;
}

static int mserial_request_port(struct uart_port *port)
{
	pr_debug("mserial console: port=%p; port->mapbase=%llx\n",
		 port, (unsigned long long) port->mapbase);

	if (!request_mem_region(port->mapbase, MSERIAL_REGION, "mserial")) {
		dev_err(port->dev, "Memory region busy\n");
		return -EBUSY;
	}

	port->membase = ioremap(port->mapbase, MSERIAL_REGION);
	if (!port->membase) {
		dev_err(port->dev, "Unable to map registers\n");
		release_mem_region(port->mapbase, MSERIAL_REGION);
		return -EBUSY;
	}
	return 0;
}

static void mserial_config_port(struct uart_port *port, int flags)
{
	if (!mserial_request_port(port))
		port->type = PORT_MFOCSERIAL;
}

static int mserial_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	/* we don't want the core code to modify any port params */
	return -EINVAL;
}

static const struct uart_ops mserial_ops = {
	.tx_empty	= mserial_tx_empty,
	.set_mctrl	= mserial_set_mctrl,
	.get_mctrl	= mserial_get_mctrl,
	.stop_tx	= mserial_stop_tx,
	.start_tx	= mserial_start_tx,
	.stop_rx	= mserial_stop_rx,
	.break_ctl	= mserial_break_ctl,
	.startup	= mserial_startup,
	.shutdown	= mserial_shutdown,
	.type		= mserial_type,
	.release_port	= mserial_release_port,
	.request_port	= mserial_request_port,
	.config_port	= mserial_config_port,
	.verify_port	= mserial_verify_port,
};

static struct uart_driver mserial_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= "mfocserial",
	.dev_name	= MSERIAL_NAME,
	.major		= MSERIAL_MAJOR,
	.minor		= MSERIAL_MINOR,
	.nr		= MSERIAL_NR_UARTS,
};

/* ---------------------------------------------------------------------
 * Port assignment functions (mapping devices to uart_port structures)
 */

static int mserial_assign(struct device *dev, int id, phys_addr_t base,
			  struct mserial_data *pdata)
{
	struct uart_port *port;
	int rc;

	/* if id = -1; then scan for a free id and use that */
	if (id < 0) {
		for (id = 0; id < MSERIAL_NR_UARTS; id++)
			if (mserial_ports[id].mapbase == 0)
				break;
	}
	if (id < 0 || id >= MSERIAL_NR_UARTS) {
		dev_err(dev, "%s%i too large\n", MSERIAL_NAME, id);
		return -EINVAL;
	}

	if ((mserial_ports[id].mapbase) && (mserial_ports[id].mapbase != base)) {
		dev_err(dev, "cannot assign to %s%i; it is already in use\n",
			MSERIAL_NAME, id);
		return -EBUSY;
	}

	port = &mserial_ports[id];

	spin_lock_init(&port->lock);
	port->fifosize = 16;
	port->regshift = 2;
	port->iotype = UPIO_MEM;
	port->iobase = 1; /* mark port in use */
	port->mapbase = base;
	port->membase = NULL;
	port->ops = &mserial_ops;
	port->flags = UPF_BOOT_AUTOCONF;
	port->dev = dev;
	port->type = PORT_UNKNOWN;
	port->line = id;
	port->private_data = pdata;
	pdata->port = port;

	dev_set_drvdata(dev, port);

	pdata->pollrate = ktime_set(0, MSERIAL_POLLRATE);
	hrtimer_init(&pdata->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pdata->timer.function = mserial_poll;

	/* Register the port */
	rc = uart_add_one_port(&mserial_uart_driver, port);
	if (rc) {
		dev_err(dev, "uart_add_one_port() failed; err=%i\n", rc);
		port->mapbase = 0;
		dev_set_drvdata(dev, NULL);
		return rc;
	}

	return 0;
}

static int mserial_release(struct device *dev)
{
	struct uart_port *port = dev_get_drvdata(dev);
	int rc = 0;

	if (port) {
		rc = uart_remove_one_port(&mserial_uart_driver, port);
		dev_set_drvdata(dev, NULL);
		port->mapbase = 0;
	}

	return rc;
}


/* ---------------------------------------------------------------------
 * Platform bus binding
 */


#if defined(CONFIG_OF)
/* Match table for of_platform binding */
static const struct of_device_id mserial_of_match[] = {
	{ .compatible = "mfoc,mfocserial", },
	{}
};
MODULE_DEVICE_TABLE(of, mserial_of_match);
#endif /* CONFIG_OF */

static int mserial_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct mserial_data *pdata;
	int ret;
	int id = pdev->id;

	pdata = devm_kzalloc(&pdev->dev, sizeof(struct mserial_data),
			     GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	if (IS_ENABLED(CONFIG_OF)) {
		const char *prop;
		struct device_node *np = pdev->dev.of_node;

		prop = "port-number";
		ret = of_property_read_u32(np, prop, &id);
		if (ret && ret != -EINVAL)
of_err:
			return dev_err_probe(&pdev->dev, ret,
					     "could not read %s\n", prop);

		prop = "current-speed";
		ret = of_property_read_u32(np, prop, &pdata->baud);
		if (ret)
			goto of_err;
	} else {
		pdata->baud = 9600;
		pdata->cflags = CS8;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	if (!mserial_uart_driver.state) {
		dev_dbg(&pdev->dev, "mfocserial: calling uart_register_driver()\n");
		ret = uart_register_driver(&mserial_uart_driver);
		if (ret < 0) {
			dev_err(&pdev->dev, "Failed to register driver\n");
			return ret;
		}
	}

	ret = mserial_assign(&pdev->dev, id, res->start, pdata);

	return ret;
}

static int mserial_remove(struct platform_device *pdev)
{
	return mserial_release(&pdev->dev);
}

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:mfocserial");

static struct platform_driver mserial_platform_driver = {
	.probe = mserial_probe,
	.remove = mserial_remove,
	.driver = {
		.name  = "mfocserial",
		.of_match_table = of_match_ptr(mserial_of_match),
	},
};

static int __init mserial_init(void)
{

	pr_debug("mfocserial: calling platform_driver_register()\n");
	return platform_driver_register(&mserial_platform_driver);
}

static void __exit mserial_exit(void)
{
	platform_driver_unregister(&mserial_platform_driver);
	if (mserial_uart_driver.state)
		uart_unregister_driver(&mserial_uart_driver);
}

module_init(mserial_init);
module_exit(mserial_exit);

MODULE_AUTHOR("Marcus Folkesson <marcus.folkesson@gmail.com>");
MODULE_DESCRIPTION("MFOC Serial Driver");
MODULE_LICENSE("GPL");
