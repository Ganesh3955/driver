// SPDX-License-Identifier: GPL-2.0-only
//
// Driver for Cadence QSPI Controller
//
// Copyright Altera Corporation (C) 2012-2014. All rights reserved.
// Copyright Intel Corporation (C) 2019-2020. All rights reserved.
// Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/firmware/xlnx-zynqmp.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/sched.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>
#include <linux/timer.h>
#include <linux/workqueue.h>

#define CQSPI_NAME			"cadence-qspi"
#define CQSPI_MAX_CHIPSELECT		16

/* Quirks */
#define CQSPI_NEEDS_WR_DELAY		BIT(0)
#define CQSPI_DISABLE_DAC_MODE		BIT(1)
#define CQSPI_SUPPORT_EXTERNAL_DMA	BIT(2)

/* Capabilities */
#define CQSPI_SUPPORTS_OCTAL		BIT(0)

struct cqspi_st;

struct cqspi_flash_pdata {
	struct cqspi_st	*cqspi;
	u32		clk_rate;
	u32		read_delay;
	u32		tshsl_ns;
	u32		tsd2d_ns;
	u32		tchsh_ns;
	u32		tslch_ns;
	u8		inst_width;
	u8		addr_width;
	u8		data_width;
	bool		dtr;
	u8		cs;
};

struct cqspi_st {
	struct platform_device	*pdev;

	struct clk		*clk;
	unsigned int		sclk;

	void __iomem		*iobase;
	void __iomem		*ahb_base;
	resource_size_t		ahb_size;
	struct completion	transfer_complete;

	struct dma_chan		*rx_chan;
	struct completion	rx_dma_complete;
	dma_addr_t		mmap_phys_base;

	int			current_cs;
	unsigned long		master_ref_clk_hz;
	bool			is_decoded_cs;
	u32			fifo_depth;
	u32			fifo_width;
	u32			num_chipselect;
	bool			rclk_en;
	u32			trigger_address;
	u32			wr_delay;
	bool			use_direct_mode;
	struct cqspi_flash_pdata f_pdata[CQSPI_MAX_CHIPSELECT];
	bool			use_dma_read;
	u32			pd_dev_id;
	bool			extra_dummy;
	bool			clk_tuned;
	u8			dll_mode;
	struct completion	tuning_complete;
	struct spi_mem_op	tuning_op;
	int			gpio;
};

struct cqspi_driver_platdata {
	u32 hwcaps_mask;
	u8 quirks;
	int (*indirect_read_dma)(struct cqspi_flash_pdata *f_pdata,
				 u_char *rxbuf, loff_t from_addr, size_t n_rx);
	u32 (*get_dma_status)(struct cqspi_st *cqspi);
	int (*device_reset)(struct cqspi_st *cqspi, u8 reset_type);
};

/* Operation timeout value */
#define CQSPI_TIMEOUT_MS			500
#define CQSPI_READ_TIMEOUT_MS			10
#define CQSPI_TUNING_TIMEOUT_MS			5000
#define CQSPI_TUNING_PERIODICITY_MS		300000

#define CQSPI_DUMMY_CLKS_PER_BYTE		8
#define CQSPI_DUMMY_BYTES_MAX			4
#define CQSPI_DUMMY_CLKS_MAX			31

#define CQSPI_STIG_DATA_LEN_MAX			8

/* Register map */
#define CQSPI_REG_CONFIG			0x00
#define CQSPI_REG_CONFIG_ENABLE_MASK		BIT(0)
#define CQSPI_REG_CONFIG_PHY_ENABLE_MASK	BIT(3)
#define CQSPI_REG_CONFIG_ENB_DIR_ACC_CTRL	BIT(7)
#define CQSPI_REG_CONFIG_DECODE_MASK		BIT(9)
#define CQSPI_REG_CONFIG_CHIPSELECT_LSB		10
#define CQSPI_REG_CONFIG_DMA_MASK		BIT(15)
#define CQSPI_REG_CONFIG_BAUD_LSB		19
#define CQSPI_REG_CONFIG_DTR_PROTO		BIT(24)
#define CQSPI_REG_CONFIG_DUAL_OPCODE		BIT(30)
#define CQSPI_REG_CONFIG_IDLE_LSB		31
#define CQSPI_REG_CONFIG_CHIPSELECT_MASK	0xF
#define CQSPI_REG_CONFIG_BAUD_MASK		0xF

#define CQSPI_REG_RD_INSTR			0x04
#define CQSPI_REG_RD_INSTR_OPCODE_LSB		0
#define CQSPI_REG_RD_INSTR_TYPE_INSTR_LSB	8
#define CQSPI_REG_RD_INSTR_TYPE_ADDR_LSB	12
#define CQSPI_REG_RD_INSTR_TYPE_DATA_LSB	16
#define CQSPI_REG_RD_INSTR_MODE_EN_LSB		20
#define CQSPI_REG_RD_INSTR_DUMMY_LSB		24
#define CQSPI_REG_RD_INSTR_TYPE_INSTR_MASK	0x3
#define CQSPI_REG_RD_INSTR_TYPE_ADDR_MASK	0x3
#define CQSPI_REG_RD_INSTR_TYPE_DATA_MASK	0x3
#define CQSPI_REG_RD_INSTR_DUMMY_MASK		0x1F

#define CQSPI_REG_WR_INSTR			0x08
#define CQSPI_REG_WR_INSTR_OPCODE_LSB		0
#define CQSPI_REG_WR_INSTR_TYPE_ADDR_LSB	12
#define CQSPI_REG_WR_INSTR_TYPE_DATA_LSB	16

#define CQSPI_REG_DELAY				0x0C
#define CQSPI_REG_DELAY_TSLCH_LSB		0
#define CQSPI_REG_DELAY_TCHSH_LSB		8
#define CQSPI_REG_DELAY_TSD2D_LSB		16
#define CQSPI_REG_DELAY_TSHSL_LSB		24
#define CQSPI_REG_DELAY_TSLCH_MASK		0xFF
#define CQSPI_REG_DELAY_TCHSH_MASK		0xFF
#define CQSPI_REG_DELAY_TSD2D_MASK		0xFF
#define CQSPI_REG_DELAY_TSHSL_MASK		0xFF

#define CQSPI_REG_READCAPTURE			0x10
#define CQSPI_REG_READCAPTURE_BYPASS_LSB	0
#define CQSPI_REG_READCAPTURE_DELAY_LSB		1
#define CQSPI_REG_READCAPTURE_DELAY_MASK	0xF
#define CQSPI_REG_READCAPTURE_DQS_ENABLE	BIT(8)

#define CQSPI_REG_SIZE				0x14
#define CQSPI_REG_SIZE_ADDRESS_LSB		0
#define CQSPI_REG_SIZE_PAGE_LSB			4
#define CQSPI_REG_SIZE_BLOCK_LSB		16
#define CQSPI_REG_SIZE_ADDRESS_MASK		0xF
#define CQSPI_REG_SIZE_PAGE_MASK		0xFFF
#define CQSPI_REG_SIZE_BLOCK_MASK		0x3F

#define CQSPI_REG_SRAMPARTITION			0x18
#define CQSPI_REG_INDIRECTTRIGGER		0x1C

#define CQSPI_REG_DMA				0x20
#define CQSPI_REG_DMA_SINGLE_LSB		0
#define CQSPI_REG_DMA_BURST_LSB			8
#define CQSPI_REG_DMA_SINGLE_MASK		0xFF
#define CQSPI_REG_DMA_BURST_MASK		0xFF

#define CQSPI_REG_REMAP				0x24
#define CQSPI_REG_MODE_BIT			0x28

#define CQSPI_REG_SDRAMLEVEL			0x2C
#define CQSPI_REG_SDRAMLEVEL_RD_LSB		0
#define CQSPI_REG_SDRAMLEVEL_WR_LSB		16
#define CQSPI_REG_SDRAMLEVEL_RD_MASK		0xFFFF
#define CQSPI_REG_SDRAMLEVEL_WR_MASK		0xFFFF

#define CQSPI_REG_WR_COMPLETION_CTRL		0x38
#define CQSPI_REG_WR_DISABLE_AUTO_POLL		BIT(14)
#define CQSPI_REG_WRCOMPLETION_POLLCNT_MASK	0xFF0000
#define CQSPI_REG_WRCOMPLETION_POLLCNY_LSB	16

#define CQSPI_REG_IRQSTATUS			0x40
#define CQSPI_REG_IRQMASK			0x44
#define CQSPI_REG_VERSAL_ECO			0x48

#define CQSPI_REG_INDIRECTRD			0x60
#define CQSPI_REG_INDIRECTRD_START_MASK		BIT(0)
#define CQSPI_REG_INDIRECTRD_CANCEL_MASK	BIT(1)
#define CQSPI_REG_INDIRECTRD_DONE_MASK		BIT(5)

#define CQSPI_REG_INDIRECTRDWATERMARK		0x64
#define CQSPI_REG_INDIRECTRDSTARTADDR		0x68
#define CQSPI_REG_INDIRECTRDBYTES		0x6C

#define CQSPI_REG_CMDCTRL			0x90
#define CQSPI_REG_CMDCTRL_EXECUTE_MASK		BIT(0)
#define CQSPI_REG_CMDCTRL_INPROGRESS_MASK	BIT(1)
#define CQSPI_REG_CMDCTRL_DUMMY_LSB		7
#define CQSPI_REG_CMDCTRL_WR_BYTES_LSB		12
#define CQSPI_REG_CMDCTRL_WR_EN_LSB		15
#define CQSPI_REG_CMDCTRL_ADD_BYTES_LSB		16
#define CQSPI_REG_CMDCTRL_ADDR_EN_LSB		19
#define CQSPI_REG_CMDCTRL_RD_BYTES_LSB		20
#define CQSPI_REG_CMDCTRL_RD_EN_LSB		23
#define CQSPI_REG_CMDCTRL_OPCODE_LSB		24
#define CQSPI_REG_CMDCTRL_WR_BYTES_MASK		0x7
#define CQSPI_REG_CMDCTRL_ADD_BYTES_MASK	0x3
#define CQSPI_REG_CMDCTRL_RD_BYTES_MASK		0x7
#define CQSPI_REG_CMDCTRL_DUMMY_MASK		0x1F

#define CQSPI_REG_INDIRECTWR			0x70
#define CQSPI_REG_INDIRECTWR_START_MASK		BIT(0)
#define CQSPI_REG_INDIRECTWR_CANCEL_MASK	BIT(1)
#define CQSPI_REG_INDIRECTWR_DONE_MASK		BIT(5)

#define CQSPI_REG_INDIRECTWRWATERMARK		0x74
#define CQSPI_REG_INDIRECTWRSTARTADDR		0x78
#define CQSPI_REG_INDIRECTWRBYTES		0x7C

#define CQSPI_REG_INDTRIG_ADDRRANGE		0x80

#define CQSPI_REG_CMDADDRESS			0x94
#define CQSPI_REG_CMDREADDATALOWER		0xA0
#define CQSPI_REG_CMDREADDATAUPPER		0xA4
#define CQSPI_REG_CMDWRITEDATALOWER		0xA8
#define CQSPI_REG_CMDWRITEDATAUPPER		0xAC

#define CQSPI_REG_POLLING_STATUS		0xB0
#define CQSPI_REG_POLLING_STATUS_DUMMY_LSB	16

#define CQSPI_REG_PHY_CONFIG			0xB4
#define CQSPI_REG_PHY_CONFIG_RESYNC_FLD_MASK	0x80000000
#define CQSPI_REG_PHY_CONFIG_RESET_FLD_MASK	0x40000000
#define CQSPI_REG_PHY_CONFIG_TX_DLL_DLY_LSB	16

#define CQSPI_REG_PHY_MASTER_CTRL		0xB8
#define CQSPI_REG_DLL_LOWER			0xBC
#define CQSPI_REG_DLL_LOWER_LPBK_LOCK_MASK	0x8000
#define CQSPI_REG_DLL_LOWER_DLL_LOCK_MASK	0x1

#define CQSPI_REG_DLL_OBSVBLE_UPPER		0xC0
#define CQSPI_REG_DLL_UPPER_RX_FLD_MASK		0x7F

#define CQSPI_REG_OP_EXT_LOWER			0xE0
#define CQSPI_REG_OP_EXT_READ_LSB		24
#define CQSPI_REG_OP_EXT_WRITE_LSB		16
#define CQSPI_REG_OP_EXT_STIG_LSB		0

#define CQSPI_REG_VERSAL_DMA_SRC_ADDR		0x1000

#define CQSPI_REG_VERSAL_DMA_DST_ADDR		0x1800
#define CQSPI_REG_VERSAL_DMA_DST_SIZE		0x1804

#define CQSPI_REG_VERSAL_DMA_DST_CTRL		0x180C

#define CQSPI_REG_VERSAL_DMA_DST_I_STS		0x1814
#define CQSPI_REG_VERSAL_DMA_DST_I_EN		0x1818
#define CQSPI_REG_VERSAL_DMA_DST_I_DIS		0x181C
#define CQSPI_REG_VERSAL_DMA_DST_DONE_MASK	BIT(1)
#define CQSPI_REG_VERSAL_DMA_DST_ALL_I_DIS_MASK	0xFE

#define CQSPI_REG_VERSAL_DMA_DST_ADDR_MSB	0x1828

#define CQSPI_REG_VERSAL_DMA_DST_CTRL_VAL	0xF43FFA00
#define CQSPI_REG_VERSAL_ADDRRANGE_WIDTH_VAL	0x6

/* Interrupt status bits */
#define CQSPI_REG_IRQ_MODE_ERR			BIT(0)
#define CQSPI_REG_IRQ_UNDERFLOW			BIT(1)
#define CQSPI_REG_IRQ_IND_COMP			BIT(2)
#define CQSPI_REG_IRQ_IND_RD_REJECT		BIT(3)
#define CQSPI_REG_IRQ_WR_PROTECTED_ERR		BIT(4)
#define CQSPI_REG_IRQ_ILLEGAL_AHB_ERR		BIT(5)
#define CQSPI_REG_IRQ_WATERMARK			BIT(6)
#define CQSPI_REG_IRQ_IND_SRAM_FULL		BIT(12)

#define CQSPI_IRQ_MASK_RD		(CQSPI_REG_IRQ_WATERMARK	| \
					 CQSPI_REG_IRQ_IND_SRAM_FULL	| \
					 CQSPI_REG_IRQ_IND_COMP)

#define CQSPI_IRQ_MASK_WR		(CQSPI_REG_IRQ_IND_COMP		| \
					 CQSPI_REG_IRQ_WATERMARK	| \
					 CQSPI_REG_IRQ_UNDERFLOW)

#define CQSPI_IRQ_STATUS_MASK		0x1FFFF
#define CQSPI_DMA_UNALIGN		0x3

#define CQSPI_REG_VERSAL_DMA_VAL		0x602
#define CQSPI_VERSAL_MIO_NODE_ID_12	0x14108027
#define CQSPI_RESET_TYPE_HWPIN		0
#define CQSPI_READ_ID_LEN		6
#define TERA_MACRO			1000000000000ULL
#define VERSAL_OSPI_RESET		0xc10402e
#define SILICON_VER_MASK		0xFF
#define SILICON_VER_1			0x10
#define CQSPI_DLL_MODE_MASTER		0
#define CQSPI_DLL_MODE_BYPASS		1
#define TAP_GRAN_SEL_MIN_FREQ		120000000
#define CQSPI_TX_TAP_MASTER		0x1E
#define CQSPI_MAX_DLL_TAPS		127

#define CQSPI_CS_LOWER			0
#define CQSPI_CS_UPPER			1

static unsigned int read_timeout_ms = CQSPI_READ_TIMEOUT_MS;
module_param(read_timeout_ms, uint, 0644);
MODULE_PARM_DESC(read_timeout_ms, "Read transfer timeout in msec");

static int cqspi_wait_for_bit(void __iomem *reg, const u32 mask, bool clr)
{
	u32 val;

	return readl_relaxed_poll_timeout(reg, val,
					  (((clr ? ~val : val) & mask) == mask),
					  10, CQSPI_TIMEOUT_MS * 1000);
}

static bool cqspi_is_idle(struct cqspi_st *cqspi)
{
	u32 reg = readl(cqspi->iobase + CQSPI_REG_CONFIG);

	return reg & (1UL << CQSPI_REG_CONFIG_IDLE_LSB);
}

static u32 cqspi_get_rd_sram_level(struct cqspi_st *cqspi)
{
	u32 reg = readl(cqspi->iobase + CQSPI_REG_SDRAMLEVEL);

	reg >>= CQSPI_REG_SDRAMLEVEL_RD_LSB;
	return reg & CQSPI_REG_SDRAMLEVEL_RD_MASK;
}

static u32 cqspi_get_versal_dma_status(struct cqspi_st *cqspi)
{
	u32 dma_status;

	dma_status = readl(cqspi->iobase +
					   CQSPI_REG_VERSAL_DMA_DST_I_STS);
	writel(dma_status, cqspi->iobase +
		   CQSPI_REG_VERSAL_DMA_DST_I_STS);

	return dma_status & CQSPI_REG_VERSAL_DMA_DST_DONE_MASK;
}

static irqreturn_t cqspi_irq_handler(int this_irq, void *dev)
{
	struct cqspi_st *cqspi = dev;
	unsigned int irq_status;
	struct device *device = &cqspi->pdev->dev;
	const struct cqspi_driver_platdata *ddata;

	ddata = of_device_get_match_data(device);

	/* Read interrupt status */
	irq_status = readl(cqspi->iobase + CQSPI_REG_IRQSTATUS);

	/* Clear interrupt */
	writel(irq_status, cqspi->iobase + CQSPI_REG_IRQSTATUS);

	if (cqspi->use_dma_read && ddata && ddata->get_dma_status) {
		if (ddata->get_dma_status(cqspi)) {
			complete(&cqspi->transfer_complete);
			return IRQ_HANDLED;
		}
	}

	irq_status &= CQSPI_IRQ_MASK_RD | CQSPI_IRQ_MASK_WR;

	if (irq_status)
		complete(&cqspi->transfer_complete);

	return IRQ_HANDLED;
}

static unsigned int cqspi_calc_rdreg(struct cqspi_flash_pdata *f_pdata)
{
	u32 rdreg = 0;

	rdreg |= f_pdata->inst_width << CQSPI_REG_RD_INSTR_TYPE_INSTR_LSB;
	rdreg |= f_pdata->addr_width << CQSPI_REG_RD_INSTR_TYPE_ADDR_LSB;
	rdreg |= f_pdata->data_width << CQSPI_REG_RD_INSTR_TYPE_DATA_LSB;

	return rdreg;
}

static unsigned int cqspi_calc_dummy(const struct spi_mem_op *op, bool dtr)
{
	unsigned int dummy_clk;

	if (!op->dummy.nbytes)
		return 0;

	dummy_clk = op->dummy.nbytes * (8 / op->dummy.buswidth);
	if (dtr)
		dummy_clk /= 2;

	return dummy_clk;
}

static int cqspi_set_protocol(struct cqspi_flash_pdata *f_pdata,
			      const struct spi_mem_op *op)
{
	/*
	 * For an op to be DTR, cmd phase along with every other non-empty
	 * phase should have dtr field set to 1. If an op phase has zero
	 * nbytes, ignore its dtr field; otherwise, check its dtr field.
	 */
	f_pdata->dtr = op->cmd.dtr &&
		       (!op->addr.nbytes || op->addr.dtr) &&
		       (!op->data.nbytes || op->data.dtr);

	f_pdata->inst_width = 0;
	if (op->cmd.buswidth)
		f_pdata->inst_width = ilog2(op->cmd.buswidth);

	f_pdata->addr_width = 0;
	if (op->addr.buswidth)
		f_pdata->addr_width = ilog2(op->addr.buswidth);

	f_pdata->data_width = 0;
	if (op->data.buswidth)
		f_pdata->data_width = ilog2(op->data.buswidth);

	/* Right now we only support 8-8-8 DTR mode. */
	if (f_pdata->dtr) {
		switch (op->cmd.buswidth) {
		case 0:
		case 8:
			break;
		default:
			return -EINVAL;
		}

		switch (op->addr.buswidth) {
		case 0:
		case 8:
			break;
		default:
			return -EINVAL;
		}

		switch (op->data.buswidth) {
		case 0:
		case 8:
			break;
		default:
			return -EINVAL;
		}
	}

	return 0;
}

static int cqspi_wait_idle(struct cqspi_st *cqspi)
{
	const unsigned int poll_idle_retry = 3;
	unsigned int count = 0;
	unsigned long timeout;

	timeout = jiffies + msecs_to_jiffies(CQSPI_TIMEOUT_MS);
	while (1) {
		/*
		 * Read few times in succession to ensure the controller
		 * is indeed idle, that is, the bit does not transition
		 * low again.
		 */
		if (cqspi_is_idle(cqspi))
			count++;
		else
			count = 0;

		if (count >= poll_idle_retry)
			return 0;

		if (time_after(jiffies, timeout)) {
			/* Timeout, in busy mode. */
			dev_err(&cqspi->pdev->dev,
				"QSPI is still busy after %dms timeout.\n",
				CQSPI_TIMEOUT_MS);
			return -ETIMEDOUT;
		}

		cpu_relax();
	}
}

static int cqspi_exec_flash_cmd(struct cqspi_st *cqspi, unsigned int reg)
{
	void __iomem *reg_base = cqspi->iobase;
	int ret;

	/* Write the CMDCTRL without start execution. */
	writel(reg, reg_base + CQSPI_REG_CMDCTRL);
	/* Start execute */
	reg |= CQSPI_REG_CMDCTRL_EXECUTE_MASK;
	writel(reg, reg_base + CQSPI_REG_CMDCTRL);

	/* Polling for completion. */
	ret = cqspi_wait_for_bit(reg_base + CQSPI_REG_CMDCTRL,
				 CQSPI_REG_CMDCTRL_INPROGRESS_MASK, 1);
	if (ret) {
		dev_err(&cqspi->pdev->dev,
			"Flash command execution timed out.\n");
		return ret;
	}

	/* Polling QSPI idle status. */
	return cqspi_wait_idle(cqspi);
}

static int cqspi_setup_opcode_ext(struct cqspi_flash_pdata *f_pdata,
				  const struct spi_mem_op *op,
				  unsigned int shift)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	void __iomem *reg_base = cqspi->iobase;
	unsigned int reg;
	u8 ext;

	if (op->cmd.nbytes != 2)
		return -EINVAL;

	/* Opcode extension is the LSB. */
	ext = op->cmd.opcode & 0xff;

	reg = readl(reg_base + CQSPI_REG_OP_EXT_LOWER);
	reg &= ~(0xff << shift);
	reg |= ext << shift;
	writel(reg, reg_base + CQSPI_REG_OP_EXT_LOWER);

	return 0;
}

static int cqspi_enable_dtr(struct cqspi_flash_pdata *f_pdata,
			    const struct spi_mem_op *op, unsigned int shift,
			    bool enable)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	void __iomem *reg_base = cqspi->iobase;
	unsigned int reg;
	int ret;

	reg = readl(reg_base + CQSPI_REG_CONFIG);

	/*
	 * We enable dual byte opcode here. The callers have to set up the
	 * extension opcode based on which type of operation it is.
	 */
	if (enable) {
		reg |= CQSPI_REG_CONFIG_DTR_PROTO;
		reg |= CQSPI_REG_CONFIG_DUAL_OPCODE;

		/* Set up command opcode extension. */
		ret = cqspi_setup_opcode_ext(f_pdata, op, shift);
		if (ret)
			return ret;
	} else {
		reg &= ~CQSPI_REG_CONFIG_DTR_PROTO;
		reg &= ~CQSPI_REG_CONFIG_DUAL_OPCODE;
	}

	writel(reg, reg_base + CQSPI_REG_CONFIG);

	return cqspi_wait_idle(cqspi);
}

static int cqspi_command_read(struct cqspi_flash_pdata *f_pdata,
			      const struct spi_mem_op *op)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	void __iomem *reg_base = cqspi->iobase;
	u8 *rxbuf = op->data.buf.in;
	u8 opcode;
	size_t n_rx = op->data.nbytes;
	unsigned int rdreg;
	unsigned int reg;
	unsigned int dummy_clk;
	size_t read_len;
	int status;

	status = cqspi_set_protocol(f_pdata, op);
	if (status)
		return status;

	status = cqspi_enable_dtr(f_pdata, op, CQSPI_REG_OP_EXT_STIG_LSB,
				  f_pdata->dtr);
	if (status)
		return status;

	if (!n_rx || n_rx > CQSPI_STIG_DATA_LEN_MAX || !rxbuf) {
		dev_err(&cqspi->pdev->dev,
			"Invalid input argument, len %zu rxbuf 0x%p\n",
			n_rx, rxbuf);
		return -EINVAL;
	}

	if (f_pdata->dtr)
		opcode = op->cmd.opcode >> 8;
	else
		opcode = op->cmd.opcode;

	reg = opcode << CQSPI_REG_CMDCTRL_OPCODE_LSB;

	if (op->addr.nbytes) {
		reg |= (0x1 << CQSPI_REG_CMDCTRL_ADDR_EN_LSB);
		reg |= ((op->addr.nbytes - 1) &
			CQSPI_REG_CMDCTRL_ADD_BYTES_MASK) <<
			CQSPI_REG_CMDCTRL_ADD_BYTES_LSB;
		writel(op->addr.val, reg_base + CQSPI_REG_CMDADDRESS);
	}

	rdreg = cqspi_calc_rdreg(f_pdata);
	writel(rdreg, reg_base + CQSPI_REG_RD_INSTR);

	dummy_clk = cqspi_calc_dummy(op, f_pdata->dtr);
	if (dummy_clk > CQSPI_DUMMY_CLKS_MAX)
		return -EOPNOTSUPP;

	if (cqspi->extra_dummy)
		dummy_clk++;

	if (dummy_clk)
		reg |= (dummy_clk & CQSPI_REG_CMDCTRL_DUMMY_MASK)
		     << CQSPI_REG_CMDCTRL_DUMMY_LSB;

	reg |= (0x1 << CQSPI_REG_CMDCTRL_RD_EN_LSB);

	/* 0 means 1 byte. */
	reg |= (((n_rx - 1) & CQSPI_REG_CMDCTRL_RD_BYTES_MASK)
		<< CQSPI_REG_CMDCTRL_RD_BYTES_LSB);
	status = cqspi_exec_flash_cmd(cqspi, reg);
	if (status)
		return status;

	reg = readl(reg_base + CQSPI_REG_CMDREADDATALOWER);

	/* Put the read value into rx_buf */
	read_len = (n_rx > 4) ? 4 : n_rx;
	memcpy(rxbuf, &reg, read_len);
	rxbuf += read_len;

	if (n_rx > 4) {
		reg = readl(reg_base + CQSPI_REG_CMDREADDATAUPPER);

		read_len = n_rx - read_len;
		memcpy(rxbuf, &reg, read_len);
	}

	return 0;
}

static int cqspi_setdlldelay(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct cqspi_st *cqspi = spi_master_get_devdata(mem->spi->master);
	struct platform_device *pdev = cqspi->pdev;
	struct cqspi_flash_pdata *f_pdata;
	int i;
	u8 j;
	int ret;
	u8 *id = op->data.buf.in;
	bool rxtapfound = false;
	u8 min_rxtap = 0;
	u8 max_rxtap = 0;
	u8 avg_rxtap;
	bool id_matched;
	u32 txtap = 0;
	u8 max_tap;
	s8 max_windowsize = -1;
	u8 windowsize;
	u8 dummy_incr;
	u8 dummy_flag = 0;
	u8 count;
	u64 tera_macro = TERA_MACRO;
	u8 max_index = 0, min_index = 0;
	unsigned int reg;

	reg = readl(cqspi->iobase + CQSPI_REG_CONFIG);
	reg &= CQSPI_REG_CONFIG_ENABLE_MASK;
	if (!reg)
		return 0;

	f_pdata = &cqspi->f_pdata[mem->spi->chip_select];
	max_tap = (do_div(tera_macro, cqspi->master_ref_clk_hz) / 160);

	if (cqspi->dll_mode == CQSPI_DLL_MODE_MASTER) {
		/* Drive DLL reset bit to low */
		writel(0, cqspi->iobase + CQSPI_REG_PHY_CONFIG);

		/* Set initial delay value */
		writel(0x4, cqspi->iobase + CQSPI_REG_PHY_MASTER_CTRL);

		/* Set DLL reset bit */
		writel(CQSPI_REG_PHY_CONFIG_RESET_FLD_MASK,
		       cqspi->iobase + CQSPI_REG_PHY_CONFIG);

		/* Check for loopback lock */
		ret = cqspi_wait_for_bit(cqspi->iobase + CQSPI_REG_DLL_LOWER,
					 CQSPI_REG_DLL_LOWER_LPBK_LOCK_MASK, 0);
		if (ret) {
			dev_err(&pdev->dev,
				"Loopback lock bit error (%i)\n", ret);
			return ret;
		}

		/* Re-synchronize slave DLLs */
		writel(CQSPI_REG_PHY_CONFIG_RESET_FLD_MASK,
		       cqspi->iobase + CQSPI_REG_PHY_CONFIG);
		writel(CQSPI_REG_PHY_CONFIG_RESET_FLD_MASK |
		       CQSPI_REG_PHY_CONFIG_RESYNC_FLD_MASK,
		       cqspi->iobase + CQSPI_REG_PHY_CONFIG);

		txtap = CQSPI_TX_TAP_MASTER <<
			CQSPI_REG_PHY_CONFIG_TX_DLL_DLY_LSB;
		max_tap = CQSPI_MAX_DLL_TAPS;
	}

	cqspi->extra_dummy = false;
	for (dummy_incr = 0; dummy_incr <= 1; dummy_incr++) {
		if (dummy_incr)
			cqspi->extra_dummy = true;
		for (i = 0; i <= max_tap; i++) {
			writel((txtap | i |
			       CQSPI_REG_PHY_CONFIG_RESET_FLD_MASK),
			       cqspi->iobase + CQSPI_REG_PHY_CONFIG);
			writel((CQSPI_REG_PHY_CONFIG_RESYNC_FLD_MASK | txtap |
			       i | CQSPI_REG_PHY_CONFIG_RESET_FLD_MASK),
			       cqspi->iobase + CQSPI_REG_PHY_CONFIG);
			if (cqspi->dll_mode == CQSPI_DLL_MODE_MASTER) {
				ret = cqspi_wait_for_bit(cqspi->iobase +
							 CQSPI_REG_DLL_LOWER,
					CQSPI_REG_DLL_LOWER_DLL_LOCK_MASK, 0);
				if (ret)
					return ret;
			}

			count = 0;
			do {
				count += 1;
				ret = cqspi_command_read(f_pdata, op);
				if (ret < 0) {
					dev_err(&pdev->dev,
						"error %d reading JEDEC ID\n", ret);
					return ret;
				}

				id_matched = true;
				for (j = 0; j < op->data.nbytes; j++) {
					if (mem->device_id[j] != id[j]) {
						id_matched = false;
						break;
					}
				}
			} while (id_matched && (count <= 10));

			if (id_matched && !rxtapfound) {
				if (cqspi->dll_mode == CQSPI_DLL_MODE_MASTER) {
					min_rxtap = readl(cqspi->iobase +
							  CQSPI_REG_DLL_OBSVBLE_UPPER) &
							  CQSPI_REG_DLL_UPPER_RX_FLD_MASK;
					max_rxtap = min_rxtap;
					max_index = i;
					min_index = i;
				} else {
					min_rxtap = i;
					max_rxtap = i;
				}
				rxtapfound = true;
			}

			if (id_matched && rxtapfound) {
				if (cqspi->dll_mode == CQSPI_DLL_MODE_MASTER) {
					max_rxtap = readl(cqspi->iobase +
							  CQSPI_REG_DLL_OBSVBLE_UPPER) &
							  CQSPI_REG_DLL_UPPER_RX_FLD_MASK;
					max_index = i;
				} else {
					max_rxtap = i;
				}
			}
			if ((!id_matched || i == max_tap) && rxtapfound) {
				windowsize = max_rxtap - min_rxtap + 1;
				if (windowsize > max_windowsize) {
					dummy_flag = dummy_incr;
					max_windowsize = windowsize;
					if (cqspi->dll_mode ==
					    CQSPI_DLL_MODE_MASTER)
						avg_rxtap = (max_index +
							     min_index);
					else
						avg_rxtap = (max_rxtap +
							     min_rxtap);
					avg_rxtap /= 2;
				}

				if (windowsize >= 3)
					i = max_tap;

				rxtapfound = false;
			}
		}
		if (!dummy_incr) {
			rxtapfound = false;
			min_rxtap = 0;
			max_rxtap = 0;
		}
	}
	if (!dummy_flag)
		cqspi->extra_dummy = false;

	if (max_windowsize < 3)
		return -EINVAL;

	writel((txtap | avg_rxtap | CQSPI_REG_PHY_CONFIG_RESET_FLD_MASK),
	       cqspi->iobase + CQSPI_REG_PHY_CONFIG);
	writel((CQSPI_REG_PHY_CONFIG_RESYNC_FLD_MASK | txtap | avg_rxtap |
	       CQSPI_REG_PHY_CONFIG_RESET_FLD_MASK),
	       cqspi->iobase + CQSPI_REG_PHY_CONFIG);
	if (cqspi->dll_mode == CQSPI_DLL_MODE_MASTER) {
		ret = cqspi_wait_for_bit(cqspi->iobase + CQSPI_REG_DLL_LOWER,
					 CQSPI_REG_DLL_LOWER_DLL_LOCK_MASK, 0);
		if (ret)
			return ret;
	}

	cqspi->clk_tuned = true;

	return 0;
}

static void cqspi_setup_ddrmode(struct cqspi_st *cqspi)
{
	u32 reg;

	reg = readl(cqspi->iobase + CQSPI_REG_CONFIG);
	reg &= ~CQSPI_REG_CONFIG_ENABLE_MASK;
	writel(reg, cqspi->iobase + CQSPI_REG_CONFIG);

	reg = readl(cqspi->iobase + CQSPI_REG_CONFIG);
	reg |= (CQSPI_REG_CONFIG_PHY_ENABLE_MASK);
	writel(reg, cqspi->iobase + CQSPI_REG_CONFIG);

	/* Program POLL_CNT */
	reg = readl(cqspi->iobase + CQSPI_REG_WR_COMPLETION_CTRL);
	reg &= ~CQSPI_REG_WRCOMPLETION_POLLCNT_MASK;
	reg |= (0x3 << CQSPI_REG_WRCOMPLETION_POLLCNY_LSB);
	writel(reg, cqspi->iobase + CQSPI_REG_WR_COMPLETION_CTRL);

	reg = readl(cqspi->iobase + CQSPI_REG_READCAPTURE);
	reg |= CQSPI_REG_READCAPTURE_DQS_ENABLE;
	writel(reg, cqspi->iobase + CQSPI_REG_READCAPTURE);

	reg = readl(cqspi->iobase + CQSPI_REG_CONFIG);
	reg |= CQSPI_REG_CONFIG_ENABLE_MASK;
	writel(reg, cqspi->iobase + CQSPI_REG_CONFIG);
}

static void cqspi_setup_sdrmode(struct cqspi_st *cqspi)
{
	u32 reg;

	reg = readl(cqspi->iobase + CQSPI_REG_CONFIG);
	reg &= ~CQSPI_REG_CONFIG_ENABLE_MASK;
	writel(reg, cqspi->iobase + CQSPI_REG_CONFIG);

	reg &= ~CQSPI_REG_CONFIG_DTR_PROTO;
	reg &= ~CQSPI_REG_CONFIG_DUAL_OPCODE;
	reg &= ~CQSPI_REG_CONFIG_PHY_ENABLE_MASK;
	writel(reg, cqspi->iobase + CQSPI_REG_CONFIG);

	/* Program POLL_CNT */
	reg = readl(cqspi->iobase + CQSPI_REG_WR_COMPLETION_CTRL);
	reg &= ~CQSPI_REG_WRCOMPLETION_POLLCNT_MASK;
	reg |= (1 << CQSPI_REG_WRCOMPLETION_POLLCNY_LSB);
	writel(reg, cqspi->iobase + CQSPI_REG_WR_COMPLETION_CTRL);

	reg = readl(cqspi->iobase + CQSPI_REG_READCAPTURE);
	reg &= ~CQSPI_REG_READCAPTURE_DQS_ENABLE;
	reg |= (1 & CQSPI_REG_READCAPTURE_DELAY_MASK)
		<< CQSPI_REG_READCAPTURE_DELAY_LSB;
	writel(reg, cqspi->iobase + CQSPI_REG_READCAPTURE);

	writel(CQSPI_REG_PHY_CONFIG_RESET_FLD_MASK,
	       cqspi->iobase + CQSPI_REG_PHY_CONFIG);

	reg = readl(cqspi->iobase + CQSPI_REG_CONFIG);
	reg |= CQSPI_REG_CONFIG_ENABLE_MASK;
	writel(reg, cqspi->iobase + CQSPI_REG_CONFIG);

	cqspi->clk_tuned = false;
	cqspi->extra_dummy = false;
}

static void cqspi_periodictuning(struct work_struct *work)
{
	struct delayed_work *d = to_delayed_work(work);
	struct spi_mem *mem = container_of(d, struct spi_mem, complete_work);
	struct cqspi_st *cqspi = spi_master_get_devdata(mem->spi->master);
	int ret;
	u8 buf[CQSPI_READ_ID_LEN];

	if (!mem->request_completion.done)
		wait_for_completion(&mem->request_completion);

	reinit_completion(&cqspi->tuning_complete);
	cqspi->tuning_op.data.buf.in = buf;
	cqspi->tuning_op.data.nbytes = CQSPI_READ_ID_LEN;
	ret = cqspi_setdlldelay(mem, &cqspi->tuning_op);
	complete_all(&cqspi->tuning_complete);
	if (ret) {
		dev_err(&cqspi->pdev->dev,
			"Setting dll delay error (%i)\n", ret);
	} else {
		schedule_delayed_work(&mem->complete_work,
				      msecs_to_jiffies(CQSPI_TUNING_PERIODICITY_MS));
	}
}

static int cqspi_setup_edgemode(struct spi_mem *mem,
				const struct spi_mem_op *op)
{
	struct cqspi_st *cqspi = spi_master_get_devdata(mem->spi->master);
	int ret;

	if (cqspi->clk_tuned)
		return 0;

	memcpy(&cqspi->tuning_op, op, sizeof(struct spi_mem_op));

	cqspi_setup_ddrmode(cqspi);

	ret = cqspi_setdlldelay(mem, op);
	if (ret)
		return ret;

	complete_all(&cqspi->tuning_complete);
	INIT_DELAYED_WORK(&mem->complete_work, cqspi_periodictuning);
	schedule_delayed_work(&mem->complete_work,
			      msecs_to_jiffies(CQSPI_TUNING_PERIODICITY_MS));

	return 0;
}

static int cqspi_command_write(struct cqspi_flash_pdata *f_pdata,
			       const struct spi_mem_op *op)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	void __iomem *reg_base = cqspi->iobase;
	u8 opcode;
	const u8 *txbuf = op->data.buf.out;
	size_t n_tx = op->data.nbytes;
	unsigned int reg;
	unsigned int data;
	size_t write_len;
	int ret;

	ret = cqspi_set_protocol(f_pdata, op);
	if (ret)
		return ret;

	ret = cqspi_enable_dtr(f_pdata, op, CQSPI_REG_OP_EXT_STIG_LSB,
			       f_pdata->dtr);
	if (ret)
		return ret;

	if (n_tx > CQSPI_STIG_DATA_LEN_MAX || (n_tx && !txbuf)) {
		dev_err(&cqspi->pdev->dev,
			"Invalid input argument, cmdlen %zu txbuf 0x%p\n",
			n_tx, txbuf);
		return -EINVAL;
	}

	reg = cqspi_calc_rdreg(f_pdata);
	writel(reg, reg_base + CQSPI_REG_RD_INSTR);

	if (f_pdata->dtr)
		opcode = op->cmd.opcode >> 8;
	else
		opcode = op->cmd.opcode;

	reg = opcode << CQSPI_REG_CMDCTRL_OPCODE_LSB;

	if (op->addr.nbytes) {
		reg |= (0x1 << CQSPI_REG_CMDCTRL_ADDR_EN_LSB);
		reg |= ((op->addr.nbytes - 1) &
			CQSPI_REG_CMDCTRL_ADD_BYTES_MASK)
			<< CQSPI_REG_CMDCTRL_ADD_BYTES_LSB;

		writel(op->addr.val, reg_base + CQSPI_REG_CMDADDRESS);
	}

	if (n_tx) {
		reg |= (0x1 << CQSPI_REG_CMDCTRL_WR_EN_LSB);
		reg |= ((n_tx - 1) & CQSPI_REG_CMDCTRL_WR_BYTES_MASK)
			<< CQSPI_REG_CMDCTRL_WR_BYTES_LSB;
		data = 0;
		write_len = (n_tx > 4) ? 4 : n_tx;
		memcpy(&data, txbuf, write_len);
		txbuf += write_len;
		writel(data, reg_base + CQSPI_REG_CMDWRITEDATALOWER);

		if (n_tx > 4) {
			data = 0;
			write_len = n_tx - 4;
			memcpy(&data, txbuf, write_len);
			writel(data, reg_base + CQSPI_REG_CMDWRITEDATAUPPER);
		}
	}

	return cqspi_exec_flash_cmd(cqspi, reg);
}

static int cqspi_read_setup(struct cqspi_flash_pdata *f_pdata,
			    const struct spi_mem_op *op)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	void __iomem *reg_base = cqspi->iobase;
	unsigned int dummy_clk = 0;
	unsigned int reg;
	int ret;
	u8 opcode;

	ret = cqspi_enable_dtr(f_pdata, op, CQSPI_REG_OP_EXT_READ_LSB,
			       f_pdata->dtr);
	if (ret)
		return ret;

	if (f_pdata->dtr)
		opcode = op->cmd.opcode >> 8;
	else
		opcode = op->cmd.opcode;

	reg = opcode << CQSPI_REG_RD_INSTR_OPCODE_LSB;
	reg |= cqspi_calc_rdreg(f_pdata);

	/* Setup dummy clock cycles */
	dummy_clk = cqspi_calc_dummy(op, f_pdata->dtr);

	if (dummy_clk > CQSPI_DUMMY_CLKS_MAX)
		return -EOPNOTSUPP;

	if (cqspi->extra_dummy)
		dummy_clk++;

	if (dummy_clk)
		reg |= (dummy_clk & CQSPI_REG_RD_INSTR_DUMMY_MASK)
		       << CQSPI_REG_RD_INSTR_DUMMY_LSB;

	writel(reg, reg_base + CQSPI_REG_RD_INSTR);

	/* Set address width */
	reg = readl(reg_base + CQSPI_REG_SIZE);
	reg &= ~CQSPI_REG_SIZE_ADDRESS_MASK;
	reg |= (op->addr.nbytes - 1);
	writel(reg, reg_base + CQSPI_REG_SIZE);
	return 0;
}

static int cqspi_indirect_read_execute(struct cqspi_flash_pdata *f_pdata,
				       u8 *rxbuf, loff_t from_addr,
				       const size_t n_rx)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	struct device *dev = &cqspi->pdev->dev;
	void __iomem *reg_base = cqspi->iobase;
	void __iomem *ahb_base = cqspi->ahb_base;
	unsigned int remaining = n_rx;
	unsigned int mod_bytes = n_rx % 4;
	unsigned int bytes_to_read = 0;
	u8 *rxbuf_end = rxbuf + n_rx;
	u8 *rxbuf_start = rxbuf;
	int ret = 0;
	u8 extra_bytes = 0;
	u32 req_bytes;
	u32 threshold_val;
	bool is_unaligned_cnt = false;

	if (n_rx % 2)
		is_unaligned_cnt = true;

	writel(from_addr, reg_base + CQSPI_REG_INDIRECTRDSTARTADDR);
	if (f_pdata->dtr && (from_addr % 2) != 0) {
		mod_bytes += 1;
		if (!is_unaligned_cnt)
			extra_bytes = 2;
	}

	if (f_pdata->dtr && (from_addr % 2) != 0)
		writel(from_addr - 1, reg_base + CQSPI_REG_INDIRECTRDSTARTADDR);

	req_bytes = remaining + is_unaligned_cnt + extra_bytes;
	writel(req_bytes, reg_base + CQSPI_REG_INDIRECTRDBYTES);

	/* Clear all interrupts. */
	writel(CQSPI_IRQ_STATUS_MASK, reg_base + CQSPI_REG_IRQSTATUS);

	writel(CQSPI_IRQ_MASK_RD, reg_base + CQSPI_REG_IRQMASK);
	threshold_val = readl(reg_base + CQSPI_REG_INDIRECTRDWATERMARK);

	reinit_completion(&cqspi->transfer_complete);
	writel(CQSPI_REG_INDIRECTRD_START_MASK,
	       reg_base + CQSPI_REG_INDIRECTRD);

	while (remaining > 0) {
		if (!wait_for_completion_timeout(&cqspi->transfer_complete,
						 msecs_to_jiffies(read_timeout_ms)))
			ret = -ETIMEDOUT;

		if (req_bytes > (threshold_val + cqspi->fifo_width))
			bytes_to_read = threshold_val + cqspi->fifo_width;
		else
			bytes_to_read = req_bytes;

		if (ret && bytes_to_read == 0) {
			dev_err(dev, "Indirect read timeout, no bytes\n");
			goto failrd;
		}

		while (bytes_to_read != 0) {
			unsigned int word_remain = round_down(remaining, 4);
			unsigned int bytes_read = 0;

			bytes_to_read = bytes_to_read > remaining ?
					remaining : bytes_to_read;
			bytes_to_read = round_down(bytes_to_read, 4);
			/* Read 4 byte word chunks then single bytes */
			if (bytes_to_read) {
				u8 offset = 0;

				if (f_pdata->dtr && ((from_addr % 2) != 0) &&
				    rxbuf == rxbuf_start) {
					unsigned int temp = ioread32(ahb_base);

					temp >>= 8;
					memcpy(rxbuf, &temp, 3);
					bytes_to_read -= 3;
					offset = 3;
					bytes_read += 3;
				}
				if (bytes_to_read >= 4) {
					ioread32_rep(ahb_base, rxbuf + offset,
						     (bytes_to_read / 4));
					bytes_read += (bytes_to_read / 4) * 4;
				}
			} else if (!word_remain && mod_bytes) {
				unsigned int temp = ioread32(ahb_base);

				if (f_pdata->dtr && ((from_addr % 2) != 0) &&
				    rxbuf == rxbuf_start)
					temp >>= 8;

				bytes_to_read = min(remaining, mod_bytes);
				bytes_read = min((unsigned int)
						    (rxbuf_end - rxbuf),
						     bytes_to_read);
				memcpy(rxbuf, &temp, bytes_read);
			}
			rxbuf += bytes_read;
			remaining -= bytes_read;
			req_bytes -= bytes_read;
			bytes_to_read = cqspi_get_rd_sram_level(cqspi);
			bytes_to_read *= cqspi->fifo_width;
		}

		if (remaining > 0)
			reinit_completion(&cqspi->transfer_complete);
	}

	/* Check indirect done status */
	ret = cqspi_wait_for_bit(reg_base + CQSPI_REG_INDIRECTRD,
				 CQSPI_REG_INDIRECTRD_DONE_MASK, 0);
	if (ret) {
		dev_err(dev, "Indirect read completion error (%i)\n", ret);
		goto failrd;
	}

	/* Disable interrupt */
	writel(0, reg_base + CQSPI_REG_IRQMASK);

	/* Clear indirect completion status */
	writel(CQSPI_REG_INDIRECTRD_DONE_MASK, reg_base + CQSPI_REG_INDIRECTRD);

	return 0;

failrd:
	/* Disable interrupt */
	writel(0, reg_base + CQSPI_REG_IRQMASK);

	/* Cancel the indirect read */
	writel(CQSPI_REG_INDIRECTWR_CANCEL_MASK,
	       reg_base + CQSPI_REG_INDIRECTRD);
	return ret;
}

static int cqspi_versal_device_reset(struct cqspi_st *cqspi, u8 reset_type)
{
	struct platform_device *pdev = cqspi->pdev;
	int ret;
	enum of_gpio_flags flags;

	if (reset_type == CQSPI_RESET_TYPE_HWPIN) {
		cqspi->gpio = of_get_named_gpio_flags(pdev->dev.of_node,
						      "reset-gpios", 0, &flags);
		if (!gpio_is_valid(cqspi->gpio))
			return -EIO;
		ret = devm_gpio_request_one(&pdev->dev, cqspi->gpio, flags,
					    "flash-reset");
		if (ret) {
			dev_err(&pdev->dev,
				"failed to get reset-gpios: %d\n", ret);
			return -EIO;
		}

		/* Request for PIN */
		zynqmp_pm_pinctrl_request(CQSPI_VERSAL_MIO_NODE_ID_12);

		/* Enable hysteresis in cmos receiver */
		zynqmp_pm_pinctrl_set_config(CQSPI_VERSAL_MIO_NODE_ID_12,
					     PM_PINCTRL_CONFIG_SCHMITT_CMOS,
					     PM_PINCTRL_INPUT_TYPE_SCHMITT);

		/* Set the direction as output and enable the output */
		gpio_direction_output(cqspi->gpio, 1);

		/* Disable Tri-state */
		zynqmp_pm_pinctrl_set_config(CQSPI_VERSAL_MIO_NODE_ID_12,
					     PM_PINCTRL_CONFIG_TRI_STATE,
					     PM_PINCTRL_TRI_STATE_DISABLE);
		udelay(1);

		/* Set value 0 to pin */
		gpio_set_value(cqspi->gpio, 0);
		udelay(10);

		/* Set value 1 to pin */
		gpio_set_value(cqspi->gpio, 1);
		udelay(35);
	} else {
		ret = -EINVAL;
	}

	return ret;
}

static int cqspi_versal_indirect_read_dma(struct cqspi_flash_pdata *f_pdata,
					  u_char *rxbuf, loff_t from_addr,
					  size_t n_rx)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	struct device *dev = &cqspi->pdev->dev;
	void __iomem *reg_base = cqspi->iobase;
	u32 phy_reg, rd_instr, addr_width;
	u32 reg, bytes_to_dma;
	loff_t addr = from_addr;
	void *buf = rxbuf;
	dma_addr_t dma_addr;
	u8 bytes_rem;
	int ret = 0;

	bytes_rem = n_rx % 4;
	bytes_to_dma = (n_rx - bytes_rem);

	if (!bytes_to_dma || (f_pdata->dtr && ((from_addr % 2) != 0))) {
		bytes_to_dma = 0;
		bytes_rem = n_rx;
		goto nondmard;
	}

	/* Issue controller reset */
	if (cqspi->dll_mode != CQSPI_DLL_MODE_MASTER) {
		phy_reg = readl(cqspi->iobase + CQSPI_REG_PHY_CONFIG);
		rd_instr = readl(cqspi->iobase + CQSPI_REG_RD_INSTR);
		addr_width = readl(cqspi->iobase + CQSPI_REG_SIZE);
		zynqmp_pm_reset_assert(VERSAL_OSPI_RESET,
				       PM_RESET_ACTION_ASSERT);
	}

	ret = zynqmp_pm_ospi_mux_select(cqspi->pd_dev_id, PM_OSPI_MUX_SEL_DMA);
	if (ret)
		return ret;

	if (cqspi->dll_mode != CQSPI_DLL_MODE_MASTER) {
		zynqmp_pm_reset_assert(VERSAL_OSPI_RESET,
				       PM_RESET_ACTION_RELEASE);
		reg = readl(reg_base + CQSPI_REG_CONFIG);
		reg &= ~CQSPI_REG_CONFIG_ENABLE_MASK;
		writel(reg, reg_base + CQSPI_REG_CONFIG);
		writel(cqspi->fifo_depth / 2, cqspi->iobase +
		       CQSPI_REG_SRAMPARTITION);

		/* Load indirect trigger address. */
		writel(cqspi->trigger_address,
		       cqspi->iobase + CQSPI_REG_INDIRECTTRIGGER);

		/* Program read watermark -- 1/2 of the FIFO. */
		writel(cqspi->fifo_depth * cqspi->fifo_width / 2,
		       cqspi->iobase + CQSPI_REG_INDIRECTRDWATERMARK);
		/* Program write watermark -- 1/8 of the FIFO. */
		writel(cqspi->fifo_depth * cqspi->fifo_width / 8,
		       cqspi->iobase + CQSPI_REG_INDIRECTWRWATERMARK);

		/* Disable direct access controller */
		if (!cqspi->use_direct_mode)
			reg &= ~CQSPI_REG_CONFIG_ENB_DIR_ACC_CTRL;

		reg |= CQSPI_REG_CONFIG_ENABLE_MASK;
		writel(reg, cqspi->iobase + CQSPI_REG_CONFIG);
		cqspi->current_cs = -1;
		cqspi->sclk = 0;
		if (f_pdata->dtr) {
			cqspi_setup_ddrmode(cqspi);
			writel(CQSPI_REG_PHY_CONFIG_RESYNC_FLD_MASK | phy_reg,
			       cqspi->iobase + CQSPI_REG_PHY_CONFIG);
		}

		writel(rd_instr, cqspi->iobase + CQSPI_REG_RD_INSTR);
		writel(addr_width, cqspi->iobase + CQSPI_REG_SIZE);
	}

	reg = readl(cqspi->iobase + CQSPI_REG_CONFIG);
	reg |= CQSPI_REG_CONFIG_DMA_MASK;
	writel(reg, cqspi->iobase + CQSPI_REG_CONFIG);

	dma_addr = dma_map_single(dev, rxbuf, bytes_to_dma, DMA_FROM_DEVICE);
	if (dma_mapping_error(dev, dma_addr)) {
		dev_err(dev, "dma mapping failed\n");
		return -ENOMEM;
	}

	writel(from_addr, reg_base + CQSPI_REG_INDIRECTRDSTARTADDR);
	writel(bytes_to_dma, reg_base + CQSPI_REG_INDIRECTRDBYTES);
	writel(CQSPI_REG_VERSAL_ADDRRANGE_WIDTH_VAL,
	       reg_base + CQSPI_REG_INDTRIG_ADDRRANGE);

	/* Clear all interrupts. */
	writel(CQSPI_IRQ_STATUS_MASK, reg_base + CQSPI_REG_IRQSTATUS);

	/* Enable DMA done interrupt */
	writel(CQSPI_REG_VERSAL_DMA_DST_DONE_MASK,
	       reg_base + CQSPI_REG_VERSAL_DMA_DST_I_EN);

	/* Default DMA periph configuration */
	writel(CQSPI_REG_VERSAL_DMA_VAL, reg_base + CQSPI_REG_DMA);

	/* Configure DMA Dst address */
	writel(lower_32_bits(dma_addr),
	       reg_base + CQSPI_REG_VERSAL_DMA_DST_ADDR);
	writel(upper_32_bits(dma_addr),
	       reg_base + CQSPI_REG_VERSAL_DMA_DST_ADDR_MSB);

	/* Configure DMA Src address */
	writel(cqspi->trigger_address, reg_base +
	       CQSPI_REG_VERSAL_DMA_SRC_ADDR);

	/* Set DMA destination size */
	writel(bytes_to_dma, reg_base + CQSPI_REG_VERSAL_DMA_DST_SIZE);

	/* Set DMA destination control */
	writel(CQSPI_REG_VERSAL_DMA_DST_CTRL_VAL,
	       reg_base + CQSPI_REG_VERSAL_DMA_DST_CTRL);

	writel(CQSPI_REG_INDIRECTRD_START_MASK,
	       reg_base + CQSPI_REG_INDIRECTRD);

	reinit_completion(&cqspi->transfer_complete);

	if (!wait_for_completion_timeout(&cqspi->transfer_complete,
					 msecs_to_jiffies(read_timeout_ms))) {
		ret = -ETIMEDOUT;
		goto failrd;
	}

	/* Disable DMA interrupt */
	writel(0x0, cqspi->iobase + CQSPI_REG_VERSAL_DMA_DST_I_DIS);

	/* Clear indirect completion status */
	writel(CQSPI_REG_INDIRECTRD_DONE_MASK,
	       cqspi->iobase + CQSPI_REG_INDIRECTRD);
	dma_unmap_single(dev, dma_addr, bytes_to_dma, DMA_FROM_DEVICE);

	reg = readl(cqspi->iobase + CQSPI_REG_CONFIG);
	reg &= ~CQSPI_REG_CONFIG_DMA_MASK;
	writel(reg, cqspi->iobase + CQSPI_REG_CONFIG);

	ret = zynqmp_pm_ospi_mux_select(cqspi->pd_dev_id,
					PM_OSPI_MUX_SEL_LINEAR);
	if (ret)
		return ret;

nondmard:
	if (bytes_rem) {
		addr += bytes_to_dma;
		buf += bytes_to_dma;
		ret = cqspi_indirect_read_execute(f_pdata, buf, addr,
						  bytes_rem);
		if (ret)
			return ret;
	}

	return 0;

failrd:
	/* Disable DMA interrupt */
	writel(0x0, reg_base + CQSPI_REG_VERSAL_DMA_DST_I_DIS);

	/* Cancel the indirect read */
	writel(CQSPI_REG_INDIRECTWR_CANCEL_MASK,
	       reg_base + CQSPI_REG_INDIRECTRD);

	dma_unmap_single(dev, dma_addr, bytes_to_dma, DMA_FROM_DEVICE);

	reg = readl(cqspi->iobase + CQSPI_REG_CONFIG);
	reg &= ~CQSPI_REG_CONFIG_DMA_MASK;
	writel(reg, cqspi->iobase + CQSPI_REG_CONFIG);

	zynqmp_pm_ospi_mux_select(cqspi->pd_dev_id, PM_OSPI_MUX_SEL_LINEAR);

	return ret;
}

static int cqspi_write_setup(struct cqspi_flash_pdata *f_pdata,
			     const struct spi_mem_op *op)
{
	unsigned int reg;
	int ret;
	struct cqspi_st *cqspi = f_pdata->cqspi;
	void __iomem *reg_base = cqspi->iobase;
	u8 opcode;

	ret = cqspi_enable_dtr(f_pdata, op, CQSPI_REG_OP_EXT_WRITE_LSB,
			       f_pdata->dtr);
	if (ret)
		return ret;

	if (f_pdata->dtr)
		opcode = op->cmd.opcode >> 8;
	else
		opcode = op->cmd.opcode;

	/* Set opcode. */
	reg = opcode << CQSPI_REG_WR_INSTR_OPCODE_LSB;
	reg |= f_pdata->data_width << CQSPI_REG_WR_INSTR_TYPE_DATA_LSB;
	reg |= f_pdata->addr_width << CQSPI_REG_WR_INSTR_TYPE_ADDR_LSB;
	writel(reg, reg_base + CQSPI_REG_WR_INSTR);
	reg = cqspi_calc_rdreg(f_pdata);
	writel(reg, reg_base + CQSPI_REG_RD_INSTR);

	/*
	 * SPI NAND flashes require the address of the status register to be
	 * passed in the Read SR command. Also, some SPI NOR flashes like the
	 * cypress Semper flash expect a 4-byte dummy address in the Read SR
	 * command in DTR mode.
	 *
	 * But this controller does not support address phase in the Read SR
	 * command when doing auto-HW polling. So, disable write completion
	 * polling on the controller's side. spinand and spi-nor will take
	 * care of polling the status register.
	 */
	reg = readl(reg_base + CQSPI_REG_WR_COMPLETION_CTRL);
	reg |= CQSPI_REG_WR_DISABLE_AUTO_POLL;
	writel(reg, reg_base + CQSPI_REG_WR_COMPLETION_CTRL);

	reg = readl(reg_base + CQSPI_REG_SIZE);
	reg &= ~CQSPI_REG_SIZE_ADDRESS_MASK;
	reg |= (op->addr.nbytes - 1);
	writel(reg, reg_base + CQSPI_REG_SIZE);
	return 0;
}

static int cqspi_indirect_write_execute(struct cqspi_flash_pdata *f_pdata,
					loff_t to_addr, const u8 *txbuf,
					const size_t n_tx)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	struct device *dev = &cqspi->pdev->dev;
	void __iomem *reg_base = cqspi->iobase;
	unsigned int remaining = n_tx;
	unsigned int write_bytes;
	int ret;
	u8 unaligned_bytes = 0;

	if (f_pdata->dtr && ((n_tx % 2) != 0))
		unaligned_bytes = 1;

	writel(to_addr, reg_base + CQSPI_REG_INDIRECTWRSTARTADDR);
	writel(remaining + unaligned_bytes,
	       reg_base + CQSPI_REG_INDIRECTWRBYTES);

	/* Clear all interrupts. */
	writel(CQSPI_IRQ_STATUS_MASK, reg_base + CQSPI_REG_IRQSTATUS);

	writel(CQSPI_IRQ_MASK_WR, reg_base + CQSPI_REG_IRQMASK);

	reinit_completion(&cqspi->transfer_complete);
	writel(CQSPI_REG_INDIRECTWR_START_MASK,
	       reg_base + CQSPI_REG_INDIRECTWR);
	/*
	 * As per 66AK2G02 TRM SPRUHY8F section 11.15.5.3 Indirect Access
	 * Controller programming sequence, couple of cycles of
	 * QSPI_REF_CLK delay is required for the above bit to
	 * be internally synchronized by the QSPI module. Provide 5
	 * cycles of delay.
	 */
	if (cqspi->wr_delay)
		ndelay(cqspi->wr_delay);

	while (remaining > 0) {
		size_t write_words, mod_bytes;

		write_bytes = remaining;
		write_words = write_bytes / 4;
		mod_bytes = write_bytes % 4;
		/* Write 4 bytes at a time then single bytes. */
		if (write_words) {
			iowrite32_rep(cqspi->ahb_base, txbuf, write_words);
			txbuf += (write_words * 4);
		}
		if (mod_bytes) {
			unsigned int temp = 0xFFFFFFFF;

			memcpy(&temp, txbuf, mod_bytes + unaligned_bytes);
			iowrite32(temp, cqspi->ahb_base);
			txbuf += mod_bytes;
		}

		if (!wait_for_completion_timeout(&cqspi->transfer_complete,
						 msecs_to_jiffies(CQSPI_TIMEOUT_MS))) {
			dev_err(dev, "Indirect write timeout\n");
			ret = -ETIMEDOUT;
			goto failwr;
		}

		remaining -= write_bytes;

		if (remaining > 0)
			reinit_completion(&cqspi->transfer_complete);
	}

	/* Check indirect done status */
	ret = cqspi_wait_for_bit(reg_base + CQSPI_REG_INDIRECTWR,
				 CQSPI_REG_INDIRECTWR_DONE_MASK, 0);
	if (ret) {
		dev_err(dev, "Indirect write completion error (%i)\n", ret);
		goto failwr;
	}

	/* Disable interrupt. */
	writel(0, reg_base + CQSPI_REG_IRQMASK);

	/* Clear indirect completion status */
	writel(CQSPI_REG_INDIRECTWR_DONE_MASK, reg_base + CQSPI_REG_INDIRECTWR);

	cqspi_wait_idle(cqspi);

	return 0;

failwr:
	/* Disable interrupt. */
	writel(0, reg_base + CQSPI_REG_IRQMASK);

	/* Cancel the indirect write */
	writel(CQSPI_REG_INDIRECTWR_CANCEL_MASK,
	       reg_base + CQSPI_REG_INDIRECTWR);
	return ret;
}

static void cqspi_chipselect(struct cqspi_flash_pdata *f_pdata)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	void __iomem *reg_base = cqspi->iobase;
	unsigned int chip_select = f_pdata->cs;
	unsigned int reg;

	reg = readl(reg_base + CQSPI_REG_CONFIG);
	if (cqspi->is_decoded_cs) {
		reg |= CQSPI_REG_CONFIG_DECODE_MASK;
	} else {
		reg &= ~CQSPI_REG_CONFIG_DECODE_MASK;

		/* Convert CS if without decoder.
		 * CS0 to 4b'1110
		 * CS1 to 4b'1101
		 * CS2 to 4b'1011
		 * CS3 to 4b'0111
		 */
		chip_select = 0xF & ~(1 << chip_select);
	}

	reg &= ~(CQSPI_REG_CONFIG_CHIPSELECT_MASK
		 << CQSPI_REG_CONFIG_CHIPSELECT_LSB);
	reg |= (chip_select & CQSPI_REG_CONFIG_CHIPSELECT_MASK)
	    << CQSPI_REG_CONFIG_CHIPSELECT_LSB;
	writel(reg, reg_base + CQSPI_REG_CONFIG);
}

static unsigned int calculate_ticks_for_ns(const unsigned int ref_clk_hz,
					   const unsigned int ns_val)
{
	unsigned int ticks;

	ticks = ref_clk_hz / 1000;	/* kHz */
	ticks = DIV_ROUND_UP(ticks * ns_val, 1000000);

	return ticks;
}

static void cqspi_delay(struct cqspi_flash_pdata *f_pdata)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	void __iomem *iobase = cqspi->iobase;
	const unsigned int ref_clk_hz = cqspi->master_ref_clk_hz;
	unsigned int tshsl, tchsh, tslch, tsd2d;
	unsigned int reg;
	unsigned int tsclk;

	/* calculate the number of ref ticks for one sclk tick */
	tsclk = DIV_ROUND_UP(ref_clk_hz, cqspi->sclk);

	tshsl = calculate_ticks_for_ns(ref_clk_hz, f_pdata->tshsl_ns);
	/* this particular value must be at least one sclk */
	if (tshsl < tsclk)
		tshsl = tsclk;

	tchsh = calculate_ticks_for_ns(ref_clk_hz, f_pdata->tchsh_ns);
	tslch = calculate_ticks_for_ns(ref_clk_hz, f_pdata->tslch_ns);
	tsd2d = calculate_ticks_for_ns(ref_clk_hz, f_pdata->tsd2d_ns);

	reg = (tshsl & CQSPI_REG_DELAY_TSHSL_MASK)
	       << CQSPI_REG_DELAY_TSHSL_LSB;
	reg |= (tchsh & CQSPI_REG_DELAY_TCHSH_MASK)
		<< CQSPI_REG_DELAY_TCHSH_LSB;
	reg |= (tslch & CQSPI_REG_DELAY_TSLCH_MASK)
		<< CQSPI_REG_DELAY_TSLCH_LSB;
	reg |= (tsd2d & CQSPI_REG_DELAY_TSD2D_MASK)
		<< CQSPI_REG_DELAY_TSD2D_LSB;
	writel(reg, iobase + CQSPI_REG_DELAY);
}

static void cqspi_config_baudrate_div(struct cqspi_st *cqspi)
{
	const unsigned int ref_clk_hz = cqspi->master_ref_clk_hz;
	void __iomem *reg_base = cqspi->iobase;
	u32 reg, div;

	/* Recalculate the baudrate divisor based on QSPI specification. */
	div = DIV_ROUND_UP(ref_clk_hz, 2 * cqspi->sclk) - 1;

	reg = readl(reg_base + CQSPI_REG_CONFIG);
	reg &= ~(CQSPI_REG_CONFIG_BAUD_MASK << CQSPI_REG_CONFIG_BAUD_LSB);
	reg |= (div & CQSPI_REG_CONFIG_BAUD_MASK) << CQSPI_REG_CONFIG_BAUD_LSB;
	writel(reg, reg_base + CQSPI_REG_CONFIG);
}

static void cqspi_readdata_capture(struct cqspi_st *cqspi,
				   const bool bypass,
				   const unsigned int delay)
{
	void __iomem *reg_base = cqspi->iobase;
	unsigned int reg;

	reg = readl(reg_base + CQSPI_REG_READCAPTURE);

	if (bypass)
		reg |= (1 << CQSPI_REG_READCAPTURE_BYPASS_LSB);
	else
		reg &= ~(1 << CQSPI_REG_READCAPTURE_BYPASS_LSB);

	reg &= ~(CQSPI_REG_READCAPTURE_DELAY_MASK
		 << CQSPI_REG_READCAPTURE_DELAY_LSB);

	reg |= (delay & CQSPI_REG_READCAPTURE_DELAY_MASK)
		<< CQSPI_REG_READCAPTURE_DELAY_LSB;

	writel(reg, reg_base + CQSPI_REG_READCAPTURE);
}

static void cqspi_controller_enable(struct cqspi_st *cqspi, bool enable)
{
	void __iomem *reg_base = cqspi->iobase;
	unsigned int reg;

	reg = readl(reg_base + CQSPI_REG_CONFIG);

	if (enable)
		reg |= CQSPI_REG_CONFIG_ENABLE_MASK;
	else
		reg &= ~CQSPI_REG_CONFIG_ENABLE_MASK;

	writel(reg, reg_base + CQSPI_REG_CONFIG);
}

static void cqspi_configure(struct cqspi_flash_pdata *f_pdata,
			    unsigned long sclk)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	int switch_cs = (cqspi->current_cs != f_pdata->cs);
	int switch_ck = (cqspi->sclk != sclk);

	if (switch_cs || switch_ck)
		cqspi_controller_enable(cqspi, 0);

	/* Switch chip select. */
	if (switch_cs) {
		cqspi->current_cs = f_pdata->cs;
		cqspi_chipselect(f_pdata);
	}

	/* Setup baudrate divisor and delays */
	if (switch_ck) {
		cqspi->sclk = sclk;
		cqspi_config_baudrate_div(cqspi);
		cqspi_delay(f_pdata);
		cqspi_readdata_capture(cqspi, !cqspi->rclk_en,
				       f_pdata->read_delay);
	}

	if (switch_cs || switch_ck)
		cqspi_controller_enable(cqspi, 1);
}

static ssize_t cqspi_write(struct cqspi_flash_pdata *f_pdata,
			   const struct spi_mem_op *op)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	loff_t to = op->addr.val;
	size_t len = op->data.nbytes;
	const u_char *buf = op->data.buf.out;
	int ret;

	ret = cqspi_set_protocol(f_pdata, op);
	if (ret)
		return ret;

	ret = cqspi_write_setup(f_pdata, op);
	if (ret)
		return ret;

	/*
	 * Some flashes like the Cypress Semper flash expect a dummy 4-byte
	 * address (all 0s) with the read status register command in DTR mode.
	 * But this controller does not support sending dummy address bytes to
	 * the flash when it is polling the write completion register in DTR
	 * mode. So, we can not use direct mode when in DTR mode for writing
	 * data.
	 */
	if (!f_pdata->dtr && cqspi->use_direct_mode &&
	    ((to + len) <= cqspi->ahb_size)) {
		memcpy_toio(cqspi->ahb_base + to, buf, len);
		return cqspi_wait_idle(cqspi);
	}

	return cqspi_indirect_write_execute(f_pdata, to, buf, len);
}

static void cqspi_rx_dma_callback(void *param)
{
	struct cqspi_st *cqspi = param;

	complete(&cqspi->rx_dma_complete);
}

static int cqspi_direct_read_execute(struct cqspi_flash_pdata *f_pdata,
				     u_char *buf, loff_t from, size_t len)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	struct device *dev = &cqspi->pdev->dev;
	enum dma_ctrl_flags flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;
	dma_addr_t dma_src = (dma_addr_t)cqspi->mmap_phys_base + from;
	int ret = 0;
	struct dma_async_tx_descriptor *tx;
	dma_cookie_t cookie;
	dma_addr_t dma_dst;
	struct device *ddev;

	if (!cqspi->rx_chan || !virt_addr_valid(buf)) {
		memcpy_fromio(buf, cqspi->ahb_base + from, len);
		return 0;
	}

	ddev = cqspi->rx_chan->device->dev;
	dma_dst = dma_map_single(ddev, buf, len, DMA_FROM_DEVICE);
	if (dma_mapping_error(ddev, dma_dst)) {
		dev_err(dev, "dma mapping failed\n");
		return -ENOMEM;
	}
	tx = dmaengine_prep_dma_memcpy(cqspi->rx_chan, dma_dst, dma_src,
				       len, flags);
	if (!tx) {
		dev_err(dev, "device_prep_dma_memcpy error\n");
		ret = -EIO;
		goto err_unmap;
	}

	tx->callback = cqspi_rx_dma_callback;
	tx->callback_param = cqspi;
	cookie = tx->tx_submit(tx);
	reinit_completion(&cqspi->rx_dma_complete);

	ret = dma_submit_error(cookie);
	if (ret) {
		dev_err(dev, "dma_submit_error %d\n", cookie);
		ret = -EIO;
		goto err_unmap;
	}

	dma_async_issue_pending(cqspi->rx_chan);
	if (!wait_for_completion_timeout(&cqspi->rx_dma_complete,
					 msecs_to_jiffies(max_t(size_t, len, 500)))) {
		dmaengine_terminate_sync(cqspi->rx_chan);
		dev_err(dev, "DMA wait_for_completion_timeout\n");
		ret = -ETIMEDOUT;
		goto err_unmap;
	}

err_unmap:
	dma_unmap_single(ddev, dma_dst, len, DMA_FROM_DEVICE);

	return ret;
}

static ssize_t cqspi_read(struct cqspi_flash_pdata *f_pdata,
			  const struct spi_mem_op *op)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	struct device *dev = &cqspi->pdev->dev;
	const struct cqspi_driver_platdata *ddata;
	loff_t from = op->addr.val;
	size_t len = op->data.nbytes;
	u_char *buf = op->data.buf.in;
	u64 dma_align = (u64)(uintptr_t)buf;
	int ret;

	ddata = of_device_get_match_data(dev);
	ret = cqspi_set_protocol(f_pdata, op);
	if (ret)
		return ret;

	ret = cqspi_read_setup(f_pdata, op);
	if (ret)
		return ret;

	if (cqspi->use_direct_mode && ((from + len) <= cqspi->ahb_size))
		return cqspi_direct_read_execute(f_pdata, buf, from, len);

	if (cqspi->use_dma_read && ddata && ddata->indirect_read_dma &&
	    virt_addr_valid(buf) && ((dma_align & CQSPI_DMA_UNALIGN) == 0))
		return ddata->indirect_read_dma(f_pdata, buf, from, len);

	return cqspi_indirect_read_execute(f_pdata, buf, from, len);
}

static int cqspi_mem_process(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct cqspi_st *cqspi = spi_master_get_devdata(mem->spi->master);
	struct cqspi_flash_pdata *f_pdata;

	f_pdata = &cqspi->f_pdata[mem->spi->chip_select];
	cqspi_configure(f_pdata, mem->spi->max_speed_hz);

	if (op->data.dir == SPI_MEM_DATA_IN && op->data.buf.in) {
		if (!op->addr.nbytes)
			return cqspi_command_read(f_pdata, op);

		return cqspi_read(f_pdata, op);
	}

	if (!op->addr.nbytes || !op->data.buf.out)
		return cqspi_command_write(f_pdata, op);

	return cqspi_write(f_pdata, op);
}

static int cqspi_exec_mem_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct cqspi_st *cqspi = spi_master_get_devdata(mem->spi->master);
	struct cqspi_flash_pdata *f_pdata;
	int ret;

	f_pdata = &cqspi->f_pdata[mem->spi->chip_select];

	if (mem->spi->master->flags & SPI_MASTER_U_PAGE)
		f_pdata->cs = CQSPI_CS_UPPER;
	else
		f_pdata->cs = CQSPI_CS_LOWER;

	if (op->cmd.dtr &&
	    (!op->addr.nbytes || op->addr.dtr) &&
	    (!op->data.nbytes || op->data.dtr)) {
		ret = cqspi_setup_edgemode(mem, op);
		if (ret)
			return ret;
	} else {
		if (cqspi->clk_tuned)
			cqspi_setup_sdrmode(cqspi);
	}

	if (f_pdata->dtr && !cqspi->tuning_complete.done &&
	    !wait_for_completion_timeout(&cqspi->tuning_complete,
		msecs_to_jiffies(CQSPI_TUNING_TIMEOUT_MS))) {
		return -ETIMEDOUT;
	}

	ret = cqspi_mem_process(mem, op);
	if (ret)
		dev_err(&mem->spi->dev, "operation failed with %d\n", ret);

	return ret;
}

static bool cqspi_supports_mem_op(struct spi_mem *mem,
				  const struct spi_mem_op *op)
{
	bool all_true, all_false;

	/*
	 * op->dummy.dtr is required for converting nbytes into ncycles.
	 * Also, don't check the dtr field of the op phase having zero nbytes.
	 */
	all_true = op->cmd.dtr &&
		   (!op->addr.nbytes || op->addr.dtr) &&
		   (!op->dummy.nbytes || op->dummy.dtr) &&
		   (!op->data.nbytes || op->data.dtr);

	all_false = !op->cmd.dtr && !op->addr.dtr && !op->dummy.dtr &&
		    !op->data.dtr;

	if (all_true) {
		/* Right now we only support 8-8-8 DTR mode. */
		if (op->cmd.nbytes && op->cmd.buswidth != 8)
			return false;
		if (op->addr.nbytes && op->addr.buswidth != 8)
			return false;
		if (op->data.nbytes && op->data.buswidth != 8)
			return false;
	} else if (all_false) {
		/* Only 1-1-X ops are supported without DTR */
		if (op->cmd.nbytes && op->cmd.buswidth > 1)
			return false;
		if (op->addr.nbytes && op->addr.buswidth > 1)
			return false;
	} else {
		/* Mixed DTR modes are not supported. */
		return false;
	}

	if (all_true)
		return spi_mem_dtr_supports_op(mem, op);
	else
		return spi_mem_default_supports_op(mem, op);
}

static int cqspi_of_get_flash_pdata(struct platform_device *pdev,
				    struct cqspi_flash_pdata *f_pdata,
				    struct device_node *np)
{
	if (of_property_read_u32(np, "cdns,read-delay", &f_pdata->read_delay)) {
		dev_err(&pdev->dev, "couldn't determine read-delay\n");
		return -ENXIO;
	}

	if (of_property_read_u32(np, "cdns,tshsl-ns", &f_pdata->tshsl_ns)) {
		dev_err(&pdev->dev, "couldn't determine tshsl-ns\n");
		return -ENXIO;
	}

	if (of_property_read_u32(np, "cdns,tsd2d-ns", &f_pdata->tsd2d_ns)) {
		dev_err(&pdev->dev, "couldn't determine tsd2d-ns\n");
		return -ENXIO;
	}

	if (of_property_read_u32(np, "cdns,tchsh-ns", &f_pdata->tchsh_ns)) {
		dev_err(&pdev->dev, "couldn't determine tchsh-ns\n");
		return -ENXIO;
	}

	if (of_property_read_u32(np, "cdns,tslch-ns", &f_pdata->tslch_ns)) {
		dev_err(&pdev->dev, "couldn't determine tslch-ns\n");
		return -ENXIO;
	}

	if (of_property_read_u32(np, "spi-max-frequency", &f_pdata->clk_rate)) {
		dev_err(&pdev->dev, "couldn't determine spi-max-frequency\n");
		return -ENXIO;
	}

	return 0;
}

static int cqspi_of_get_pdata(struct cqspi_st *cqspi)
{
	struct device *dev = &cqspi->pdev->dev;
	struct device_node *np = dev->of_node;
	u32 id[2];

	cqspi->is_decoded_cs = of_property_read_bool(np, "cdns,is-decoded-cs");

	if (of_property_read_u32(np, "cdns,fifo-depth", &cqspi->fifo_depth)) {
		dev_err(dev, "couldn't determine fifo-depth\n");
		return -ENXIO;
	}

	if (of_property_read_u32(np, "cdns,fifo-width", &cqspi->fifo_width)) {
		dev_err(dev, "couldn't determine fifo-width\n");
		return -ENXIO;
	}

	if (of_property_read_u32(np, "cdns,trigger-address",
				 &cqspi->trigger_address)) {
		dev_err(dev, "couldn't determine trigger-address\n");
		return -ENXIO;
	}

	if (of_property_read_u32(np, "num-cs", &cqspi->num_chipselect))
		cqspi->num_chipselect = CQSPI_MAX_CHIPSELECT;

	cqspi->rclk_en = of_property_read_bool(np, "cdns,rclk-en");

	if (!of_property_read_u32_array(np, "power-domains", id,
					ARRAY_SIZE(id)))
		cqspi->pd_dev_id = id[1];

	return 0;
}

static void cqspi_controller_init(struct cqspi_st *cqspi)
{
	u32 reg;

	cqspi_controller_enable(cqspi, 0);

	/* Configure the remap address register, no remap */
	writel(0, cqspi->iobase + CQSPI_REG_REMAP);

	/* Reset the Delay lines */
	writel(CQSPI_REG_PHY_CONFIG_RESET_FLD_MASK,
	       cqspi->iobase + CQSPI_REG_PHY_CONFIG);

	/* Disable all interrupts. */
	writel(0, cqspi->iobase + CQSPI_REG_IRQMASK);

	/* Configure the SRAM split to 1:1 . */
	writel(cqspi->fifo_depth / 2, cqspi->iobase + CQSPI_REG_SRAMPARTITION);

	/* Load indirect trigger address. */
	writel(cqspi->trigger_address,
	       cqspi->iobase + CQSPI_REG_INDIRECTTRIGGER);

	/* Program read watermark -- 1/2 of the FIFO. */
	writel(cqspi->fifo_depth * cqspi->fifo_width / 2,
	       cqspi->iobase + CQSPI_REG_INDIRECTRDWATERMARK);
	/* Program write watermark -- 1/8 of the FIFO. */
	writel(cqspi->fifo_depth * cqspi->fifo_width / 8,
	       cqspi->iobase + CQSPI_REG_INDIRECTWRWATERMARK);

	/* Disable direct access controller */
	if (!cqspi->use_direct_mode) {
		reg = readl(cqspi->iobase + CQSPI_REG_CONFIG);
		reg &= ~CQSPI_REG_CONFIG_ENB_DIR_ACC_CTRL;
		writel(reg, cqspi->iobase + CQSPI_REG_CONFIG);
	}

	/* Enable DMA interface */
	if (cqspi->use_dma_read) {
		reg = readl(cqspi->iobase + CQSPI_REG_CONFIG);
		reg &= ~CQSPI_REG_CONFIG_PHY_ENABLE_MASK;
		writel(reg, cqspi->iobase + CQSPI_REG_CONFIG);
	}

	cqspi_controller_enable(cqspi, 1);
}

static int cqspi_request_mmap_dma(struct cqspi_st *cqspi)
{
	dma_cap_mask_t mask;

	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);

	cqspi->rx_chan = dma_request_chan_by_mask(&mask);
	if (IS_ERR(cqspi->rx_chan)) {
		int ret = PTR_ERR(cqspi->rx_chan);

		cqspi->rx_chan = NULL;
		return dev_err_probe(&cqspi->pdev->dev, ret, "No Rx DMA available\n");
	}
	init_completion(&cqspi->rx_dma_complete);

	return 0;
}

static const char *cqspi_get_name(struct spi_mem *mem)
{
	struct cqspi_st *cqspi = spi_master_get_devdata(mem->spi->master);
	struct device *dev = &cqspi->pdev->dev;

	return devm_kasprintf(dev, GFP_KERNEL, "%s.%d", dev_name(dev), mem->spi->chip_select);
}

static const struct spi_controller_mem_ops cqspi_mem_ops = {
	.exec_op = cqspi_exec_mem_op,
	.get_name = cqspi_get_name,
	.supports_op = cqspi_supports_mem_op,
};

static int cqspi_setup_flash(struct cqspi_st *cqspi)
{
	struct platform_device *pdev = cqspi->pdev;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct cqspi_flash_pdata *f_pdata;
	unsigned int cs;
	int ret;

	/* Get flash device data */
	for_each_available_child_of_node(dev->of_node, np) {
		ret = of_property_read_u32(np, "reg", &cs);
		if (ret) {
			dev_err(dev, "Couldn't determine chip select.\n");
			of_node_put(np);
			return ret;
		}

		if (cs >= CQSPI_MAX_CHIPSELECT) {
			dev_err(dev, "Chip select %d out of range.\n", cs);
			of_node_put(np);
			return -EINVAL;
		}

		f_pdata = &cqspi->f_pdata[cs];
		f_pdata->cqspi = cqspi;
		f_pdata->cs = cs;

		ret = cqspi_of_get_flash_pdata(pdev, f_pdata, np);
		if (ret) {
			of_node_put(np);
			return ret;
		}
	}

	return 0;
}

static int cqspi_probe(struct platform_device *pdev)
{
	const struct cqspi_driver_platdata *ddata;
	struct reset_control *rstc, *rstc_ocp;
	struct device *dev = &pdev->dev;
	struct spi_master *master;
	struct resource *res_ahb;
	struct cqspi_st *cqspi;
	struct resource *res;
	int ret;
	int irq;
	u32 idcode;
	u32 version;

	master = spi_alloc_master(&pdev->dev, sizeof(*cqspi));
	if (!master) {
		dev_err(&pdev->dev, "spi_alloc_master failed\n");
		return -ENOMEM;
	}
	master->mode_bits = SPI_RX_QUAD | SPI_RX_DUAL;
	master->mem_ops = &cqspi_mem_ops;
	master->dev.of_node = pdev->dev.of_node;

	cqspi = spi_master_get_devdata(master);

	cqspi->pdev = pdev;
	platform_set_drvdata(pdev, cqspi);

	/* Obtain configuration from OF. */
	ret = cqspi_of_get_pdata(cqspi);
	if (ret) {
		dev_err(dev, "Cannot get mandatory OF data.\n");
		ret = -ENODEV;
		goto probe_master_put;
	}

	/* Obtain QSPI clock. */
	cqspi->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(cqspi->clk)) {
		dev_err(dev, "Cannot claim QSPI clock.\n");
		ret = PTR_ERR(cqspi->clk);
		goto probe_master_put;
	}

	/* Obtain and remap controller address. */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	cqspi->iobase = devm_ioremap_resource(dev, res);
	if (IS_ERR(cqspi->iobase)) {
		dev_err(dev, "Cannot remap controller address.\n");
		ret = PTR_ERR(cqspi->iobase);
		goto probe_master_put;
	}

	/* Obtain and remap AHB address. */
	res_ahb = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	cqspi->ahb_base = devm_ioremap_resource(dev, res_ahb);
	if (IS_ERR(cqspi->ahb_base)) {
		dev_err(dev, "Cannot remap AHB address.\n");
		ret = PTR_ERR(cqspi->ahb_base);
		goto probe_master_put;
	}
	cqspi->mmap_phys_base = (dma_addr_t)res_ahb->start;
	cqspi->ahb_size = resource_size(res_ahb);

	init_completion(&cqspi->transfer_complete);
	init_completion(&cqspi->tuning_complete);

	/* Obtain IRQ line. */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = -ENXIO;
		goto probe_master_put;
	}

	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		pm_runtime_put_noidle(dev);
		goto probe_master_put;
	}

	ret = clk_prepare_enable(cqspi->clk);
	if (ret) {
		dev_err(dev, "Cannot enable QSPI clock.\n");
		goto probe_clk_failed;
	}

	/* Obtain QSPI reset control */
	rstc = devm_reset_control_get_optional_exclusive(dev, "qspi");
	if (IS_ERR(rstc)) {
		ret = PTR_ERR(rstc);
		dev_err(dev, "Cannot get QSPI reset.\n");
		goto probe_reset_failed;
	}

	rstc_ocp = devm_reset_control_get_optional_exclusive(dev, "qspi-ocp");
	if (IS_ERR(rstc_ocp)) {
		ret = PTR_ERR(rstc_ocp);
		dev_err(dev, "Cannot get QSPI OCP reset.\n");
		goto probe_reset_failed;
	}

	reset_control_assert(rstc);
	reset_control_deassert(rstc);

	reset_control_assert(rstc_ocp);
	reset_control_deassert(rstc_ocp);

	cqspi->master_ref_clk_hz = clk_get_rate(cqspi->clk);
	master->max_speed_hz = cqspi->master_ref_clk_hz;
	ddata  = of_device_get_match_data(dev);
	if (ddata) {
		if (ddata->quirks & CQSPI_NEEDS_WR_DELAY)
			cqspi->wr_delay = 50 * DIV_ROUND_UP(NSEC_PER_SEC,
						cqspi->master_ref_clk_hz);
		if (ddata->hwcaps_mask & CQSPI_SUPPORTS_OCTAL)
			master->mode_bits |= SPI_RX_OCTAL | SPI_TX_OCTAL;
		if (!(ddata->quirks & CQSPI_DISABLE_DAC_MODE))
			cqspi->use_direct_mode = true;
		if (ddata->quirks & CQSPI_SUPPORT_EXTERNAL_DMA)
			cqspi->use_dma_read = true;

		if (of_device_is_compatible(pdev->dev.of_node,
					    "xlnx,versal-ospi-1.0")) {
			dma_set_mask(&pdev->dev, DMA_BIT_MASK(64));
			cqspi->dll_mode = CQSPI_DLL_MODE_BYPASS;
			ret = zynqmp_pm_get_chipid(&idcode, &version);
			if (ret < 0) {
				dev_err(dev, "Cannot get chipid is %d\n", ret);
				goto probe_clk_failed;
			}
			if ((version & SILICON_VER_MASK) != SILICON_VER_1) {
				cqspi->dll_mode = CQSPI_DLL_MODE_MASTER;
				if (cqspi->master_ref_clk_hz >= TAP_GRAN_SEL_MIN_FREQ)
					writel(0x1, cqspi->iobase + CQSPI_REG_VERSAL_ECO);
			}
		}
	}

	ret = devm_request_irq(dev, irq, cqspi_irq_handler, 0,
			       pdev->name, cqspi);
	if (ret) {
		dev_err(dev, "Cannot request IRQ.\n");
		goto probe_reset_failed;
	}

	cqspi_wait_idle(cqspi);
	cqspi_controller_init(cqspi);
	cqspi->current_cs = -1;
	cqspi->sclk = 0;
	cqspi->extra_dummy = false;
	cqspi->clk_tuned = false;

	master->num_chipselect = cqspi->num_chipselect;

	ret = cqspi_setup_flash(cqspi);
	if (ret) {
		dev_err(dev, "failed to setup flash parameters %d\n", ret);
		goto probe_setup_failed;
	}

	if (ddata->device_reset) {
		ret = ddata->device_reset(cqspi, CQSPI_RESET_TYPE_HWPIN);
		if (ret)
			goto probe_setup_failed;
	}

	if (cqspi->use_direct_mode) {
		ret = cqspi_request_mmap_dma(cqspi);
		if (ret == -EPROBE_DEFER)
			goto probe_setup_failed;
	}

	ret = devm_spi_register_master(dev, master);
	if (ret) {
		dev_err(&pdev->dev, "failed to register SPI ctlr %d\n", ret);
		goto probe_setup_failed;
	}

	return 0;
probe_setup_failed:
	cqspi_controller_enable(cqspi, 0);
probe_reset_failed:
	clk_disable_unprepare(cqspi->clk);
probe_clk_failed:
	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);
probe_master_put:
	spi_master_put(master);
	return ret;
}

static int cqspi_remove(struct platform_device *pdev)
{
	struct cqspi_st *cqspi = platform_get_drvdata(pdev);

	cqspi_controller_enable(cqspi, 0);

	if (cqspi->rx_chan)
		dma_release_channel(cqspi->rx_chan);

	clk_disable_unprepare(cqspi->clk);

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int cqspi_suspend(struct device *dev)
{
	struct cqspi_st *cqspi = dev_get_drvdata(dev);

	if (cqspi->clk_tuned && !cqspi->tuning_complete.done &&
	    !wait_for_completion_timeout(&cqspi->tuning_complete,
		msecs_to_jiffies(CQSPI_TUNING_TIMEOUT_MS))) {
		return -ETIMEDOUT;
	}

	cqspi_controller_enable(cqspi, 0);
	return 0;
}

static int cqspi_resume(struct device *dev)
{
	struct cqspi_st *cqspi = dev_get_drvdata(dev);
	u32 ret;

	cqspi_controller_init(cqspi);
	cqspi->current_cs = -1;
	cqspi->sclk = 0;
	cqspi->extra_dummy = false;
	cqspi->clk_tuned = false;

	ret = cqspi_setup_flash(cqspi);
	if (ret) {
		dev_err(dev, "failed to setup flash parameters %d\n", ret);
		return ret;
	}

	ret = zynqmp_pm_ospi_mux_select(cqspi->pd_dev_id,
					PM_OSPI_MUX_SEL_LINEAR);
	if (ret)
		return ret;

	/* Set the direction as output and enable the output */
	gpio_direction_output(cqspi->gpio, 1);
	udelay(1);

	/* Set value 0 to pin */
	gpio_set_value(cqspi->gpio, 0);
	udelay(10);

	/* Set value 1 to pin */
	gpio_set_value(cqspi->gpio, 1);
	udelay(35);

	return 0;
}

static const struct dev_pm_ops cqspi__dev_pm_ops = {
	.suspend = cqspi_suspend,
	.resume = cqspi_resume,
};

#define CQSPI_DEV_PM_OPS	(&cqspi__dev_pm_ops)
#else
#define CQSPI_DEV_PM_OPS	NULL
#endif

static const struct cqspi_driver_platdata cdns_qspi = {
	.quirks = CQSPI_DISABLE_DAC_MODE,
};

static const struct cqspi_driver_platdata k2g_qspi = {
	.quirks = CQSPI_NEEDS_WR_DELAY,
};

static const struct cqspi_driver_platdata am654_ospi = {
	.hwcaps_mask = CQSPI_SUPPORTS_OCTAL,
	.quirks = CQSPI_NEEDS_WR_DELAY,
};

static const struct cqspi_driver_platdata intel_lgm_qspi = {
	.quirks = CQSPI_DISABLE_DAC_MODE,
};

static const struct cqspi_driver_platdata versal_ospi = {
	.hwcaps_mask = CQSPI_SUPPORTS_OCTAL,
	.quirks = CQSPI_DISABLE_DAC_MODE | CQSPI_SUPPORT_EXTERNAL_DMA,
	.indirect_read_dma = cqspi_versal_indirect_read_dma,
	.get_dma_status = cqspi_get_versal_dma_status,
	.device_reset = cqspi_versal_device_reset,
};

static const struct of_device_id cqspi_dt_ids[] = {
	{
		.compatible = "cdns,qspi-nor",
		.data = &cdns_qspi,
	},
	{
		.compatible = "ti,k2g-qspi",
		.data = &k2g_qspi,
	},
	{
		.compatible = "ti,am654-ospi",
		.data = &am654_ospi,
	},
	{
		.compatible = "intel,lgm-qspi",
		.data = &intel_lgm_qspi,
	},
	{
		.compatible = "xlnx,versal-ospi-1.0",
		.data = (void *)&versal_ospi,
	},
	{ /* end of table */ }
};

MODULE_DEVICE_TABLE(of, cqspi_dt_ids);

static struct platform_driver cqspi_platform_driver = {
	.probe = cqspi_probe,
	.remove = cqspi_remove,
	.driver = {
		.name = CQSPI_NAME,
		.pm = CQSPI_DEV_PM_OPS,
		.of_match_table = cqspi_dt_ids,
	},
};

module_platform_driver(cqspi_platform_driver);

MODULE_DESCRIPTION("Cadence QSPI Controller Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" CQSPI_NAME);
MODULE_AUTHOR("Ley Foon Tan <lftan@altera.com>");
MODULE_AUTHOR("Graham Moore <grmoore@opensource.altera.com>");
MODULE_AUTHOR("Vadivel Murugan R <vadivel.muruganx.ramuthevar@intel.com>");
MODULE_AUTHOR("Vignesh Raghavendra <vigneshr@ti.com>");
MODULE_AUTHOR("Pratyush Yadav <p.yadav@ti.com>");
