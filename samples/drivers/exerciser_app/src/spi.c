#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <string.h>
#include <zephyr/drivers/spi.h>
#include <soc_common.h>
#include "exerciser_app.h"

#define Mhz		1000000
#define Khz		1000

/* master_spi and slave_spi aliases are defined in
 * overlay files to use different SPI instance if needed.
 */

#define SPIDW_NODE	DT_ALIAS(master_spi)

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* default SPI master SS(slave select) is H/W controlled,
 * enable this to use as S/W controlled using gpio.
 */
#define SPI_MASTER_SS_SW_CONTROLLED_GPIO   0

/* scheduling priority used by each thread,
 * as we are testing Loopback on the same board,
 * make sure master priority is higher than slave priority.
 */
#define MASTER_PRIORITY 6

/* delay between greetings (in ms) */
#define SLEEPTIME 1000

//K_THREAD_STACK_DEFINE(MasterT_stack, STACKSIZE);
//static struct k_thread MasterT_data;

/* Master and Slave buffer size */
#define BUFF_SIZE  200

/* Master and Slave buffer word size */
#define SPI_WORD_SIZE 8

/* Master and Slave buffer frequency */
#define SPI_FREQUENCY (1 * Mhz)

/* Master and Slave buffers */
static uint32_t master_txdata[BUFF_SIZE];
static uint32_t master_rxdata[BUFF_SIZE];

/*
 * Send/Receive data through master spi
 */
int master_spi_transceive(const struct device *dev,
				 struct spi_cs_control *cs)
{
	struct spi_config cnfg;
	int ret;

	cnfg.frequency = SPI_FREQUENCY;
	cnfg.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(SPI_WORD_SIZE) | SPI_MODE_LOOP ;
	cnfg.slave = 0;
	cnfg.cs = *cs;

	int length = (BUFF_SIZE) * sizeof(master_txdata[0]);

	struct spi_buf tx_buf = {
		.buf = master_txdata,
		.len = length
	};

	struct spi_buf_set tx_bufset = {
		.buffers = &tx_buf,
		.count = 1
	};

	struct spi_buf rx_buf = {
		.buf = master_rxdata,
		.len = length
	};
	struct spi_buf_set rx_bufset = {
		.buffers = &rx_buf,
		.count = 1
	};

	ret = spi_transceive(dev, &cnfg, &tx_bufset, &rx_bufset);
	if (ret) {
		printf("[ SPI ]ERROR: SPI=%p transceive: %d\n", dev, ret);
	}
	printf("[ SPI ]Master wrote: %08x %08x %08x %08x %08x\n",
		master_txdata[0], master_txdata[1], master_txdata[2],
		master_txdata[3], master_txdata[4]);
	printf("[ SPI ]Master receive: %08x %08x %08x %08x %08x\n",
		master_rxdata[0], master_rxdata[1], master_rxdata[2],
		master_rxdata[3], master_rxdata[4]);

	ret = memcmp(master_txdata, master_rxdata, length);
	if (ret) {
		printf("[ SPI ]ERROR: SPI Master RX & Slave TX DATA NOT MATCHING: %d\n", ret);
	} else {
		printf("[ SPI ]SUCCESS: SPI Master RX & Slave TX DATA IS MATCHING: %d\n", ret);
	}

	return ret;
}

/* DMA configurations */
#define DMA_CTRL_ACK_TYPE_Pos          (16U)
#define DMA_CTRL_ENA                   (1U << 4)

#define HE_DMA_SEL_LPSPI_Pos           (4)
#define HE_DMA_SEL_LPSPI_Msk           (0x3U << HE_DMA_SEL_LPSPI_Pos)

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(dma2), arm_dma_pl330, okay) /* dma2 */

#if (IS_ENABLED(CONFIG_SOC_SERIES_E1C) ||  \
	IS_ENABLED(CONFIG_SOC_SERIES_B1)) /* E1C/B1 dma2 */

#if DT_NODE_HAS_PROP(DT_NODELABEL(lpspi0), dmas) /* E1C/B1 LPSPI0 dma2 */
static void configure_lpspi0_for_dma2(void)
{
	/* Enable LPSPI EVTRTR channel */
	#define LPSPI_DMA_RX_PERIPH_REQ		12
	#define LPSPI_DMA_TX_PERIPH_REQ		13
	#define LPSPI_DMA_GROUP             1

	uint32_t regdata;

	printf("\n[ SPI ] configure lpspi0 for dma2\n");

	/* select DMA2 group 1 for LPSPI (default dma2 group 1) */
	sys_clear_bits(M55HE_CFG_HE_DMA_SEL, HE_DMA_SEL_LPSPI_Msk);

	/* channel enable for LPSPI-RX */
	sys_write32(DMA_CTRL_ENA |
			(0 << DMA_CTRL_ACK_TYPE_Pos)|
			(LPSPI_DMA_GROUP),
			EVTRTRLOCAL_DMA_CTRL0 + (LPSPI_DMA_RX_PERIPH_REQ * 4));

	/* DMA Handshake enable LPSPI-RX */
	regdata = sys_read32(EVTRTRLOCAL_DMA_ACK_TYPE0 + (LPSPI_DMA_GROUP * 4));
	regdata |= (1 << LPSPI_DMA_RX_PERIPH_REQ);
	sys_write32(regdata, EVTRTRLOCAL_DMA_ACK_TYPE0 + (LPSPI_DMA_GROUP * 4));

	/* channel enable for LPSPI-TX */
	sys_write32(DMA_CTRL_ENA |
			(0 << DMA_CTRL_ACK_TYPE_Pos)|
			(LPSPI_DMA_GROUP),
			EVTRTRLOCAL_DMA_CTRL0 + (LPSPI_DMA_TX_PERIPH_REQ * 4));

	/* DMA Handshake enable LPSPI-TX */
	regdata = sys_read32(EVTRTRLOCAL_DMA_ACK_TYPE0 + (LPSPI_DMA_GROUP * 4));
	regdata |= (1 << LPSPI_DMA_TX_PERIPH_REQ);
	sys_write32(regdata, EVTRTRLOCAL_DMA_ACK_TYPE0 + (LPSPI_DMA_GROUP * 4));
}
#endif /* E1C/B1 LPSPI0 dma2 */

#if DT_NODE_HAS_PROP(DT_NODELABEL(spi0), dmas) /* E1C/B1 SPI0 dma2 */
static void configure_spi0_for_dma2(void)
{
	/* Enable SPI0 dma2 EVTRTR channel */
	#define SPI0_DMA_RX_PERIPH_REQ         16
	#define SPI0_DMA_TX_PERIPH_REQ         20
	#define SPI0_DMA_GROUP                 2

	uint32_t regdata;

	printf("\n[ SPI ] configure spi0 for dma2\n");

	/* channel enable SPI0-RX */
	sys_write32(DMA_CTRL_ENA |
			(0 << DMA_CTRL_ACK_TYPE_Pos)|
			(SPI0_DMA_GROUP),
			EVTRTRLOCAL_DMA_CTRL0 + (SPI0_DMA_RX_PERIPH_REQ * 4));

	/* DMA Handshake enable SPI0-RX */
	regdata = sys_read32(EVTRTRLOCAL_DMA_ACK_TYPE0 + (SPI0_DMA_GROUP * 4));
	regdata |= (1 << SPI0_DMA_RX_PERIPH_REQ);
	sys_write32(regdata, EVTRTRLOCAL_DMA_ACK_TYPE0 + (SPI0_DMA_GROUP * 4));

	/* channel enable SPI0-TX */
	sys_write32(DMA_CTRL_ENA |
			(0 << DMA_CTRL_ACK_TYPE_Pos)|
			(SPI0_DMA_GROUP),
			EVTRTRLOCAL_DMA_CTRL0 + (SPI0_DMA_TX_PERIPH_REQ * 4));

	/* DMA Handshake enable SPI0-TX */
	regdata = sys_read32(EVTRTRLOCAL_DMA_ACK_TYPE0 + (SPI0_DMA_GROUP * 4));
	regdata |= (1 << SPI0_DMA_TX_PERIPH_REQ);
	sys_write32(regdata, EVTRTRLOCAL_DMA_ACK_TYPE0 + (SPI0_DMA_GROUP * 4));
}
#endif /* E1C/B1 SPI0 dma2 */

#endif /* E1C/B1 dma2 */
#endif /* dma2 */

static void prepare_data(uint32_t *data, uint16_t def_mask)
{
	for (uint32_t cnt = 0; cnt < BUFF_SIZE; cnt++) {
		data[cnt] = (def_mask << 16) | cnt;
	}
}

void spi_thread(void)
{
	const struct device *const dev = DEVICE_DT_GET(SPIDW_NODE);
	int ret;

	//ARG_UNUSED(p1);
	//ARG_UNUSED(p2);
	//ARG_UNUSED(p3);
	
	#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(dma2), arm_dma_pl330, okay) /* dma2 */

	#if (IS_ENABLED(CONFIG_SOC_SERIES_E1C) || \
	IS_ENABLED(CONFIG_SOC_SERIES_B1)) /* E1C/B1 dma2 */

	#if DT_NODE_HAS_PROP(DT_NODELABEL(lpspi0), dmas) /* E1C/B1 LPSPI0 dma2 */
	configure_lpspi0_for_dma2();
	#endif

	#if DT_NODE_HAS_PROP(DT_NODELABEL(spi0), dmas) /* E1C/B1 SPI0 dma2 */
	configure_spi0_for_dma2();
	#endif
	/* end E1C/B1 dma2 */

	#endif
	#endif /* dma2 */

	if (!device_is_ready(dev)) {
		printf("[ SPI ]%s: Master device not ready.\n", dev->name);
		return;
	}
	prepare_data(master_txdata, 0xA5A5);

#if SPI_MASTER_SS_SW_CONTROLLED_GPIO /* SPI master SS as S/W controlled using gpio */
	struct spi_cs_control cs_ctrl = (struct spi_cs_control){
		.gpio  = GPIO_DT_SPEC_GET(SPIDW_NODE, cs_gpios),
		.delay = 100u, /* k_busy_wait(uint32_t usec_to_wait) */
	};
#else /* SPI master SS as H/W controlled */
	struct spi_cs_control cs_ctrl = {0};
#endif /* SPI_MASTER_SS_SW_CONTROLLED_GPIO */

	while (1) {
		ret = master_spi_transceive(dev, &cs_ctrl);
		k_msleep(SLEEPTIME);
		if (ret < 0) {
			printf("[ SPI ]Stopping the Master Thread due to error\n");
			return;
		}
	}
	printf("[ SPI ]Master Transfer Successfully Completed\n");
}
