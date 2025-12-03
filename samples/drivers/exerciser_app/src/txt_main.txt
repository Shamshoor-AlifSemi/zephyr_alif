/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <string.h>
#include <zephyr/drivers/spi.h>
#include <soc.h>
#include <stdio.h>

#include <zephyr/drivers/gpio.h>
/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000
/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#define Mhz		1000000
#define Khz		1000

/* master_spi and slave_spi aliases are defined in
 * overlay files to use different SPI instance if needed.
 */
#include <sample_usbd.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>

#define RING_BUF_SIZE 1024

uint8_t ring_buffer[RING_BUF_SIZE];
struct ring_buf ringbuf;
static bool rx_throttled;
static struct usbd_contex *sample_usbd;

#define SPIDW_NODE	DT_ALIAS(master_spi)

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define MASTER_PRIORITY 7
#define SLAVE_PRIORITY 6

/* delay between greetings (in ms) */
#define SLEEPTIME 1000

/*Sd-header files*/
#include <zephyr/storage/disk_access.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>

#if defined(CONFIG_FAT_FILESYSTEM_ELM)

#include <ff.h>

/*
 *  Note the fatfs library is able to mount only strings inside _VOLUME_STRS
 *  in ffconf.h
 */
#define DISK_DRIVE_NAME "SD"
#define DISK_MOUNT_PT "/"DISK_DRIVE_NAME":"

FATFS __alif_ns_section fat_fs __aligned(512);
uint8_t data_rw_buf[1025];

/* mounting info */
static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
};

#elif defined(CONFIG_FILE_SYSTEM_EXT2)

#include <zephyr/fs/ext2.h>

#define DISK_DRIVE_NAME "SDMMC"
#define DISK_MOUNT_PT "/ext"

static struct fs_mount_t mp = {
	.type = FS_EXT2,
	.flags = FS_MOUNT_FLAG_NO_FORMAT,
	.storage_dev = (void *)DISK_DRIVE_NAME,
	.mnt_point = "/ext",
};

#endif

/*I2S Header*/
#include <zephyr/audio/codec.h>
#include "codec.h"
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2s.h>

/*OSPI headers*/
#include <zephyr/drivers/flash.h>

#define SPI_FLASH_TEST_REGION_OFFSET 0x0
#define SPI_FLASH_SECTOR_SIZE        4096
#define OSPI_BUFF_SIZE               1024

const struct flash_parameters *flash_param;

/*gpio thread*/
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
const char *gpio4;
K_THREAD_STACK_DEFINE(blinky_stack, STACKSIZE);
static struct k_thread blinky;

/*USB-Dev thread*/
K_THREAD_STACK_DEFINE(usb_dev_stack, STACKSIZE);
static struct k_thread usb_dev;

K_THREAD_STACK_DEFINE(MasterT_stack, STACKSIZE*2);
static struct k_thread MasterT_data;

/*sdmmc thread*/
K_THREAD_STACK_DEFINE(sdmmc_stack, STACKSIZE*4);
static struct k_thread sdmmc;

/*i2s thread*/
K_THREAD_STACK_DEFINE(i2s_stack, STACKSIZE*2);
static struct k_thread i2s;

/*ospi thread*/
K_THREAD_STACK_DEFINE(ospi_stack, STACKSIZE*2);
static struct k_thread ospi;

/* Master and Slave buffer size */
#define BUFF_SIZE  200

/* Master and Slave buffer word size */
#define SPI_WORD_SIZE 8

/* Master and Slave buffer frequency */
#define SPI_FREQUENCY (1 * Mhz)

/* Master and Slave buffer transfers */
#define SPI_NUM_TRANSFERS  100

/* Master and Slave buffers */
static uint32_t master_txdata[BUFF_SIZE];
static uint32_t master_rxdata[BUFF_SIZE];

/*I2S configs*/
#define I2S_RX_NODE  DT_NODELABEL(i2s_rx)
#define I2S_TX_NODE  DT_NODELABEL(i2s_tx)

#define I2S_CODEC_TX  DT_ALIAS(i2s_codec_tx)

#define SAMPLE_FREQUENCY    48000 //4100
#define SAMPLE_BIT_WIDTH    16
#define BYTES_PER_SAMPLE    sizeof(int16_t)
#define NUMBER_OF_CHANNELS  2
/* Such block length provides an echo with the delay of 100 ms. */
#define SAMPLES_PER_BLOCK   ((SAMPLE_FREQUENCY / 10) * NUMBER_OF_CHANNELS)
#define INITIAL_BLOCKS      2
#define TIMEOUT             1000

#define SW0_NODE        DT_ALIAS(sw0)
#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
static struct gpio_dt_spec sw0_spec = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
#endif

#define BLOCK_SIZE  (BYTES_PER_SAMPLE * SAMPLES_PER_BLOCK)
#define BLOCK_COUNT (INITIAL_BLOCKS + 2)
K_MEM_SLAB_DEFINE_STATIC(mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);

static int16_t echo_block[SAMPLES_PER_BLOCK];
static volatile bool echo_enabled = true;
static K_SEM_DEFINE(toggle_transfer, 1, 1);

#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
static void sw0_handler(const struct device *dev, struct gpio_callback *cb,
			uint32_t pins)
{
	bool enable = !echo_enabled;

	echo_enabled = enable;
	printk("Echo %sabled\n", (enable ? "en" : "dis"));
}
#endif

/*
 * Button init for echo
 */
static bool init_buttons(void)
{
	int ret;
#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
	static struct gpio_callback sw0_cb_data;

	if (!gpio_is_ready_dt(&sw0_spec)) {
		printk("%s is not ready\n", sw0_spec.port->name);
		return false;
	}

	ret = gpio_pin_configure_dt(&sw0_spec, GPIO_INPUT);
	if (ret < 0) {
		printk("Failed to configure %s pin %d: %d\n",
		       sw0_spec.port->name, sw0_spec.pin, ret);
		return false;
	}

	ret = gpio_pin_interrupt_configure_dt(&sw0_spec,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		printk("Failed to configure interrupt on %s pin %d: %d\n",
		       sw0_spec.port->name, sw0_spec.pin, ret);
		return false;
	}

	gpio_init_callback(&sw0_cb_data, sw0_handler, BIT(sw0_spec.pin));
	gpio_add_callback(sw0_spec.port, &sw0_cb_data);
	printk("Press \"%s\" to toggle the echo effect\n", sw0_spec.port->name);
#endif
	(void)ret;
	return true;
}

/*
 *configure stream for I2S
 */

static bool configure_streams(const struct device *i2s_dev_rx,
			      const struct device *i2s_dev_tx,
			      const struct i2s_config *config)
{
	int ret;
	ret = i2s_configure(i2s_dev_rx, I2S_DIR_RX, config);
	if (ret < 0) {
		printk("Failed to configure RX stream: %d\n", ret);
		return false;
	}
	ret = i2s_configure(i2s_dev_tx, I2S_DIR_TX, config);
	if (ret < 0) {
		printk("Failed to configure TX stream: %d\n", ret);
		return false;
	}
	return true;
}

/*
 * Preapare the transfer fro I2S
 */
static bool prepare_transfer(const struct device *i2s_dev_rx,
			     const struct device *i2s_dev_tx)
{
	int ret;
	for (int i = 0; i < INITIAL_BLOCKS; ++i) {
		void *mem_block;
		ret = k_mem_slab_alloc(&mem_slab, &mem_block, K_NO_WAIT);
		if (ret < 0) {
			printk("Failed to allocate TX block %d: %d\n", i, ret);
			return false;
		}
		memset(mem_block, 0, BLOCK_SIZE);
		ret = i2s_write(i2s_dev_tx, mem_block, BLOCK_SIZE);
		if (ret < 0) {
			printk("Failed to write block %d: %d\n", i, ret);
			return false;
		}
	}
	return true;
}

/*
 * Trigger command for I2S
 */
static bool trigger_command(const struct device *i2s_dev_rx,
			    const struct device *i2s_dev_tx,
			    enum i2s_trigger_cmd cmd)
{
	int ret;
	ret = i2s_trigger(i2s_dev_rx, I2S_DIR_RX, cmd);
	if (ret < 0) {
		printk("Failed to trigger command %d on RX: %d\n", cmd, ret);
		return false;
	}
	ret = i2s_trigger(i2s_dev_tx, I2S_DIR_TX, cmd);
	if (ret < 0) {
		printk("Failed to trigger command %d on TX: %d\n", cmd, ret);
		return false;
	}
	return true;
}

/*
 *Process block Data
 */
static void process_block_data(void *mem_block, uint32_t number_of_samples)
{
	static bool clear_echo_block;
	if (echo_enabled) {
		for (int i = 0; i < number_of_samples; ++i) {
			int16_t *sample = &((int16_t *)mem_block)[i];
			*sample += echo_block[i];
			echo_block[i] = (*sample) / 2;
		}
		clear_echo_block = true;
	} else if (clear_echo_block) {
		clear_echo_block = false;
		memset(echo_block, 0, sizeof(echo_block));
	}
}

/*
 * usb interrupt handler
 */

static void interrupt_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);
	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (!rx_throttled && uart_irq_rx_ready(dev)) {
			int recv_len, rb_len;
			uint8_t buffer[64];
			size_t len = MIN(ring_buf_space_get(&ringbuf),
					 sizeof(buffer));
			if (len == 0) {
				/* Throttle because ring buffer is full */
				uart_irq_rx_disable(dev);
				rx_throttled = true;
				continue;
			}
			recv_len = uart_fifo_read(dev, buffer, len);
			if (recv_len < 0) {
				LOG_ERR("Failed to read UART FIFO");
				recv_len = 0;
			};
			rb_len = ring_buf_put(&ringbuf, buffer, recv_len);
			if (rb_len < recv_len) {
				LOG_ERR("Drop %u bytes", recv_len - rb_len);
			}
			LOG_DBG("tty fifo -> ringbuf %d bytes", rb_len);
			if (rb_len) {
				uart_irq_tx_enable(dev);
			}
		}
		if (uart_irq_tx_ready(dev)) {
			uint8_t buffer[64];
			int rb_len, send_len;
			rb_len = ring_buf_get(&ringbuf, buffer, sizeof(buffer));
			if (!rb_len) {
				LOG_DBG("Ring buffer empty, disable TX IRQ");
				uart_irq_tx_disable(dev);
				continue;
			}
			if (rx_throttled) {
				uart_irq_rx_enable(dev);
				rx_throttled = false;
			}
			send_len = uart_fifo_fill(dev, buffer, rb_len);
			if (send_len < rb_len) {
				LOG_ERR("Drop %d bytes", rb_len - send_len);
			}
			LOG_DBG("ringbuf -> tty fifo %d bytes", send_len);
		}
	}
}


#define MAX_PATH 128
#define SOME_FILE_NAME "some1.txt"
#define SOME_DIR_NAME "some"
#define SOME_REQUIRED_LEN MAX(sizeof(SOME_FILE_NAME), sizeof(SOME_DIR_NAME))

static int lsdir(const char *path);
#ifdef CONFIG_FS_SAMPLE_CREATE_SOME_ENTRIES
static bool create_some_entries(const char *base_path)
{
	char path[MAX_PATH];
	struct fs_file_t file;
	int base = strlen(base_path);

	fs_file_t_init(&file);

	if (base >= (sizeof(path) - SOME_REQUIRED_LEN)) {
		LOG_ERR("Not enough concatenation buffer to create file paths");
		return false;
	}

	LOG_INF("Creating some dir entries in %s", base_path);
	strncpy(path, base_path, sizeof(path));

	path[base++] = '/';
	path[base] = 0;
	strcat(&path[base], SOME_FILE_NAME);

	if (fs_open(&file, path, FS_O_CREATE | FS_O_WRITE) != 0) {
		LOG_ERR("Failed to create file %s", path);
		return false;
	}

	memset(data_rw_buf, 'D', 1024);
	fs_write(&file, &data_rw_buf[0], 1024);

	fs_close(&file);

	path[base] = 0;
	strcat(&path[base], SOME_DIR_NAME);

	if (fs_mkdir(path) != 0) {
		LOG_ERR("Failed to create dir %s", path);
		/* If code gets here, it has at least successes to create the
		 * file so allow function to return true.
		 */
	}
	return true;
}
#endif

static const char *disk_mount_pt = DISK_MOUNT_PT;

/* List dir entry by path
 *
 * @param path Absolute path to list
 *
 * @return Negative errno code on error, number of listed entries on
 *         success.
 */
static int lsdir(const char *path)
{
	int res;
	struct fs_dir_t dirp;
	static struct fs_dirent entry;
	int count = 0;
	fs_dir_t_init(&dirp);
	/* Verify fs_opendir() */
	res = fs_opendir(&dirp, path);
	if (res) {
		printf("Error opening dir %s [%d]\n", path, res);
		return res;
	}
	printf("\n[SDMMC Test] :Listing dir %s ...\n", path);
	for (;;) {
		/* Verify fs_readdir() */
		res = fs_readdir(&dirp, &entry);
		/* entry.name[0] == 0 means end-of-dir */
		if (res || entry.name[0] == 0) {
			break;
		}
		if (entry.type == FS_DIR_ENTRY_DIR) {
			printf("[SDMMC Test] :[DIR ] %s\n", entry.name);
		} else {
			printf("[SDMMC Test] :[FILE] %s (size = %zu)\n",
				entry.name, entry.size);
		}
		count++;
	}
	/* Verify fs_closedir() */
	fs_closedir(&dirp);
	if (res == 0) {
		res = count;
	}
	return res;
}

/*
 * enable usb function call
 */

static int enable_usb_device_next(void)
{
	int err;
	sample_usbd = sample_usbd_init_device();
	if (sample_usbd == NULL) {
		LOG_ERR("Failed to initialize USB device");
		return -ENODEV;
	}
	err = usbd_enable(sample_usbd);
	if (err) {
		LOG_ERR("Failed to enable device support");
		return err;
	}
	LOG_DBG("USB device support enabled");
	return 0;
}

/* DMA configurations */
#define DMA_CTRL_ACK_TYPE_Pos          (16U)
#define DMA_CTRL_ENA                   (1U << 4)

#define HE_DMA_SEL_LPSPI_Pos           (4)
#define HE_DMA_SEL_LPSPI_Msk           (0x3U << HE_DMA_SEL_LPSPI_Pos)

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(dma2), arm_dma_pl330, okay) /* dma2 */

#if (IS_ENABLED(CONFIG_SOC_SERIES_ENSEMBLE_E1C) ||  \
        IS_ENABLED(CONFIG_SOC_SERIES_BALLETTO_B1)) /* E1C/B1 dma2 */

#if DT_NODE_HAS_PROP(DT_NODELABEL(lpspi0), dmas) /* E1C/B1 LPSPI0 dma2 */
static void configure_lpspi0_for_dma2(void)
{
        /* Enable LPSPI EVTRTR channel */
        #define LPSPI_DMA_RX_PERIPH_REQ         12
        #define LPSPI_DMA_TX_PERIPH_REQ         13
        #define LPSPI_DMA_GROUP             1

        uint32_t regdata;

        printk("\n configure lpspi0 for dma2\n");

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
#endif /* E1C/B1 LPSPI0 dma2 */
#endif /* E1C/B1 LPSPI0 dma2 */

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

	while(1){
	k_msleep(SLEEPTIME);
	ret = spi_transceive(dev, &cnfg, &tx_bufset, &rx_bufset);
	if (ret) {
		printk("ERROR: SPI=%p transceive: %d\n", dev, ret);
	}
	printk("Master wrote: %08x %08x %08x %08x %08x\n",
		master_txdata[0], master_txdata[1], master_txdata[2],
		master_txdata[3], master_txdata[4]);
	printk("Master receive: %08x %08x %08x %08x %08x\n",
                master_rxdata[0], master_rxdata[1], master_rxdata[2],
                master_rxdata[3], master_rxdata[4]);

        ret = memcmp(master_rxdata, master_txdata, length);
        if (ret) {
                printk("ERROR: SPI Master RX & Slave TX DATA NOT MATCHING: %d\n", ret);
        } else {
                printk("SUCCESS: SPI Master RX & Slave TX DATA IS MATCHING: %d\n", ret);
        }

	//printk("[SPI Master] :SUCCESS: SPI Master RX & Slave TX DATA IS MATCHING: %d\n", ret);
	}

	return ret;
}

void single_sector_test(const struct device *flash_dev)
{
	const uint16_t expected[] = {0x55, 0xaa, 0x66, 0x99};
	const size_t len = ARRAY_SIZE(expected);
	uint16_t buf[len];
	int rc;
	int i, e_count = 0;

	printf("\n[OSPI Test] :Test 1: Flash erase\n");
        k_msleep(SLEEP_TIME_MS);
	/* Full flash erase if SPI_FLASH_TEST_REGION_OFFSET = 0 and
	 * SPI_FLASH_SECTOR_SIZE = flash size
	 */
	rc = flash_erase(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, SPI_FLASH_SECTOR_SIZE);
	if (rc != 0) {
		printf("[OSPI Test] :Flash erase failed! %d\n", rc);
                k_msleep(SLEEP_TIME_MS);
	} else {
		printf("[OSPI Test] :Flash erase succeeded!\n");
                k_msleep(SLEEP_TIME_MS);
	}

	printf("\n[OSPI Test] :Test 1: Flash write\n");

	printf("[OSPI Test] :Attempting to write %zu bytes\n", len);
        k_msleep(SLEEP_TIME_MS);
	rc = flash_write(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, expected, len);
	if (rc != 0) {
		printf("[OSPI Test] :Flash write failed! %d\n", rc);
		return;
	}

	printf("\n[OSPI Test] :Test 1: Flash read\n");
        k_msleep(SLEEP_TIME_MS);

	memset(buf, 0, len);
	rc = flash_read(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, buf, len);
	if (rc != 0) {
		printf("[OSPI Test] :Flash read failed! %d\n", rc);
		return;
	}

	for (i = 0; i < len; i++) {
		if (buf[i] != expected[i]) {
			e_count++;
			printf("[OSPI Test] :Not matched at [%d] _w[%4x] _r[%4x]\n", i, expected[i], buf[i]);
		}
	}

	if (e_count) {
		printf("[OSPI Test] :Error:Data read NOT matches data written\n");
	} else {
		printf("[OSPI Test] :Data read matches data written. Good!!\n");
	}
}

static void ospi_main(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
        ARG_UNUSED(p2);
        ARG_UNUSED(p3);
	const struct device *flash_dev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));
	if (!device_is_ready(flash_dev)) {
		printk("%s: device not ready.\n", flash_dev->name);
		return -1;
	}
	printf("\n%s OSPI flash testing\n", flash_dev->name);
	printf("========================================\n");
	flash_param = flash_get_parameters(flash_dev);
	printf("****Flash Configured Parameters******\n");
	printf("* Num Of Sectors : %d\n", flash_param->num_of_sector);
	printf("* Sector Size : %d\n", flash_param->sector_size);
	printf("* Page Size : %d\n", flash_param->page_size);
	printf("* Erase value : %d\n", flash_param->erase_value);
	printf("* Write Blk Size: %d\n", flash_param->write_block_size);
	printf("* Total Size in MB: %d\n",
	       (flash_param->num_of_sector * flash_param->sector_size) / (1024 * 1024));
	while(1){
		single_sector_test(flash_dev);
		k_msleep(5000);
	}	

	return 0;
}

static void i2s_main(void *p1, void *p2, void *p3)
{
	const struct device *i2s_dev_rx = (const struct device *)p1;
	const struct device *i2s_dev_tx = (const struct device *)p2;
        ARG_UNUSED(p3);
	while (k_sem_take(&toggle_transfer, K_NO_WAIT) != 0) {
		void *mem_block;
		uint32_t block_size;
		int ret;
		ret = i2s_read(i2s_dev_rx, &mem_block, &block_size);
		if (ret < 0) {
			printk("Failed to read data: %d\n", ret);
			break;
		}
		process_block_data(mem_block, SAMPLES_PER_BLOCK);
		ret = i2s_write(i2s_dev_tx, mem_block, block_size);
		if (ret < 0) {
			printk("Failed to write data: %d\n", ret);
			break;
		}
		printk("[I2S Tx and RX] :I2S echo Completed\n");
	}
	if (!trigger_command(i2s_dev_rx, i2s_dev_tx,
			     I2S_TRIGGER_DROP)) {
		return 0;
	}
	printk("Streams stopped\n");
	return 0;
}

static void sdmmc_main(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
        ARG_UNUSED(p2);
        ARG_UNUSED(p3);
	while(1){
	/* raw disk i/o */
	do {
		static const char *disk_pdrv = DISK_DRIVE_NAME;
		uint64_t memory_size_mb;
		uint32_t block_count;
		uint32_t block_size;
		if (disk_access_init(disk_pdrv) != 0) {
			LOG_ERR("[SDMMC Test] :Storage init ERROR!");
			break;
		}
		if (disk_access_ioctl(disk_pdrv,
				DISK_IOCTL_GET_SECTOR_COUNT, &block_count)) {
			LOG_ERR("[SDMMC Test] :Unable to get sector count");
			break;
		}
		LOG_INF("[SDMMC Test] :Block count %u", block_count);
		if (disk_access_ioctl(disk_pdrv,
				DISK_IOCTL_GET_SECTOR_SIZE, &block_size)) {
			LOG_ERR("[SDMMC Test] :Unable to get sector size");
			break;
		}
		printf("[SDMMC Test] :Sector size %u\n", block_size);
		memory_size_mb = (uint64_t)block_count * block_size;
		printf("[SDMMC Test] :Memory Size(MB) %u\n", (uint32_t)(memory_size_mb >> 20));
	} while (0);
	mp.mnt_point = disk_mount_pt;
	int res = fs_mount(&mp);
#if defined(CONFIG_FAT_FILESYSTEM_ELM)
	if (res == FR_OK) {
#else
	if (res == 0) {
#endif
		printf("[SDMMC Test] :Disk mounted.\n");
		if (lsdir(disk_mount_pt) == 0) {
#ifdef CONFIG_FS_SAMPLE_CREATE_SOME_ENTRIES
			if (create_some_entries(disk_mount_pt)) {
				lsdir(disk_mount_pt);
			}
#endif
		}
	} else {
		printf("[SDMMC Test] :Error mounting disk.\n");
	}
	fs_unmount(&mp);
	k_sleep(K_MSEC(3000));
	}
	while (1) {
		//k_sleep(K_MSEC(1000));
	}
	return 0;
}
static void usb_dev_main(void *p1, void *p2, void *p3)
{
	const struct device *dev = (const struct device *)p1;
        ARG_UNUSED(p2);
        ARG_UNUSED(p3);
	uart_irq_callback_set(dev, interrupt_handler);
	/* Enable rx interrupts */
	uart_irq_rx_enable(dev);
	return 0;
}

static void blinky_main(void *p1, void *p2, void *p3)
{
	int ret;
	bool led_state = true;
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}
	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}
	printk("Inside blinky: toggle Master device\n");
	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}
		led_state = !led_state;
		printf("[Gpio Blinky] :LED state: %s\n", led_state ? "ON" : "OFF");
		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}

static void master_spi(void *p1, void *p2, void *p3)
{
	const struct device *const dev = DEVICE_DT_GET(SPIDW_NODE);
	int ret;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	if (!device_is_ready(dev)) {
		printk("%s: Master device not ready.\n", dev->name);
		return;
	}
	struct spi_cs_control cs_ctrl = (struct spi_cs_control){
		.gpio = GPIO_DT_SPEC_GET(SPIDW_NODE, cs_gpios),
		.delay = 0u,
	};
		ret = master_spi_transceive(dev, &cs_ctrl);
		if (ret < 0) {
			printk("Stopping the Master Thread due to error\n");
			return;
		}
	printk("[SPI Master] :Master Transfer Successfully Completed\n");
}

static void prepare_data(uint32_t *data, uint16_t def_mask)
{
	for (uint32_t cnt = 0; cnt < BUFF_SIZE; cnt++) {
		data[cnt] = (def_mask << 16) | cnt;
	}
}

int main(void)
{
	prepare_data(master_txdata, 0xA5A5);
	configure_lpspi0_for_dma2();
	/*Usb_setup*/
	/*const struct device *dev;
        int ret;
        dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
        if (!device_is_ready(dev)) {
                LOG_ERR("CDC ACM device not ready");
                return 0;
        }
        printk("USB device CDC-ACM app started\n");
        ret = enable_usb_device_next();
        if (ret != 0)
        {
                LOG_ERR("Failed to enable USB");
                return 0;
        }
        ring_buf_init(&ringbuf, sizeof(ring_buffer), ring_buffer);
        /* Wait 100ms for the host to do all settings */
        k_msleep(100);

	/*I2S setup*/
	/*const struct device *const i2s_dev_rx = DEVICE_DT_GET(I2S_RX_NODE);
	const struct device *const i2s_dev_tx = DEVICE_DT_GET(I2S_TX_NODE);
	const struct device *const codec_dev = DEVICE_DT_GET(DT_NODELABEL(audio_codec));

	struct i2s_config config;
	struct audio_codec_cfg audio_cfg;
	printk("I2S echo sample\n");
	if (!init_buttons()) {
		return 0;
	}
	if (!device_is_ready(i2s_dev_rx)) {
		printk("%s is not ready\n", i2s_dev_rx->name);
		return 0;
	}
	if (i2s_dev_rx != i2s_dev_tx && !device_is_ready(i2s_dev_tx)) {
		printk("%s is not ready\n", i2s_dev_tx->name);
		return 0;
	}
	if (!device_is_ready(codec_dev)) {
		printk("%s is not ready\n", codec_dev->name);
		return 0;
	}

	audio_cfg.dai_type = AUDIO_DAI_TYPE_I2S;
	audio_cfg.dai_cfg.i2s.word_size = SAMPLE_BIT_WIDTH;
	audio_cfg.dai_cfg.i2s.channels =  2;
	audio_cfg.dai_cfg.i2s.format = I2S_FMT_DATA_FORMAT_I2S;
	audio_cfg.dai_cfg.i2s.options = I2S_OPT_FRAME_CLK_MASTER;
	audio_cfg.dai_cfg.i2s.frame_clk_freq = SAMPLE_FREQUENCY;
	audio_cfg.dai_cfg.i2s.mem_slab = &mem_slab;
	audio_cfg.dai_cfg.i2s.block_size = BLOCK_SIZE;
	audio_codec_configure(codec_dev, &audio_cfg);
	k_msleep(1000);

	config.word_size = SAMPLE_BIT_WIDTH;
	config.channels = NUMBER_OF_CHANNELS;
	config.format = I2S_FMT_DATA_FORMAT_I2S;
	config.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
	config.frame_clk_freq = SAMPLE_FREQUENCY;
	config.mem_slab = &mem_slab;
	config.block_size = BLOCK_SIZE;
	config.timeout = TIMEOUT;
	if (!configure_streams(i2s_dev_rx, i2s_dev_tx, &config)) {
		return 0;
	}
		k_sem_take(&toggle_transfer, K_FOREVER);
		if (!prepare_transfer(i2s_dev_rx, i2s_dev_tx)) {
			return 0;
		}
		if (!trigger_command(i2s_dev_rx, i2s_dev_tx,
				     I2S_TRIGGER_START)) {
			return 0;
		}
		printk("Streams started\n");

	/*Thread_creation*/
#if 1
/*	k_tid_t tidu = k_thread_create(&usb_dev, usb_dev_stack, STACKSIZE,
			&usb_dev_main, (void *)dev, NULL, NULL,
			SLAVE_PRIORITY, 0, K_NO_WAIT);
	if (tidu == NULL) {
		printk("Error creating Usb Thread\n");
	}*/
	k_tid_t tidm = k_thread_create(&MasterT_data, MasterT_stack, 2048,
			&master_spi, NULL, NULL, NULL,
			MASTER_PRIORITY, 0, K_NO_WAIT);
	if (tidm == NULL) {
		printk("Error creating SPI Master Thread\n");
	}
	k_tid_t tidg = k_thread_create(&blinky, blinky_stack, STACKSIZE,
			&blinky_main, NULL, NULL, NULL,
			MASTER_PRIORITY, 0, K_NO_WAIT);
	if (tidg == NULL) {
		printk("Error creating Blinky Thread\n");
	}
	k_tid_t tids = k_thread_create(&sdmmc, sdmmc_stack, STACKSIZE*4,
			&sdmmc_main, NULL, NULL, NULL,
			MASTER_PRIORITY, 0, K_NO_WAIT);
	if (tids == NULL) {
		printk("Error creating SDMMC Thread\n");
	}
#endif
	/*k_tid_t tidi = k_thread_create(&i2s, i2s_stack, STACKSIZE*4,
			&i2s_main, (void *)i2s_dev_rx, (void *)i2s_dev_tx, NULL,
			MASTER_PRIORITY, 0, K_NO_WAIT);
	if (tidi == NULL) {
		printk("Error creating I2S Thread\n");
	}*/
	k_tid_t tido = k_thread_create(&ospi, ospi_stack, STACKSIZE,
			&ospi_main, NULL, NULL, NULL,
			8, 0, K_NO_WAIT);
	if (tido == NULL) {
		printk("Error creating OSPI Thread\n");
	}
	k_msleep(500);
	return 0;
}
