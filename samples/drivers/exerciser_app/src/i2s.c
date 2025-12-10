/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/audio/codec.h>
#include <string.h>
#include "exerciser_app.h"
#include "hello_sample.h"

#define I2S_CODEC_TX  DT_ALIAS(i2s_codec_tx)

#define SAMPLE_FREQUENCY    CONFIG_SAMPLE_FREQ
#define SAMPLE_BIT_WIDTH    (24U)
#define BYTES_PER_SAMPLE    sizeof(int16_t)
#if CONFIG_USE_DMIC
#define NUMBER_OF_CHANNELS  CONFIG_DMIC_CHANNELS
#else
#define NUMBER_OF_CHANNELS (2U)
#endif
/* Such block length provides an echo with the delay of 100 ms. */
// #define SAMPLES_PER_BLOCK   ((SAMPLE_FREQUENCY / 10) * NUMBER_OF_CHANNELS)
// #define INITIAL_BLOCKS      CONFIG_I2S_INIT_BUFFERS
// #define TIMEOUT             (2000U)

//#define BLOCK_SIZE  (BYTES_PER_SAMPLE * SAMPLES_PER_BLOCK)
//#define BLOCK_COUNT (INITIAL_BLOCKS + 32)
//K_MEM_SLAB_DEFINE_STATIC(mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);

#define SAMPLE_NO 64

/* The data represent a sine wave */
static int16_t data[SAMPLE_NO] = {
	3211,   6392,   9511,  12539,  15446,  18204,  20787,  23169,
	25329,  27244,  28897,  30272,  31356,  32137,  32609,  32767,
	32609,  32137,  31356,  30272,  28897,  27244,  25329,  23169,
	20787,  18204,  15446,  12539,   9511,   6392,   3211,      0,
	-3212,  -6393,  -9512, -12540, -15447, -18205, -20788, -23170,
	-25330, -27245, -28898, -30273, -31357, -32138, -32610, -32767,
	-32610, -32138, -31357, -30273, -28898, -27245, -25330, -23170,
	-20788, -18205, -15447, -12540,  -9512,  -6393,  -3212,     -1,
};

/* Fill buffer with sine wave on left channel, and sine wave shifted by
 * 90 degrees on right channel. "att" represents a power of two to attenuate
 * the samples by
 */
static void fill_buf(int16_t *tx_block, int att)
{
	int r_idx;

	for (int i = 0; i < SAMPLE_NO; i++) {
		/* Left channel is sine wave */
		tx_block[2 * i] = data[i] / (1 << att);
		/* Right channel is same sine wave, shifted by 90 degrees */
		r_idx = (i + (ARRAY_SIZE(data) / 4)) % ARRAY_SIZE(data);
		tx_block[2 * i + 1] = data[r_idx] / (1 << att);
	}
}

/*static void fill_hellosample(int32_t *tx_block, int32_t len) {

  for (int i = 0; i < len ; i ++)
  tx_block[i] = ulHelloSamples24bit48khz[i];
  }*/

#define NUM_BLOCKS 20
#define BLOCK_SIZE (2 * sizeof(data))

#ifdef CONFIG_NOCACHE_MEMORY
#define MEM_SLAB_CACHE_ATTR __nocache
#else
#define MEM_SLAB_CACHE_ATTR
#endif /* CONFIG_NOCACHE_MEMORY */

static char MEM_SLAB_CACHE_ATTR __aligned(WB_UP(32))
	_k_mem_slab_buf_tx_0_mem_slab[(NUM_BLOCKS) * WB_UP(BLOCK_SIZE)];

	static STRUCT_SECTION_ITERABLE(k_mem_slab, tx_0_mem_slab) =
	Z_MEM_SLAB_INITIALIZER(tx_0_mem_slab, _k_mem_slab_buf_tx_0_mem_slab,
			WB_UP(BLOCK_SIZE), NUM_BLOCKS);

#define SLAB_TOTAL_SIZE sizeof(ulHelloSamples24bit48khz)
#define SLAB_BLOCK_SIZE (SLAB_TOTAL_SIZE / 1024)
#define SLAB_NUM_BLOCK  (SLAB_TOTAL_SIZE / SLAB_BLOCK_SIZE)

K_MEM_SLAB_DEFINE(tx_slab, SLAB_BLOCK_SIZE, SLAB_NUM_BLOCK, 4);

void i2s_thread(void)
{
	uint32_t tx_idx, flag;
	void *tx_block[NUM_BLOCKS];

	struct i2s_config i2s_cfg;

	const struct device *const i2s_dev_codec = DEVICE_DT_GET(I2S_CODEC_TX);
	const struct device *const codec_dev = DEVICE_DT_GET(DT_NODELABEL(audio_codec));
	struct i2s_config config;
	struct audio_codec_cfg audio_cfg;
	int ret;
	printf("[ I2S ]codec sample\n");

	if (!device_is_ready(i2s_dev_codec)) {
		printf("[ I2S ]%s is not ready\n", i2s_dev_codec->name);
		return;
	}

	if (!device_is_ready(codec_dev)) {
		printf("[ I2S ]%s is not ready", codec_dev->name);
		return;
	}
	//audio_cfg.dai_route = AUDIO_ROUTE_PLAYBACK;
	audio_cfg.dai_type = AUDIO_DAI_TYPE_I2S;
	audio_cfg.dai_cfg.i2s.word_size = SAMPLE_BIT_WIDTH;
	audio_cfg.dai_cfg.i2s.channels =  2;
	audio_cfg.dai_cfg.i2s.format = I2S_FMT_DATA_FORMAT_I2S;
	audio_cfg.dai_cfg.i2s.options = I2S_OPT_FRAME_CLK_MASTER;
	audio_cfg.dai_cfg.i2s.frame_clk_freq = SAMPLE_FREQUENCY;
	audio_cfg.dai_cfg.i2s.mem_slab = &tx_0_mem_slab;
	audio_cfg.dai_cfg.i2s.block_size = BLOCK_SIZE;
	audio_codec_configure(codec_dev, &audio_cfg);
	k_msleep(1000);


	/* Configure I2S stream */
	i2s_cfg.word_size = 16U;
	i2s_cfg.channels = 2U;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	i2s_cfg.frame_clk_freq = SAMPLE_FREQUENCY;
	i2s_cfg.block_size = BLOCK_SIZE;
	i2s_cfg.timeout = 2000;
	/* Configure the Transmit port as Master */
	i2s_cfg.options = I2S_OPT_FRAME_CLK_MASTER
		| I2S_OPT_BIT_CLK_MASTER;
	i2s_cfg.mem_slab = &tx_0_mem_slab;
	while(1){
		ret = i2s_configure(i2s_dev_codec, I2S_DIR_TX, &i2s_cfg);
		if (ret < 0) {
			printf("[ I2S ]Failed to configure I2S stream\n");
		}
		/* Prepare all TX blocks */
		for (tx_idx = 0; tx_idx < NUM_BLOCKS; tx_idx++) {
			ret = k_mem_slab_alloc(&tx_0_mem_slab, &tx_block[tx_idx],
					K_FOREVER);
			if (ret < 0) {
				printf("[ I2S ]Failed to allocate TX block\n");
			}
			fill_buf((uint16_t *)tx_block[tx_idx], tx_idx % 3);
		}
		for(;;) {
			flag = 0;
			tx_idx = 0;
			/* Send first block */
			ret = i2s_write(i2s_dev_codec, tx_block[tx_idx++], BLOCK_SIZE);
			if (ret < 0) {
				printf("[ I2S ]Could not write TX buffer %d, error %d\n", tx_idx,ret);
				flag++;//		return;
			}
//			k_msleep(1000);
			/* Trigger the I2S transmission */
			ret = i2s_trigger(i2s_dev_codec, I2S_DIR_TX, I2S_TRIGGER_START);
			if (ret < 0) {
				printf("[ I2S ]Could not trigger I2S tx, error %d\n",ret);
				flag++;//		return;
			}
			for (; tx_idx < NUM_BLOCKS; ) {
				ret = i2s_write(i2s_dev_codec, tx_block[tx_idx++], BLOCK_SIZE);
				if (ret < 0) {
					printf("[ I2S ]Could not write TX buffer %d, error %d\n", tx_idx,ret);
					flag++;//			return;
				}
			}
			/* Drain TX queue */
			ret = i2s_trigger(i2s_dev_codec, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
			if (ret < 0) {
				printf("[ I2S ]Could not trigger I2S Trigger Drain tx, error %d\n",ret);
				i2s_trigger(i2s_dev_codec, I2S_DIR_TX, I2S_TRIGGER_PREPARE);
				flag++;//		return ret;
			}
			printf("[ I2S ]All I2S blocks written\n");
			if(flag != 0){
				break;
			}
		}
	}
	return;

}
