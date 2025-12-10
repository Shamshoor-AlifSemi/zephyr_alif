/*
 * Copyright (C) 2024 Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/audio/codec.h>
#include <zephyr/drivers/pdm/pdm_alif.h>
#include <string.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pdm_i2s, LOG_LEVEL_INF);

/* Device tree aliases */
#define PDM_NODE	DT_ALIAS(pdm_audio)
#define I2S_NODE	DT_ALIAS(i2s_codec_tx)

/* Audio configuration */
#define SAMPLE_FREQUENCY	CONFIG_SAMPLE_FREQ
#define SAMPLE_BIT_WIDTH	16
#define PDM_CHANNELS		1	/* Mono capture */
#define I2S_CHANNELS		2	/* Stereo playback */

/* Buffer configuration - 100ms latency */
#define SAMPLES_PER_BLOCK	((SAMPLE_FREQUENCY / 10) * PDM_CHANNELS)
#define BLOCK_SIZE		(SAMPLES_PER_BLOCK * sizeof(int16_t))
#define NUM_BLOCKS		16  /* Increased for real-time continuous operation */
#define TIMEOUT_MS		5000

/* PDM Channel 4 configuration */
#define CHANNEL_4		4
#define CH4_PHASE		0x0000001F
#define CH4_GAIN		0x0000000D
#define CH4_PEAK_DETECT_TH	0x00060002
#define CH4_PEAK_DETECT_ITV	0x0004002D

/* Channel 4 FIR coefficients */
static uint32_t ch4_fir[18] = {
	0x00000001, 0x00000003, 0x00000003, 0x000007F4,
	0x00000004, 0x000007ED, 0x000007F5, 0x000007F4,
	0x000007D3, 0x000007FE, 0x000007BC, 0x000007E5,
	0x000007D9, 0x00000793, 0x00000029, 0x0000072C,
	0x00000072, 0x000002FD
};

/* Memory slab for PDM audio buffers */
K_MEM_SLAB_DEFINE(audio_mem_slab, BLOCK_SIZE, NUM_BLOCKS, 4);

/* Memory slab for I2S buffers (stereo, so double size) */
K_MEM_SLAB_DEFINE(i2s_mem_slab, BLOCK_SIZE * 2, NUM_BLOCKS, 4);

/* I2S stereo buffer (double size for stereo) */
static int16_t i2s_buffer[SAMPLES_PER_BLOCK * 2];

/**
 * Configure PDM channel 4
 */
static void pdm_ch_config(const struct device *pdm_dev)
{
	struct pdm_ch_config pdm_coef_reg;

	pdm_set_ch_phase(pdm_dev, CHANNEL_4, CH4_PHASE);
	pdm_set_ch_gain(pdm_dev, CHANNEL_4, CH4_GAIN);
	pdm_set_peak_detect_th(pdm_dev, CHANNEL_4, CH4_PEAK_DETECT_TH);
	pdm_set_peak_detect_itv(pdm_dev, CHANNEL_4, CH4_PEAK_DETECT_ITV);

	pdm_coef_reg.ch_num = 4;
	memcpy(pdm_coef_reg.ch_fir_coef, ch4_fir, sizeof(pdm_coef_reg.ch_fir_coef));
	pdm_coef_reg.ch_iir_coef = 0x00000004;

	pdm_channel_config(pdm_dev, &pdm_coef_reg);
	
	/* Use high quality mode for 16kHz (1.024MHz clock) */
	pdm_mode(pdm_dev, PDM_MODE_HIGH_QUALITY_1024_CLK_FRQ);
}

/**
 * Convert mono PCM to stereo by duplicating samples
 */
static void mono_to_stereo(const int16_t *mono, int16_t *stereo, size_t samples)
{
	for (size_t i = 0; i < samples; i++) {
		stereo[2 * i] = mono[i];	/* Left channel */
		stereo[2 * i + 1] = mono[i];	/* Right channel */
	}
}

int main(void)
{
	const struct device *pdm_dev = DEVICE_DT_GET(PDM_NODE);
	const struct device *i2s_dev = DEVICE_DT_GET(I2S_NODE);
	const struct device *codec_dev = DEVICE_DT_GET(DT_NODELABEL(audio_codec));
	
	struct pcm_stream_cfg pdm_stream;
	struct dmic_cfg pdm_cfg;
	struct i2s_config i2s_cfg;
	struct audio_codec_cfg codec_cfg;
	
	void *pdm_buffer;
	uint32_t pdm_size;
	int ret;

	printk("PDM to I2S Audio Loopback Sample\n");

	/* Check PDM device */
	if (!device_is_ready(pdm_dev)) {
		LOG_ERR("PDM device not ready");
		return -ENODEV;
	}
	LOG_INF("PDM device ready");

	/* Check I2S device */
	if (!device_is_ready(i2s_dev)) {
		LOG_ERR("I2S device not ready");
		return -ENODEV;
	}
	LOG_INF("I2S device ready");

	/* Check codec device */
	if (!device_is_ready(codec_dev)) {
		LOG_ERR("Codec device not ready");
		return -ENODEV;
	}
	LOG_INF("Codec device ready");

	/* Configure PDM */
	LOG_INF("Initializing PDM...");
	pdm_stream.pcm_width = SAMPLE_BIT_WIDTH;
	pdm_stream.mem_slab = &audio_mem_slab;

	pdm_cfg.io.min_pdm_clk_freq = 1000000;
	pdm_cfg.io.max_pdm_clk_freq = 3500000;
	pdm_cfg.io.min_pdm_clk_dc = 40;
	pdm_cfg.io.max_pdm_clk_dc = 60;
	pdm_cfg.streams = &pdm_stream;
	pdm_cfg.streams[0].mem_slab = &audio_mem_slab;
	pdm_cfg.streams[0].block_size = BLOCK_SIZE;
	pdm_cfg.streams[0].pcm_rate = SAMPLE_FREQUENCY;  /* Set 48kHz output rate */
	pdm_cfg.channel.req_num_streams = 1;
	pdm_cfg.channel.req_num_chan = PDM_CHANNELS;
	pdm_cfg.channel.req_chan_map_lo = PDM_MASK_CHANNEL_4;

	ret = dmic_configure(pdm_dev, &pdm_cfg);
	if (ret < 0) {
		LOG_ERR("Failed to configure PDM: %d", ret);
		return ret;
	}

	/* Apply PDM channel configuration */
	pdm_ch_config(pdm_dev);

	/* Configure Audio Codec - use same slab as I2S */
	LOG_INF("Initializing Codec...");
	codec_cfg.dai_type = AUDIO_DAI_TYPE_I2S;
	codec_cfg.dai_cfg.i2s.word_size = 24;  /* Codec uses 24-bit internally */
	codec_cfg.dai_cfg.i2s.channels = I2S_CHANNELS;
	codec_cfg.dai_cfg.i2s.format = I2S_FMT_DATA_FORMAT_I2S;
	codec_cfg.dai_cfg.i2s.options = I2S_OPT_FRAME_CLK_MASTER;
	codec_cfg.dai_cfg.i2s.frame_clk_freq = SAMPLE_FREQUENCY;
	codec_cfg.dai_cfg.i2s.mem_slab = &i2s_mem_slab;
	codec_cfg.dai_cfg.i2s.block_size = BLOCK_SIZE * 2; /* Stereo */

	ret = audio_codec_configure(codec_dev, &codec_cfg);
	if (ret < 0) {
		LOG_ERR("Failed to configure codec: %d", ret);
		return ret;
	}

	/* Let codec stabilize - increased to 1 second like working sample */
	k_msleep(1000);

	/* Configure I2S */
	LOG_INF("Initializing I2S...");
	i2s_cfg.word_size = SAMPLE_BIT_WIDTH;
	i2s_cfg.channels = I2S_CHANNELS;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	i2s_cfg.options = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER;
	i2s_cfg.frame_clk_freq = SAMPLE_FREQUENCY;
	i2s_cfg.mem_slab = &i2s_mem_slab;
	i2s_cfg.block_size = BLOCK_SIZE * 2; /* Stereo */
	i2s_cfg.timeout = TIMEOUT_MS;

	ret = i2s_configure(i2s_dev, I2S_DIR_TX, &i2s_cfg);
	if (ret < 0) {
		LOG_ERR("Failed to configure I2S: %d", ret);
		return ret;
	}

	/* Start PDM capture */
	ret = dmic_trigger(pdm_dev, DMIC_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("Failed to start PDM: %d", ret);
		return ret;
	}

	printk("Audio loopback started\n");

#ifdef CONFIG_DEBUG_MODE
	/* ... (Debug mode omitted) ... */
#else
	/* ===== REAL-TIME MODE: Continuous loopback ===== */
	printk("REAL-TIME MODE: Speak into the microphone!\n");

	bool i2s_started = false;
	int block_count = 0;
	void *i2s_tx_buffer;
	
	while (1) {
		/* Read audio from PDM */
		ret = dmic_read(pdm_dev, 0, &pdm_buffer, &pdm_size, TIMEOUT_MS);
		if (ret < 0) {
			LOG_ERR("PDM read failed: %d", ret);
			/* If PDM fails, try to restart it? */
			dmic_trigger(pdm_dev, DMIC_TRIGGER_STOP);
			dmic_trigger(pdm_dev, DMIC_TRIGGER_START);
			continue;
		}

		block_count++;
		if (block_count % 100 == 0) {
			LOG_INF("Still running... processed %d blocks", block_count);
		}

		/* Allocate I2S buffer from slab */
		ret = k_mem_slab_alloc(&i2s_mem_slab, &i2s_tx_buffer, K_NO_WAIT);
		if (ret < 0) {
			/* If I2S slab is full, it means I2S is stalled or slow */
			LOG_WRN("I2S slab full! Dropping frame.");
			k_mem_slab_free(&audio_mem_slab, pdm_buffer);
			continue;
		}

		/* Convert mono to stereo directly into I2S buffer */
		mono_to_stereo((int16_t *)pdm_buffer, (int16_t *)i2s_tx_buffer, 
			       pdm_size / sizeof(int16_t));

		/* Free PDM buffer immediately */
		k_mem_slab_free(&audio_mem_slab, pdm_buffer);

		/* Write to I2S */
		ret = i2s_write(i2s_dev, i2s_tx_buffer, pdm_size * 2);
		if (ret < 0) {
			LOG_ERR("I2S write failed: %d", ret);
			k_mem_slab_free(&i2s_mem_slab, i2s_tx_buffer);
			
			/* Try to recover I2S */
			i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
			i2s_started = false;
			continue;
		}

		if (!i2s_started) {
			ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
			if (ret < 0) {
				LOG_ERR("Failed to start I2S: %d", ret);
			} else {
				i2s_started = true;
				LOG_INF("I2S playback started - continuous mode");
			}
		}
	}
#endif

	return 0;
}

