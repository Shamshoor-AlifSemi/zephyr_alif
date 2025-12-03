/*
 * Copyright (c) 2024 Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include "exerciser_app.h"

#define SPI_FLASH_TEST_REGION_OFFSET 0x0
#define SPI_FLASH_SECTOR_SIZE        4096
#define BUFF_SIZE                    1024

const struct flash_parameters *flash_param;

void single_sector_test(const struct device *flash_dev)
{
	const uint8_t expected[] = {0x55, 0xaa, 0x66, 0x99};
	const size_t len = ARRAY_SIZE(expected);
	uint8_t buf[len];
	int rc;
	int i, e_count = 0;

	printf("\n[ OSPI ]Test 1: Flash erase\n");

	/* Full flash erase if SPI_FLASH_TEST_REGION_OFFSET = 0 and
	 * SPI_FLASH_SECTOR_SIZE = flash size
	 */
	rc = flash_erase(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, SPI_FLASH_SECTOR_SIZE);
	if (rc != 0) {
		printf("[ OSPI ]Flash erase failed! %d\n", rc);
	} else {
		printf("[ OSPI ]Flash erase succeeded!\n");
	}

	printf("\n[ OSPI ]Test 1: Flash write\n");

	printf("[ OSPI ]Attempting to write %zu bytes\n", len);
	rc = flash_write(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, expected, len);
	if (rc != 0) {
		printf("[ OSPI ]Flash write failed! %d\n", rc);
		return;
	}

	printf("\n[ OSPI ]Test 1: Flash read\n");

	memset(buf, 0, len);
	rc = flash_read(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, buf, len);
	if (rc != 0) {
		printf("[ OSPI ]Flash read failed! %d\n", rc);
		return;
	}

	for (i = 0; i < len; i++) {
		if (buf[i] != expected[i]) {
			e_count++;
			printf("[ OSPI ]Not matched at [%d] _w[%4x] _r[%4x]\n", i, expected[i], buf[i]);
		}
	}

	if (e_count) {
		printf("[ OSPI ]Error:Data read NOT matches data written\n");
	} else {
		printf("[ OSPI ]Data read matches data written. Good!!\n");
	}
}

void erase_test(const struct device *dev, uint32_t len)
{
	int ret = 0, i = 0, count = 0;
	uint8_t r_buf[BUFF_SIZE] = {0};

	printf("\n[ OSPI ]Test 2: Flash Full Erase\n");

	ret = flash_erase(dev, SPI_FLASH_TEST_REGION_OFFSET, len);
	if (ret == 0) {
		printf("[ OSPI ]Successfully Erased whole Flash Memory\n");
	} else {
		printf("[ OSPI ]Error: Bulk Erase Failed [%d]\n", ret);
	}

	/* Read data Cleared Buffer */
	ret = flash_read(dev, SPI_FLASH_TEST_REGION_OFFSET, r_buf, BUFF_SIZE);
	if (ret != 0) {
		printf("[ OSPI ]Error: [RetVal :%d] Reading Erased value\n", ret);
		return;
	}

	/* Verify the read data */
	for (i = 0; i < BUFF_SIZE; i++) {
		if (r_buf[i] != flash_param->erase_value) {
			count++;
		}
	}

	printf("[ OSPI ]Total errors after reading erased chip = %d\n", count);
}

void multi_page_test(const struct device *flash_dev)
{
	int rc, i, e_count = 0;

	uint8_t w_buf[BUFF_SIZE] = {0};
	uint8_t r_buf[BUFF_SIZE] = {0};

	const size_t len = ARRAY_SIZE(w_buf);

	printf("\n[ OSPI ]Test 3: Flash erase\n");

	/* Full flash erase if SPI_FLASH_TEST_REGION_OFFSET = 0 and
	 * SPI_FLASH_SECTOR_SIZE = flash size
	 */
	rc = flash_erase(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, SPI_FLASH_SECTOR_SIZE);
	if (rc != 0) {
		printf("[ OSPI ]Flash erase failed! %d\n", rc);
	} else {
		printf("[ OSPI ]Flash erase succeeded!\n");
	}

	for (i = 0; i < BUFF_SIZE; i++) {
		w_buf[i] = i % 256;
	}

	printf("\n[ OSPI ]Test 3: Flash write\n");

	printf("[ OSPI ]Attempting to write %zu bytes\n", len);
	rc = flash_write(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, w_buf, len);
	if (rc != 0) {
		printf("[ OSPI ]Flash write failed! %d\n", rc);
		return;
	}

	printf("\n[ OSPI ]Test 3: Flash read\n");

	memset(r_buf, 0, len);
	rc = flash_read(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, r_buf, len);
	if (rc != 0) {
		printf("[ OSPI ]Flash read failed! %d\n", rc);
		return;
	}

	for (i = 0; i < BUFF_SIZE; i++) {
		if (r_buf[i] != w_buf[i]) {
			e_count++;
			printf("[ OSPI ]Not matched at [%d] _w[%4x] _r[%4x]\n", i, w_buf[i], r_buf[i]);
		}
	}

	if (e_count) {
		printf("[ OSPI ]Error:Data read NOT matches data written\n");
		printf("[ OSPI ] -- number of Unmatched data [%d]\n", e_count);
	} else {
		printf("[ OSPI ]Data read matches data written. Good!!\n");
	}
}

#define SPI_FLASH_SECTOR_4_OFFSET (4 * 1024 * 4)
#define SPI_FLASH_SECTOR_5_OFFSET (5 * 1024 * 4)

void multi_sector_test(const struct device *flash_dev)
{
	int rc, i, e_count = 0;

	uint8_t w_buf[BUFF_SIZE] = {0};
	uint8_t r_buf[BUFF_SIZE] = {0};

	const size_t len = ARRAY_SIZE(w_buf);

	for (i = 0; i < BUFF_SIZE; i++) {
		w_buf[i] =  i % 256;
	}

	printf("\n[ OSPI ]Test 4: write sector %d\n", SPI_FLASH_SECTOR_4_OFFSET);
	/* Write into Sector 4 */
	rc = flash_write(flash_dev, SPI_FLASH_SECTOR_4_OFFSET, w_buf, len);
	if (rc != 0) {
		printf("\n[ OSPI ]Flash write failed at Sec 4! %d\n", rc);
		return;
	}

	printf("\n[ OSPI ]Test 4: write sector %d\n", SPI_FLASH_SECTOR_5_OFFSET);
	/* Write into Sector 5 */
	rc = flash_write(flash_dev, SPI_FLASH_SECTOR_5_OFFSET, w_buf, len);
	if (rc != 0) {
		printf("\n[ OSPI ]Flash write failed at Sec 5! %d\n", rc);
		return;
	}

	/* Read from Sector 4 */
	printf("[ OSPI ]Sec4: Read and Verify written data\n");

	e_count = 0;
	memset(r_buf, 0, len);

	printf("\n[ OSPI ]Test 4: read sector %d\n", SPI_FLASH_SECTOR_4_OFFSET);

	rc = flash_read(flash_dev, SPI_FLASH_SECTOR_4_OFFSET, r_buf, len);
	if (rc != 0) {
		printf("[ OSPI ]Flash read failed at Sector 4! %d\n", rc);
		return;
	}

	for (i = 0; i < BUFF_SIZE; i++) {
		if (r_buf[i] != w_buf[i]) {
			e_count++;
			printf("[ OSPI ]Not matched at [%d] _w[%4x] _r[%4x]\n", i, w_buf[i], r_buf[i]);
		}
	}

	if (e_count) {
		printf("\n[ OSPI ]Error:Data read NOT matches data written\n");
		printf("[ OSPI ] -- number of Unmatched data [%d]\n", e_count);
	} else {
		printf("\n[ OSPI ]Data read matches data written. Good!!\n");
	}

	/* Read from Sector 5 */
	printf("[ OSPI ]Sec5: Read and Verify written data\n");

	e_count = 0;
	memset(r_buf, 0, len);

	printf("\n[ OSPI ]Test 4: read sector %d\n", SPI_FLASH_SECTOR_5_OFFSET);

	rc = flash_read(flash_dev, SPI_FLASH_SECTOR_5_OFFSET, r_buf, len);
	if (rc != 0) {
		printf("[ OSPI ]Flash read failed at Sector 5! %d\n", rc);
		return;
	}

	for (i = 0; i < BUFF_SIZE; i++) {
		if (r_buf[i] != w_buf[i]) {
			e_count++;
			printf("[ OSPI ]Not matched at [%d] _w[%4x] _r[%4x]\n", i, w_buf[i], r_buf[i]);
		}
	}

	if (e_count) {
		printf("[ OSPI ]Error:Data read NOT matches data written\n");
		printf("[ OSPI ] -- number of Unmatched data [%d]\n", e_count);
	} else {
		printf("[ OSPI ]Data read matches data written. Good!!\n");
	}

	/* Erase multiple Sector Sec 4+5 */
	printf("\n[ OSPI ]Test 4: Erase Sector 4 and 5\n");
	printf("[ OSPI ]Flash Erase from Sector %d Size to Erase %d\n", SPI_FLASH_SECTOR_4_OFFSET,
			SPI_FLASH_SECTOR_SIZE * 2);

	rc = flash_erase(flash_dev, SPI_FLASH_SECTOR_4_OFFSET, SPI_FLASH_SECTOR_SIZE * 2);
	if (rc != 0) {
		printf("\n[ OSPI ]Multi-Sector erase failed! %d\n", rc);
	} else {
		printf("\n[ OSPI ]Multi-Sector erase succeeded!\n");
	}

	int count_1 = 0;

	memset(r_buf, 0, len);

	printf("\n[ OSPI ]Test 4: read sector %d\n", SPI_FLASH_SECTOR_4_OFFSET);
	/* Read Erased value and compare */
	rc = flash_read(flash_dev, SPI_FLASH_SECTOR_4_OFFSET, r_buf, BUFF_SIZE);
	if (rc != 0) {
		printf("[ OSPI ]Error: [RetVal :%d] Reading Erased value\n", rc);
		return;
	}

	/* Verify the read data */
	for (i = 0; i < BUFF_SIZE; i++) {
		if (r_buf[i] != flash_param->erase_value) {
			count_1++;
		}
	}

	printf("[ OSPI ]Total errors after reading erased Sector 4 = %d\n", count_1);

	int count_2 = 0;

	memset(r_buf, 0, len);
	printf("\n[ OSPI ]Test 4: read sector %d\n", SPI_FLASH_SECTOR_5_OFFSET);

	/* Read Erased value and compare */
	rc = flash_read(flash_dev, SPI_FLASH_SECTOR_5_OFFSET, r_buf, BUFF_SIZE);
	if (rc != 0) {
		printf("[ OSPI ]Error: [RetVal :%d] Reading Erased value\n", rc);
		return;
	}

	/* Verify the read data */
	for (i = 0; i < BUFF_SIZE; i++) {
		if (r_buf[i] != flash_param->erase_value) {
			count_2++;
		}
	}

	printf("[ OSPI ]Total errors after reading erased Sector 5 = %d\n", count_2);

	if (count_1 == 0 && count_2 == 0) {
		printf("\n[ OSPI ]Multi-Sector Erase Test Succeeded !\n");
	} else {
		printf("\n[ OSPI ]Multi-Sector Erase Failed\n");
	}
}

void xip_test(const struct device *flash_dev)
{
	uint8_t i;
	uint32_t xip_r[64] = {0}, fls_r[64] = {0}, cnt;
	uint32_t *ptr = (uint32_t *)DT_PROP_BY_IDX(DT_PARENT(DT_ALIAS(spi_flash0)),
			xip_base_address, 0);
	int32_t rc, e_count = 0;

	printf("\n[ OSPI ]Test 5: XiP Read\n");

	memcpy(xip_r, ptr, sizeof(xip_r));

	printf("[ OSPI ]Content Read from OSPI Flash in XiP Mode successfully\n\n");

	cnt = ARRAY_SIZE(xip_r);

	printf("[ OSPI ]Read from Flash cmd while XiP Mode turnned on\n\n");

	rc = flash_read(flash_dev, SPI_FLASH_TEST_REGION_OFFSET,
			fls_r, cnt * sizeof(uint32_t));
	if (rc != 0) {
		printf("[ OSPI ]Flash read failed! %d\n", rc);
		return;
	}

	for (i = 0; i < cnt; i++)
		if (fls_r[i] != xip_r[i]) {
			e_count++;
		}

	if (!e_count) {
		printf("[ OSPI ]XiP Read Test Succceeded !!\n\n");
	} else {
		printf("[ OSPI ]XiP Test Failed !"
				" contents are NOT Matching : Err Count [%d]!!!\n", e_count);
	}
}

void ospi_thread(void)
{
	const struct device *flash_dev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));

	if (!device_is_ready(flash_dev)) {
		printk("[ OSPI ]%s: device not ready.\n", flash_dev->name);
		return;
	}
	while(1){
		printf("\n[ OSPI ]%s OSPI flash testing\n", flash_dev->name);
		printf("========================================\n");
		flash_param = flash_get_parameters(flash_dev);

		printf("[ OSPI ]****Flash Configured Parameters******\n");
		printf("[ OSPI ]* Num Of Sectors : %d\n", flash_param->num_of_sector);
		printf("[ OSPI ]* Sector Size : %d\n", flash_param->sector_size);
		printf("[ OSPI ]* Page Size : %d\n", flash_param->page_size);
		printf("[ OSPI ]* Erase value : %d\n", flash_param->erase_value);
		printf("[ OSPI ]* Write Blk Size: %d\n", flash_param->write_block_size);
		printf("[ OSPI ]* Total Size in MB: %d\n",
				(flash_param->num_of_sector * flash_param->sector_size) / (1024 * 1024));

		/* ---- RUN ALL OSPI TESTS HERE ---- */
		single_sector_test(flash_dev);
		erase_test(flash_dev, flash_param->num_of_sector * flash_param->sector_size);
		multi_page_test(flash_dev);
		multi_sector_test(flash_dev);

#ifdef CONFIG_ALIF_OSPI_FLASH_XIP
		xip_test(flash_dev);
#endif
		printf("\n[ OSPI ]OSPI flash test thread completed.\n");
	};
} 
