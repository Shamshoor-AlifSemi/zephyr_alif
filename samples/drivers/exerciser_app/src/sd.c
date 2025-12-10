/*
 * Copyright (c) 2019 Tavish Naruka <tavishnaruka@gmail.com>
 * Copyright (c) 2023 Nordic Semiconductor ASA
 * Copyright (c) 2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Sample which uses the filesystem API and SDHC driver */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include "exerciser_app.h"

#if defined(CONFIG_FAT_FILESYSTEM_ELM)

#include <ff.h>

/*
 *  Note the fatfs library is able to mount only strings inside _VOLUME_STRS
 *  in ffconf.h
 */
#if defined(CONFIG_DISK_DRIVER_MMC)
#define DISK_DRIVE_NAME "SD2"
#else
#define DISK_DRIVE_NAME "SD"
#endif

#define DISK_MOUNT_PT "/"DISK_DRIVE_NAME":"

static FATFS Z_GENERIC_SECTION(CONFIG_SD_BUFFER_SECTION) fat_fs;
/* mounting info */
static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
};

#elif defined(CONFIG_FILE_SYSTEM_EXT2)

#include <zephyr/fs/ext2.h>

#define DISK_DRIVE_NAME "SD"
#define DISK_MOUNT_PT "/ext"

static struct fs_mount_t mp = {
	.type = FS_EXT2,
	.flags = FS_MOUNT_FLAG_NO_FORMAT,
	.storage_dev = (void *)DISK_DRIVE_NAME,
	.mnt_point = "/ext",
};

#endif

#if defined(CONFIG_FAT_FILESYSTEM_ELM)
#define FS_RET_OK FR_OK
#else
#define FS_RET_OK 0
#endif

LOG_MODULE_REGISTER(main);

#define MAX_PATH 128
#define SOME_FILE_NAME "some.dat"
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
		LOG_ERR("[ SDMMC ]Not enough concatenation buffer to create file paths");
		return false;
	}

	LOG_INF("[ SDMMC ]Creating some dir entries in %s", base_path);
	strncpy(path, base_path, sizeof(path));

	path[base++] = '/';
	path[base] = 0;
	strcat(&path[base], SOME_FILE_NAME);

	if (fs_open(&file, path, FS_O_CREATE) != 0) {
		LOG_ERR("[ SDMMC ]Failed to create file %s", path);
		return false;
	}
	fs_close(&file);

	path[base] = 0;
	strcat(&path[base], SOME_DIR_NAME);

	if (fs_mkdir(path) != 0) {
		LOG_ERR("[ SDMMC ]Failed to create dir %s", path);
		/* If code gets here, it has at least successes to create the
		 * file so allow function to return true.
		 */
	}
	return true;
}
#endif

static const char *disk_mount_pt = DISK_MOUNT_PT;


void sd_thread(void)
{
	/* raw disk i/o */
		do {
			static const char *disk_pdrv = DISK_DRIVE_NAME;
			uint64_t memory_size_mb;
			uint32_t block_count;
			uint32_t block_size;

			if (disk_access_ioctl(disk_pdrv,
					DISK_IOCTL_CTRL_INIT, NULL) != 0) {
				LOG_ERR("[ SDMMC ]Storage init ERROR!");
				break;
			}

			if (disk_access_ioctl(disk_pdrv,
					DISK_IOCTL_GET_SECTOR_COUNT, &block_count)) {
				LOG_ERR("[ SDMMC ]Unable to get sector count");
				break;
			}
			LOG_INF("[ SDMMC ]Block count %u", block_count);

			if (disk_access_ioctl(disk_pdrv,
					DISK_IOCTL_GET_SECTOR_SIZE, &block_size)) {
				LOG_ERR("[ SDMMC ]Unable to get sector size");
				break;
			}
			printf("[ SDMMC ]Sector size %u\n", block_size);

			memory_size_mb = (uint64_t)block_count * block_size;
			printf("[ SDMMC ]Memory Size(MB) %u\n", (uint32_t)(memory_size_mb >> 20));

			if (disk_access_ioctl(disk_pdrv,
					DISK_IOCTL_CTRL_DEINIT, NULL) != 0) {
				LOG_ERR("[ SDMMC ]Storage deinit ERROR!");
				break;
			}
		} while (0);
	while(1){
		mp.mnt_point = disk_mount_pt;

		int res = fs_mount(&mp);

		if (res == FS_RET_OK) {
			printf("[ SDMMC ]Disk mounted.\n");
			/* Try to unmount and remount the disk */
			res = fs_unmount(&mp);
			if (res != FS_RET_OK) {
				printf("[ SDMMC ]Error unmounting disk\n");
//				return;
			}
			res = fs_mount(&mp);
			if (res != FS_RET_OK) {
				printf("[ SDMMC ]Error remounting disk\n");
//				return;
			}
			while(1){
			if (lsdir(disk_mount_pt) == 0) {
	#ifdef CONFIG_FS_SAMPLE_CREATE_SOME_ENTRIES
				if (create_some_entries(disk_mount_pt)) {
					lsdir(disk_mount_pt);
				}
	#endif
			}
			k_sleep(K_MSEC(2000));
			}
//			sd_operations(disk_mount_pt);
		} else {
			printf("[ SDMMC ]Error mounting disk.\n");
		}

		fs_unmount(&mp);
	}
		while (1) {
			k_sleep(K_MSEC(1000));
		}
}

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
		printf("[ SDMMC ]Error opening dir %s [%d]\n", path, res);
		return res;
	}

	printf("\n[ SDMMC ]Listing dir %s ...\n", path);
	for (;;) {
		/* Verify fs_readdir() */
		res = fs_readdir(&dirp, &entry);

		/* entry.name[0] == 0 means end-of-dir */
		if (res || entry.name[0] == 0) {
			break;
		}

		if (entry.type == FS_DIR_ENTRY_DIR) {
			printf("[ SDMMC ][DIR ] %s\n", entry.name);
		} else {
			printf("[ SDMMC ][FILE] %s (size = %zu)\n",
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

int sd_operations(const char *base_path)
{
    char path[MAX_PATH];
    struct fs_file_t file;
    int ret;

    printf("\n--- SD Card File Operations ---\n");

    /* Build full path for the file: <mount_point>/test.txt */
    snprintf(path, sizeof(path), "%s/%s", base_path, "test.txt");

    fs_file_t_init(&file);

    /* -----------------------------
     * 1) CREATE FILE
     * ----------------------------- */
    ret = fs_open(&file, path, FS_O_CREATE | FS_O_WRITE);
    if (ret) {
        printf("Failed to create file %s (%d)\n", path, ret);
        return ret;
    }
    printf("File created: %s\n", path);

    /* -----------------------------
     * 2) WRITE DATA
     * ----------------------------- */
    const char msg1[] = "Hello from Zephyr SD card!\n";

    ret = fs_write(&file, msg1, strlen(msg1));
    if (ret < 0) {
        printf("Failed to write (%d)\n", ret);
        fs_close(&file);
        return ret;
    }
    printf("Wrote %d bytes\n", ret);

    fs_close(&file);

    /* -----------------------------
     * 3) READ BACK
     * ----------------------------- */
    ret = fs_open(&file, path, FS_O_READ);
    if (ret) {
        printf("Failed to reopen file for reading (%d)\n", ret);
        return ret;
    }

    char read_buf[64] = {0};
    ret = fs_read(&file, read_buf, sizeof(read_buf) - 1);
    if (ret < 0) {
        printf("Read error (%d)\n", ret);
        fs_close(&file);
        return ret;
    }

    printf("Read %d bytes: %s\n", ret, read_buf);

    fs_close(&file);

    /* -----------------------------
     * 4) APPEND SOME EXTRA DATA
     * ----------------------------- */
    ret = fs_open(&file, path, FS_O_WRITE | FS_O_APPEND);
    if (ret) {
        printf("Failed to open for append (%d)\n", ret);
        return ret;
    }

    const char msg2[] = "Appending another line.\n";
    ret = fs_write(&file, msg2, strlen(msg2));
    if (ret < 0) {
        printf("Append error (%d)\n", ret);
        fs_close(&file);
        return ret;
    }

    printf("Appended %d bytes\n", ret);
    fs_close(&file);

    /* -----------------------------
     * 5) LIST DIRECTORY (before deletion)
     * ----------------------------- */
    printf("\nDirectory before deleting file:\n");
    lsdir(base_path);

    /* -----------------------------
     * 6) DELETE FILE
     * ----------------------------- */
    printf("\nDeleting file: %s\n", path);

    ret = fs_unlink(path);
    if (ret) {
        printf("File delete failed (%d)\n", ret);
        return ret;
    }

    printf("File deleted successfully.\n");

    /* -----------------------------
     * 7) LIST DIRECTORY (after deletion)
     * ----------------------------- */
    printf("\nDirectory after deleting file:\n");
    lsdir(base_path);

    return 0;
}
