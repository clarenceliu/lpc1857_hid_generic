/*
 * @brief DFU Utility program for internal FLASH
 *        This programming algortihm allows reading or writing
 *        to the internal FLASH on 18xx/43xx devices.
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "board.h"
#include "dfuutil_programming_api.h"
#include "stdio.h"
#include "string.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* LPC18xx/43xx IAP command defines */
#define IAP_INIT                                    49
#define IAP_PREP_SECS                           50
#define IAP_RAM_TO_FLASH                    51
#define IAP_ERASE_SECS                      52
#define IAP_BLANK_CHECK_SECS            53
#define IAP_READ_PART_ID                    54
#define IAP_READ_BOOT_CODE_VER      55
#define IAP_READ_DEV_SERIAL_NUM     58
#define IAP_COMPARE                             56
#define IAP_REINVOKE_ISP                    57
#define IAP_ERASE_PAGE                      59
#define IAP_SET_ACTIVE_FLASH_BANK   60

/* Maximum size of FLASH */
#define FLASHMAXSIZE (512 * 1024)

/* Some IAP functions need to know the clock speed in KHz */
#define CPUCLOCKFRQINKHZ (SystemCoreClock / 1000)

/* 18xx and 43xx sector information */
typedef struct {
	uint32_t sector_offset;
	uint32_t sector_size;
} SECTOR_INFO_T;
static const SECTOR_INFO_T sectorinfo[] = {
	{0x00000000, 0x00002000},	/* Offset 0x00002000, 8K size */
	{0x00002000, 0x00002000},	/* Offset 0x00004000, 8K size */
	{0x00004000, 0x00002000},	/* Offset 0x00006000, 8K size */
	{0x00006000, 0x00002000},	/* Offset 0x00008000, 8K size */
	{0x00008000, 0x00002000},	/* Offset 0x0000A000, 8K size */
	{0x0000A000, 0x00002000},	/* Offset 0x0000C000, 8K size */
	{0x0000C000, 0x00002000},	/* Offset 0x0000E000, 8K size */
	{0x0000E000, 0x00002000},	/* Offset 0x00010000, 8K size */
	{0x00010000, 0x00010000},	/* Offset 0x00020000, 64K size */
	{0x00020000, 0x00010000},	/* Offset 0x00030000, 64K size */
	{0x00030000, 0x00010000},	/* Offset 0x00040000, 64K size */
	{0x00040000, 0x00010000},	/* Offset 0x00050000, 64K size */
	{0x00050000, 0x00010000},	/* Offset 0x00060000, 64K size */
	{0x00060000, 0x00010000},	/* Offset 0x00070000, 64K size */
	{0x00070000, 0x00010000},	/* Offset 0x00000000, 64K size */
	{0xFFFFFFFF, 0xFFFFFFFF}	/* End of list */
};

/* FLASH page size */
#define PAGE_SIZE 512

/* IAP function and support structures */
typedef void (*IAP)(uint32_t *, uint32_t *);
#define IAP_LOCATION *((uint32_t *) 0x10400100);
static uint32_t command[6], result[5];

/* Forward references */
static int32_t progalgo_intflash_erase_region(uint32_t start, uint32_t size);
static int32_t progalgo_intflash_erase_all(uint32_t start, uint32_t size);
static int32_t progalgo_intflash_write(void *buff, uint32_t start, uint32_t size);
static int32_t progalgo_intflash_read(void *buff, uint32_t start, uint32_t size);
static void progalgo_intflash_close(uint32_t start);

/* Function table for exposed API functions */
static const PROGALGOS_T palgos = {
	progalgo_intflash_init,
	progalgo_intflash_erase_region,
	progalgo_intflash_erase_all,
	progalgo_intflash_write,
	progalgo_intflash_read,
	progalgo_intflash_close,
};

/* Number of program regions */
#define PROGRAM_REGIONS 2

/* Multiple regions, only used in Init function */
static const DFUPROG_REGION_T pregions[PROGRAM_REGIONS] = {
	{0x1A000000, 0x00080000, "FLASH bank A", &palgos, 512},
	{0x1B000000, 0x00080000, "FLASH bank B", &palgos, 512},
};

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Find bank for an address */
static int progalgo_iflash_findbank(uint32_t addr)
{
	int bank = -1;
	int32_t regIndex = algo_root_isRegionValid(addr, 0);

	/* Bank 0 check first, then bank 1 */
	for (bank = 0; bank < PROGRAM_REGIONS; bank++) {
		if ((addr >= pregions[bank].region_addr) &&
			(addr < (pregions[bank].region_addr + pregions[bank].region_size))) {
			return bank;
		}
	}

	/* No matching bank, note this function returns -1 for an error, while
	   most return 0 for error. */
	return bank;
}

/* Verify a program address range is valid. It should not cross banks and
   should not excced the end of the banks address range. */
static int progalgo_iflash_progaddrvalid(uint32_t addr, uint32_t size)
{
	int bank, sg;

	/* Determine bank first */
	bank = progalgo_iflash_findbank(addr);
	if (bank < 0) {
		DFUDEBUG("FLASH: Address does not map to bank\n");
		return 0;
	}

	/* The address must always be 512 byte aligned */
	if ((addr & (PAGE_SIZE - 1)) != 0) {
		DFUDEBUG("FLASH: Address is not 512 byte aligned\n");
		return 0;
	}

	/* The size must be 512 byte aligned */
	sg = size / PAGE_SIZE;
	if (size != (PAGE_SIZE * sg)) {
		DFUDEBUG("FLASH: Size must be a multiple of 512 bytes\n");
		return 0;
	}

	/* The range check can be done here, but both erase and programming
	   functions need to know the starting and ending sector for the address
	   range, so we'll let the sector find function handle it. */
	return (int) size;
}

/* For erase and write prepare operations, the starting and ending sector
   need to be known for an address range. This determines that sector
   range. Before calling this function, the address and size should have
   been validated with a call to progalgo_iflash_progaddrvalid(). */
static int progalgo_iflash_find_sectorrange(uint32_t addr, uint32_t size,
											int *bank, uint32_t *secstart, uint32_t *secend, int *aligned)
{
	int idx;
	uint32_t addrend, addrbase, regstart, regend;

	/* Bank number for address range */
	*bank = progalgo_iflash_findbank(addr);

	/* Base address for bank */
	addrbase = pregions[*bank].region_addr;
	addrend = addr + size - 1;

	/* The aligned flag is returned if the address range exactly maps to a
	   range of sectors boundaries. For erase operations, the aligned flag
	   must be set. */
	*aligned = 0;

	/* Find starting sector */
	idx = 0;
	*secstart = 0xFFFFFFFF;
	while ((*secstart == 0xFFFFFFFF) && (idx != -1)) {
		if (sectorinfo[idx].sector_offset == 0xFFFFFFFF) {
			idx = -1;
		}
		else {
			regstart = addrbase + sectorinfo[idx].sector_offset;
			regend = regstart + sectorinfo[idx].sector_size - 1;
			if ((addr >= regstart) && (addr <= regend)) {
				/* Falls in sector range */
				*secstart = (uint32_t) idx;

				/* Is starting address aligned? */
				if (addr == regstart) {
					*aligned = 1;
				}
				else {
					*aligned = 0;
				}
			}
			else {
				/* Check next range */
				idx++;
			}
		}
	}

	/* A starting sector was found? */
	if (idx == -1) {
		return 0;
	}

	/* Find the last sector for the address range */
	*secend = 0xFFFFFFFF;
	while ((*secend == 0xFFFFFFFF) && (idx != -1)) {
		if (sectorinfo[idx].sector_offset == 0xFFFFFFFF) {
			idx = -1;
		}
		else {
			regstart = addrbase + sectorinfo[idx].sector_offset;
			regend = regstart + sectorinfo[idx].sector_size - 1;
			if ((addrend >= regstart) && (addrend <= regend)) {
				/* Falls in sector range */
				*secend = (uint32_t) idx;

				/* Is ending address aligned? */
				if (addrend == regend) {
					*aligned = 1;
				}
				else {
					*aligned = 0;
				}
			}
			else {
				/* Check next range */
				idx++;
			}
		}
	}

	/* A ending sector was found? */
	if (idx == -1) {
		return 0;
	}

	return size;
}

/* Prepare a range of sectors for write/erase, returns a IAP status
   value of IAP_* */
static int progalgo_iflash_prepwrite(uint32_t bank, uint32_t secstart,
									 uint32_t secend)
{
	command[0] = IAP_PREP_SECS;
	command[1] = secstart;
	command[2] = secend;
	command[3] = bank;
	iap_entry(command, result);

	if (result[0] != IAP_CMD_SUCCESS) {
		DFUDEBUG("FLASH: Error preparing sectors %d-%d (bank %d), error=%d\n",
				secstart, secend, bank, result[0]);
	}

	return result[0];
}

/* Erase a range of sectors */
static int progalgo_iflash_erasesectors(uint32_t bank, uint32_t secstart,
										uint32_t secend)
{
	command[0] = IAP_ERASE_SECS;
	command[1] = secstart;
	command[2] = secend;
	command[3] = CPUCLOCKFRQINKHZ;
	command[4] = bank;
	iap_entry(command, result);

	if (result[0] != IAP_CMD_SUCCESS) {
		DFUDEBUG("FLASH: Error erasing sectors %d-%d (bank %d)\n",
				secstart, secend, bank);
	}

	return result[0];
}

/* Erase a region of FLASH memory */
static int32_t progalgo_intflash_erase_region(uint32_t start, uint32_t size)
{
	int bank, aligned;
	uint32_t secstart, secend;

	DFUDEBUG("FLASHERASE: %p with size %p\n", (void *) start, (void *) size);

	/* Basic verification first of input parameters */
	if (progalgo_iflash_progaddrvalid(start, size) == 0) {
		DFUDEBUG("FLASHERASE: address/size validation failure\r\n");
		return 0;
	}

	/* Get sector and bank info for the operation */
	if (progalgo_iflash_find_sectorrange(start, size, &bank,
										 &secstart, &secend, &aligned) == 0) {
		DFUDEBUG("FLASHERASE: sector range lookup failure\r\n");
		return 0;
	}

	/* Must be aligned to a sector range! */
	if (aligned == 0) {
		DFUDEBUG("FLASHERASE: Address range must be sector aligned\r\n");
		return 0;
	}

	DFUDEBUG("FLASHERASE: Bank %d, Start sec %d, End sec %d\n", bank, secstart, secend);

	/* Prepare for write */
	if (progalgo_iflash_prepwrite(bank, secstart, secend) != IAP_CMD_SUCCESS) {
		return 0;
	}

	/* Erase sectors */
	if (progalgo_iflash_erasesectors(bank, secstart, secend) != IAP_CMD_SUCCESS) {
		return 0;
	}

	/* Verify they are erased */
	command[0] = IAP_BLANK_CHECK_SECS;
	command[1] = secstart;
	command[2] = secend;
	command[3] = bank;
	iap_entry(command, result);
	if (result[0] != IAP_CMD_SUCCESS) {
		DFUDEBUG("FLASHERASE: Error erasing sectors\r\n");
		return 0;
	}

	return size;
}

/* Erase the entire device */
static int32_t progalgo_intflash_erase_all(uint32_t start, uint32_t size)
{
	uint32_t sz;
	int32_t regIndex;

	/* Get region index into region structure. Region is guaranteed to
	   be valid */
	regIndex = algo_root_isRegionValid(start, size);

	/* Erase 1 or 2 banks */
	sz = progalgo_intflash_erase_region(dfuRegionList.regionList[regIndex].region_addr,
		dfuRegionList.regionList[regIndex].region_size);

	return sz;
}

/* Write buffer to FLASH */
static int32_t progalgo_intflash_write(void *buff, uint32_t start, uint32_t size)
{
	uint8_t *fbuff = (uint8_t *) buff;
	int bank, aligned;
	uint32_t secstart, secend, wsize = size;

	DFUDEBUG("FLASHWRITE: %p with size %p\n", (void *) start, (void *) size);

	/* Verify size doesn't exceed buffer length. This check is not
	   really needed as the DFU streamer keeps the transfer at or
	   below the buffer size, but it's here for debug. */
	if (size > PAGE_SIZE) {
		DFUDEBUG("FLASHWRITE: Program buffer too big\r\n");
		return 0;
	}

	/* If the passed size is less than the buffer size, then this is
	   the last fragment of code to program into FLASH. Since images to
	   program may not be on 512 byte boundaries, we'll fill up the
	   unused space with 0xFF to allow programming. */
	if (wsize < PAGE_SIZE) {
		for (; wsize < PAGE_SIZE; wsize++) {
			fbuff[wsize] = 0xFF;
		}

		DFUDEBUG("FLASHWRITE: Last sector too small, padded %d bytes\n",
				(wsize - size));
	}

	/* Verify basic parameters */
	if (progalgo_iflash_progaddrvalid(start, wsize) == 0) {
		DFUDEBUG("FLASHWRITE: Input address/size is not valid\r\n");
		return 0;
	}

	/* Get sector and bank info for the operation */
	if (progalgo_iflash_find_sectorrange(start, wsize, &bank,
										 &secstart, &secend, &aligned) == 0) {
		DFUDEBUG("FLASHWRITE: sector range lookup failure\r\n");
		return 0;
	}

	/* Prepare for write */
	if (progalgo_iflash_prepwrite(bank, secstart, secend) != IAP_CMD_SUCCESS) {
		return 0;
	}

	/* Program the data */
	command[0] = IAP_RAM_TO_FLASH;
	command[1] = start;
	command[2] = (uint32_t) buff;
	command[3] = PAGE_SIZE;
	command[4] = CPUCLOCKFRQINKHZ;
	iap_entry(command, result);
	if (result[0] != IAP_CMD_SUCCESS) {
		DFUDEBUG("FLASHWRITE: Error programming address range\r\n");
		return 0;
	}

	/* Verify */
	command[0] = IAP_COMPARE;
	command[1] = start;
	command[2] = (uint32_t) buff;
	command[3] = PAGE_SIZE;
	iap_entry(command, result);
	if (result[0] != IAP_CMD_SUCCESS) {
		DFUDEBUG("FLASHWRITE: Verify error on program\r\n");
		return 0;
	}

	return size;
}

/* Read data from the device. Returns 0 if the region cannot
   be read. */
static int32_t progalgo_intflash_read(void *buff, uint32_t start, uint32_t size)
{
	DFUDEBUG("FLASHREAD @ 0x%p, %p bytes\n", (void *) start, (void *) size);

	/* SPI FLASH is memory mapped, so just copy the data to the buffer */
	memmove(buff, (void *) start, size);

	return size;
}

/* Close device */
static void progalgo_intflash_close(uint32_t start)
{
	(void) start;
}

/* Attempt to compute FLASH size from ID */
static uint32_t progalgo_iflash_getsize(int bank, uint32_t part_id2)
{
	uint32_t flash_size;
	const int bank_shift[2] = {0, 4};
	const int bank_mask[2]  = {0x0F, 0xF0};

	flash_size = ((part_id2 & bank_mask[bank]) >> bank_shift[bank]) * 0x10000;
	if (FLASHMAXSIZE <= flash_size) {
		flash_size = 0;
	}
	else {
		flash_size = FLASHMAXSIZE - flash_size;
	}

	return flash_size;
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Initializes device programming capability */
int32_t progalgo_intflash_init(struct DFUPROG_REGION *reg, int32_t avail)
{
	int idx, ni = 0;
	uint32_t iapTest32 = IAP_LOCATION;


	/* Limit regions to map to what's available */
	if (avail > PROGRAM_REGIONS) {
		avail = PROGRAM_REGIONS;
	}

	/* Device IDs */
	result[1] = * (uint32_t *) 0x40045000;
	result[2] = * (uint32_t *) 0x4004500c;
	DFUDEBUG("FLASHINIT: ID1/2 = 0x%08x/0x%08x\n", result[1], result[2]);

	if ((iapTest32 >= 0x10400000) && (iapTest32 < 0x10410000)) {
		/* Initialize IAP */
		command[0] = IAP_INIT;
		iap_entry(command, result);
		if (result[0] == IAP_CMD_SUCCESS) {
			/* Setup region sizes based on part ID */
			for (idx = 0; idx < avail; idx++) {
				reg[ni] = pregions[idx];
				reg[ni].region_size = progalgo_iflash_getsize(idx, result[2]);
				if (reg[ni].region_size > 0) {
					ni++;
				}
			}
		}
	}

	/* Number of regions */
	return ni;
}
