/*
 * @brief DFU Utility program for all root devices
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

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/* Discovered regions */
DFUPROG_REGIONLIST_T dfuRegionList;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Return a region index for a valid range, returns -1 if range is not valid */
int32_t algo_root_isRegionValid(uint32_t addr, uint32_t size)
{
	uint32_t regAddr;
	int32_t validIndex = -1, regIndex = 0;

	while (regIndex < dfuRegionList.num_regions) {
		regAddr = dfuRegionList.regionList[regIndex].region_addr;
		if (addr >= regAddr) {
			if ((addr + size) <= (regAddr + dfuRegionList.regionList[regIndex].region_size)) {
				validIndex = regIndex;
				regIndex = dfuRegionList.num_regions;
			}
		}

		regIndex++;
	}

	if (validIndex < 0) {
		DFUDEBUG("Op on invalid region address/size: %p/%p %d\n", (void *) addr,
			(void *) size, validIndex);
	}

	return validIndex;
}

/* Root region erase function */
int32_t algo_root_erase_region(uint32_t addr, uint32_t size)
{
	int32_t regIndex;

	/* Find match region index for passed range */
	regIndex = algo_root_isRegionValid(addr, size);
	if (regIndex < 0) {
		DFUDEBUG("erase_region invalid: %p:%p\n", (void *) addr, (void *) size);
		return 0;
	}

	return dfuRegionList.regionList[regIndex].pprogalgos->erase_region(addr, size);
}

/* Root region all function (specific region) */
int32_t algo_root_erase_all(uint32_t addr)
{
	int32_t regIndex;

	/* Find match region index for passed range */
	regIndex = algo_root_isRegionValid(addr, 0);
	if (regIndex < 0) {
		DFUDEBUG("eraseall_region invalid: %p %d\n", (void *) addr, regIndex);
		return 0;
	}

	return dfuRegionList.regionList[regIndex].pprogalgos->erase_all(
		dfuRegionList.regionList[regIndex].region_addr,
		dfuRegionList.regionList[regIndex].region_size);
}

/* Write data to a device */
int32_t algo_root_write(void *buff, uint32_t addr, uint32_t size)
{
	int32_t regIndex;

	/* Find match region index for passed range */
	regIndex = algo_root_isRegionValid(addr, size);
	if (regIndex < 0) {
		DFUDEBUG("write_region invalid: %p:%p\n", (void *) addr, (void *) size);
		return 0;
	}

	return dfuRegionList.regionList[regIndex].pprogalgos->write(buff, addr, size);
}

/* Read data from a device */
int32_t algo_root_read(void *buff, uint32_t addr, uint32_t size)
{
	int32_t regIndex;

	/* Find match region index for passed range */
	regIndex = algo_root_isRegionValid(addr, size);
	if (regIndex < 0) {
		DFUDEBUG("read_region invalid: %p:%p\n", (void *) addr, (void *) size);
		return 0;
	}

	return dfuRegionList.regionList[regIndex].pprogalgos->read(buff, addr, size);
}

/* Close a region */
bool algo_root_close(uint32_t addr)
{
	int32_t regIndex;

	/* Find match region index for passed range */
	regIndex = algo_root_isRegionValid(addr, 0);
	if (regIndex < 0) {
		DFUDEBUG("close_region invalid: %p\n", (void *) addr);
		return false;
	}

	dfuRegionList.regionList[regIndex].pprogalgos->close(addr);

	return true;
}

/* Initializes device programming and enumerates prog interfaces */
DFUPROG_REGIONLIST_T *algo_root_init(void)
{
	int regIndex = 0;

	/* Discover regions */
	regIndex += progalgo_spiflash_init(&dfuRegionList.regionList[regIndex],
		(MAXREGIONS - regIndex));
	regIndex += progalgo_intflash_init(&dfuRegionList.regionList[regIndex],
		(MAXREGIONS - regIndex));
	regIndex += progalgo_intiram_init(&dfuRegionList.regionList[regIndex],
		(MAXREGIONS - regIndex));

	/* Initialize EEPROM after internal FLASH */
	regIndex += progalgo_inteeprom_init(&dfuRegionList.regionList[regIndex],
		(MAXREGIONS - regIndex));

	/* Save region count */
	dfuRegionList.num_regions = regIndex;

	/* Return pointer to region data */
	return &dfuRegionList;
}
