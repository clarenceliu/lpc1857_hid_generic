/*
 * @brief DFU Utility program for internal EEPROM
 *        This programming algortihm allows reading or writing
 *        to the internal EEPROM on 18xx/43xx devices.
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

/* Forward references */
static int32_t progalgo_inteeprom_erase_region(uint32_t start, uint32_t size);
static int32_t progalgo_inteeprom_write(void *buff, uint32_t start, uint32_t size);
static int32_t progalgo_inteeprom_read(void *buff, uint32_t start, uint32_t size);
static void progalgo_inteeprom_close(uint32_t start);

/* Function table for exposed API functions */
static const PROGALGOS_T palgos = {
	progalgo_inteeprom_init,
	progalgo_inteeprom_erase_region,
	progalgo_inteeprom_erase_region, /* erase all uses same function as region erase */
	progalgo_inteeprom_write,
	progalgo_inteeprom_read,
	progalgo_inteeprom_close,
};

/* Number of program regions */
#define PROGRAM_REGIONS 1

/* Multiple regions, only used in Init function */
static const DFUPROG_REGION_T pregions[PROGRAM_REGIONS] = {
	{0x20040000, 0x00004000, "Internal EEPROM", &palgos, EEPROM_PAGE_SIZE}
};

#define IAP_LOCATION *((uint32_t *) 0x10400100);

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Erase the entire device */
static int32_t progalgo_inteeprom_erase_region(uint32_t start, uint32_t size)
{

	int i, regIndex;
	uint32_t *pEepromMem, pageAddr;

	DFUDEBUG("EEPROMERASE: %p with size %p\n", (void *) start, (void *) size);

	/* Get region index into region structure. Region is guaranteed to
	   be valid */
	regIndex = algo_root_isRegionValid(start, size);
	pageAddr = (start - dfuRegionList.regionList[regIndex].region_addr) /
		EEPROM_PAGE_SIZE;
	pEepromMem = (uint32_t*) EEPROM_ADDRESS(pageAddr, 0);

	for(i = 0; i < EEPROM_PAGE_SIZE / 4; i++) {
		pEepromMem[i] = 0;
		Chip_EEPROM_WaitForIntStatus(LPC_EEPROM, EEPROM_INT_ENDOFPROG);
	}

	return size;
}

/* Write buffer to FLASH */
static int32_t progalgo_inteeprom_write(void *buff, uint32_t start, uint32_t size)
{
	uint8_t *fbuff = (uint8_t *) buff;
	int i, regIndex;
	uint32_t *pEepromMem, pageAddr, wsize = size;

	DFUDEBUG("EEPROMWRITE: %p with size %p\n", (void *) start, (void *) size);

	/* Get region index into region structure. Region is guaranteed to
	   be valid */
	regIndex = algo_root_isRegionValid(start, size);
	pageAddr = (start - dfuRegionList.regionList[regIndex].region_addr) /
		EEPROM_PAGE_SIZE;
	pEepromMem = (uint32_t*) EEPROM_ADDRESS(pageAddr, 0);
	
	/* If the passed size is less than the buffer size, then this is
	   the last fragment of code to program into FLASH. Since images to
	   program may not be on 512 byte boundaries, we'll fill up the
	   unused space with 0xFF to allow programming. */
	if (wsize < EEPROM_PAGE_SIZE) {
		for (; wsize < EEPROM_PAGE_SIZE; wsize++) {
			fbuff[wsize] = 0xFF;
		}

		DFUDEBUG("EEPROMWRITE: Last sector too small, padded %d bytes\n",
				(wsize - size));
	}

	for (i = 0; i < EEPROM_PAGE_SIZE / 4; i++) {
		pEepromMem[i] = fbuff[i];
		Chip_EEPROM_WaitForIntStatus(LPC_EEPROM, EEPROM_INT_ENDOFPROG);
	}

	return size;
}

/* Read data from the device. Returns 0 if the region cannot
   be read. */
static int32_t progalgo_inteeprom_read(void *buff, uint32_t start, uint32_t size)
{
	DFUDEBUG("EEPROMREAD @ 0x%p, %p bytes\n", (void *) start, (void *) size);

	/* SPI FLASH is memory mapped, so just copy the data to the buffer */
	memmove(buff, (void *) start, size);

	return size;
}

/* Close device */
static void progalgo_inteeprom_close(uint32_t start)
{
	(void) start;
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Initializes device programming capability */
int32_t progalgo_inteeprom_init(struct DFUPROG_REGION *reg, int32_t avail)
{
	int32_t nr = 0;
	uint32_t iapTest32 = IAP_LOCATION;

	/* If internal FLASH exists, so does EEPROM */
	if ((iapTest32 >= 0x10400000) && (iapTest32 < 0x10410000)) {
		if (avail > 0) {
			reg[0] = pregions[0];

			/* Init EEPROM */
			Chip_EEPROM_Init(LPC_EEPROM);
			Chip_EEPROM_SetAutoProg(LPC_EEPROM,EEPROM_AUTOPROG_AFT_1WORDWRITTEN);

			DFUDEBUG("EEPROM: 16K available\n");
			
			nr = 1;
		}
	}

	/* Number of regions */
	return nr;
}
