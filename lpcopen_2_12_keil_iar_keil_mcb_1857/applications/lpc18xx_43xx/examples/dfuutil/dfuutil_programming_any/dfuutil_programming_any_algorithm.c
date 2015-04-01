/*
 * @brief DFU Utility program for IRAM/peripheral addresses
 *        This programming algortihm allows reading or writing
 *        any address in the device.
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2012
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
static int32_t progalgo_intiram_erase_region(uint32_t start, uint32_t size);
static int32_t progalgo_intiram_write(void *buff, uint32_t start, uint32_t size);
static int32_t progalgo_intiram_read(void *buff, uint32_t start, uint32_t size);
static void progalgo_intiram_close(uint32_t start);

/* Function table for exposed API functions */
static const PROGALGOS_T palgos = {
	progalgo_intiram_init,
	progalgo_intiram_erase_region,
	progalgo_intiram_erase_region, /* erase all uses same function as region erase */
	progalgo_intiram_write,
	progalgo_intiram_read,
	progalgo_intiram_close,
};

/* Number of program regions */
#define PROGRAM_REGIONS 5

/* Multiple regions, only used in Init function */
static const DFUPROG_REGION_T pregions[PROGRAM_REGIONS] = {
	{0x10000000, 0x00020000, "Local SRAM 1", &palgos, 2048},
	{0x10080000, 0x00012000, "Local SRAM 2", &palgos, 2048},
	{0x20000000, 0x00008000, "AHB SRAM 1", &palgos, 2048},
	{0x20080000, 0x00004000, "AHB SRAM 2", &palgos, 2048},
	{0x200C0000, 0x00004000, "ETB SRAM", &palgos, 2048},
};

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Programming algorithm region erase function */
static int32_t progalgo_intiram_erase_region(uint32_t start, uint32_t size)
{
	if (algo_root_isRegionValid(start, size)) {
		DFUDEBUG("IRAMERASE: %p with size %p\n", (void *) start, (void *) size);
		memset((void *) start, 0, size);
	}
	else {
		size = 0;
	}

	return size;
}

/* Programming algorithm write function */
static int32_t progalgo_intiram_write(void *buff, uint32_t start, uint32_t size)
{
	if (algo_root_isRegionValid(start, size)) {
		DFUDEBUG("IRAMWRITE @ 0x%p, %p bytes\n", (void *) start, (void *) size);
		memmove((void *) start, buff, size);
	}
	else {
		size = 0;
	}

	return size;
}

/* Programming algorithm read function*/
static int32_t progalgo_intiram_read(void *buff, uint32_t start, uint32_t size)
{
	if (algo_root_isRegionValid(start, size)) {
		DFUDEBUG("IRAMREAD @ 0x%p, %p bytes\n", (void *) start, (void *) size);
		memmove(buff, (void *) start, size);
	}
	else {
		size = 0;
	}

	return size;
}

/* Programming algorithm close function */
static void progalgo_intiram_close(uint32_t start)
{
	/* Nothing to do here */
	;
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Initializes device programming capability */
int32_t progalgo_intiram_init(struct DFUPROG_REGION *reg, int32_t avail)
{
	int i;

	/* Limit regions to map to what's available */
	if (avail > PROGRAM_REGIONS) {
		avail = PROGRAM_REGIONS;
	}

	for (i = 0; i < avail; i++) {
		reg[i] = pregions[i];
	}

	return avail;
}
