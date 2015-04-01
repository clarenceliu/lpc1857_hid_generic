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

/* Use the SPIFI library instead of ROM PTR table */
#include "spifilib_api.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
#ifndef SPIFLASH_BASE_ADDRESS
#define SPIFLASH_BASE_ADDRESS 0x14000000
#endif
#ifndef SPIFLASH_BASE2_ADDRESS
#define SPIFLASH_BASE2_ADDRESS 0x80000000
#endif

/* Forward references */
static int32_t progalgo_spiflash_erase_region(uint32_t start, uint32_t size);
static int32_t progalgo_spiflash_write(void *buff, uint32_t start, uint32_t size);
static int32_t progalgo_spiflash_read(void *buff, uint32_t start, uint32_t size);
static void progalgo_spiflash_close(uint32_t start);

/* Function table for exposed API functions */
static const PROGALGOS_T palgos = {
	progalgo_spiflash_init,
	progalgo_spiflash_erase_region,
	progalgo_spiflash_erase_region,
	progalgo_spiflash_write,
	progalgo_spiflash_read,
	progalgo_spiflash_close,
};

/* Number of program regions */
#define PROGRAM_REGIONS 2

/* Multiple regions, only used in Init function */
static const DFUPROG_REGION_T pregions[PROGRAM_REGIONS] = {
	{SPIFLASH_BASE_ADDRESS, 0x08000000, "SPIFLASH@14", &palgos, 2048},
	{SPIFLASH_BASE2_ADDRESS, 0x08000000, "SPIFLASH@80", &palgos, 2048},
};

/* Needed SPIFI library supoprt objects */
static SPIFI_HANDLE_T *pSpifi;

/* SPIFI FLASH good initialization flag */
static uint32_t SpiGood;

/* debug buffer */
static char debugBuffer[256];

#define SPIFIDEBUG(...) { \
	sprintf(debugBuffer, __VA_ARGS__); \
	usbDebug(debugBuffer); \
	}

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/
static uint32_t calculate_divider(uint32_t baseClock, uint32_t target)
{
	uint32_t divider = (baseClock / target);
	
	/* If there is a remainder then increment the dividor so that the resultant
	   clock is not over the target */
	if(baseClock % target) {
		++divider;
	}
	return divider;
}

/* Alternate region */
static uint32_t progalgo_spiflash_chk_alt(uint32_t addr)
{
	addr &= ~0xFF000000;
	addr |= SpiGood;

	return addr;
}

/* Verify a program address range is valid */
static int progalgo_spiflash_progaddrvalid(uint32_t addr, uint32_t size)
{
	int32_t regIndex;

	if (!SpiGood) {
		return 0;
	}

	/* Make sure operation is 32-bit algined */
	if (addr & 0x3) {
		return 0;
	}

	addr = progalgo_spiflash_chk_alt(addr);
	regIndex = algo_root_isRegionValid(addr, size);

	if ((addr >= dfuRegionList.regionList[regIndex].region_addr) &&
		((addr + size) <= (dfuRegionList.regionList[regIndex].region_addr +
		dfuRegionList.regionList[regIndex].region_size))) {
		return size;
	}

	return 0;
}

/* Region erase not supported on SPI FLASH */
static int32_t progalgo_spiflash_erase_region(uint32_t start, uint32_t size)
{
	uint32_t endAddr;
	
	SPIFIDEBUG("SPIFIERASE region: Start %p, size %p\n", (void *) start, (void *) size);
	
	if (progalgo_spiflash_progaddrvalid(start, size) == 0) {
		SPIFIDEBUG("SPIFIERASE Invalid address\n");
		return 0;
	}

	start = progalgo_spiflash_chk_alt(start);
	endAddr = start + size;
	
	if (spifiEraseByAddr(pSpifi, start, endAddr) != SPIFI_ERR_NONE) {
		SPIFIDEBUG("SPIFIERASE spifiEraseByAddr() failed\n");
		return 0;
	}
	SPIFIDEBUG("SPIFIERASE complete.\n");
	return size;
}

/* Write the buffer to the device. Returns 0 if the region cannot
   be written (programming failure or region overlap) or the write
   size>0 if it passed. */
static int32_t progalgo_spiflash_write(void *buff, uint32_t start, uint32_t size)
{
	uint8_t *p8s = (uint8_t *) start, *p8d = (uint8_t *) buff;
	uint32_t vsz = size;
	SPIFI_ERR_T status;
	
	SPIFIDEBUG("SPIFIWRITE region: Start %p, size %p\n", (void *) start, (void *) size);
	
	/* Check address range and alternate range */
	start = progalgo_spiflash_chk_alt(start);
	if (progalgo_spiflash_progaddrvalid(start, size) == 0) {
		SPIFIDEBUG("SPIFIWRITE address invalid\n");
		return 0;
	}
	
	status = spifiProgram(pSpifi, start, (uint32_t *)buff, size);
	if (status != SPIFI_ERR_NONE ) {
		SPIFIDEBUG("SPIFIWRTIE fail: status %s at %p, size %p\n", spifiReturnErrString(status), 
			(void *) start, (void *) size);
		return 0;
	}
	
	/* Verify enter memMode to make it easier. */
	SPIFIDEBUG("SPIFIWRTIE Program complete.  Verifying...\n");
	spifiDevSetMemMode(pSpifi, true);
	while (vsz > 0) {
		if (*p8s != *p8d) {
			SPIFIDEBUG("SPIFIWRTIE verify fail: address %p, is: %x, should be: %x\n", p8s, *p8s, *p8d);
			spifiDevSetMemMode(pSpifi, false);
			return 0;
		}
		p8d++;
		p8s++;
		vsz--;
	}

	/* Leave memMode off by default */
	spifiDevSetMemMode(pSpifi, false);

	return size;
}

/* Read data from the device. Returns 0 if the region cannot
   be read. */
static int32_t progalgo_spiflash_read(void *buff, uint32_t start, uint32_t size)
{
	SPIFIDEBUG("SPIFIREAD @ 0x%p, %p bytes\n", (void *) start, (void *) size);
	
	/* If the read fails report 0 bytes read */
	if (spifiDevRead(pSpifi, start, (uint32_t *)buff, size) != SPIFI_ERR_NONE) {
		return 0;
	}
	
	return size;
}

/* Read data from the device. Returns 0 if the region cannot
   be read. */
static void progalgo_spiflash_close(uint32_t start)
{
	(void) start;
	
	SPIFIDEBUG("SPIFLASH: close()...\r\n");
	
	/* NOTE:  cancel_mem_mode() must be called because of the spifi
	   errata in the Rev. A parts.  If not called, wakeup will take 60s */
	spifiDevSetMemMode(pSpifi, false); //Errata_3_11(Obj);
}

static SPIFI_HANDLE_T *initializeSpifi(void)
{
	int idx;
	int devIdx;
	uint32_t memSize;
	SPIFI_HANDLE_T *pReturnVal;
	static uint32_t lmem[21];
	
	/* Initialize LPCSPIFILIB library, reset the interface */
	spifiInit(LPC_SPIFI_BASE, true);

	/* register support for the family(s) we may want to work with
	     (only 1 is required) */
	spifiRegisterFamily(SPIFI_REG_FAMILY_SpansionS25FLP);
	spifiRegisterFamily(SPIFI_REG_FAMILY_SpansionS25FL1);
	spifiRegisterFamily(SPIFI_REG_FAMILY_MacronixMX25L);
	
	/* Return the number of families that are registered */
	idx = spifiGetSuppFamilyCount();
	
	/* Show all families that are registered */
	for (devIdx = 0; devIdx < idx; ++devIdx) {
		SPIFIDEBUG("FAMILY: %s\r\n", spifiGetSuppFamilyName(devIdx));
	}

	/* Get required memory for detected device, this may vary per device family */
	memSize = spifiGetHandleMemSize(LPC_SPIFI_BASE);
	if (memSize == 0) {
		/* No device detected, error */
		SPIFIDEBUG("SPIFLASH: spifiGetHandleMemSize() failed.\r\n");
		return NULL;
	}

	/* Initialize and detect a device and get device context */
	pReturnVal = spifiInitDevice(&lmem, sizeof(lmem), LPC_SPIFI_BASE, SPIFLASH_BASE_ADDRESS);
	if (pReturnVal == NULL) {
		SPIFIDEBUG("SPIFLASH: spifiInitDevice() failed.\r\n");
		return NULL;
	}
	
	/* Enable quad mode if supported */
	spifiDevSetOpts(pReturnVal, SPIFI_OPT_USE_QUAD, true);
	
	return pReturnVal;
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Initializes device programming capability */
int32_t progalgo_spiflash_init(struct DFUPROG_REGION *reg, int32_t avail)
{
	int idx;
	uint32_t spifiBaseClockRate;
	STATIC const PINMUX_GRP_T spifipinmuxing[] = {
		{0x3, 3,  (SCU_PINIO_FAST | SCU_MODE_FUNC3)},	/* SPIFI CLK */
		{0x3, 4,  (SCU_PINIO_FAST | SCU_MODE_FUNC3)},	/* SPIFI D3 */
		{0x3, 5,  (SCU_PINIO_FAST | SCU_MODE_FUNC3)},	/* SPIFI D2 */
		{0x3, 6,  (SCU_PINIO_FAST | SCU_MODE_FUNC3)},	/* SPIFI D1 */
		{0x3, 7,  (SCU_PINIO_FAST | SCU_MODE_FUNC3)},	/* SPIFI D0 */
		{0x3, 8,  (SCU_PINIO_FAST | SCU_MODE_FUNC3)}	/* SPIFI CS/SSEL */
	};
	
	SpiGood = 0;
	
	SPIFIDEBUG("SPIFLASH: init()...\r\n");
	
	/* Setup SPIFI FLASH pin muxing (QUAD) */
	Chip_SCU_SetPinMuxing(spifipinmuxing, sizeof(spifipinmuxing) / sizeof(PINMUX_GRP_T));
	
	/* SPIFI base clock will be based on the main PLL rate and a divider */
	spifiBaseClockRate = Chip_Clock_GetClockInputHz(CLKIN_MAINPLL);

	/* Setup SPIFI clock to run around 12Mhz. Use divider E for this, as it allows
	   higher divider values up to 256 maximum) */
	Chip_Clock_SetDivider(CLK_IDIV_E, CLKIN_MAINPLL, calculate_divider(spifiBaseClockRate, 12000000));
	Chip_Clock_SetBaseClock(CLK_BASE_SPIFI, CLKIN_IDIVE, true, false);
	
	/* initialize and get a handle to the spifi lib */
	pSpifi = initializeSpifi();
	
	/* If successfull indicate good */
	if (!pSpifi) {
		
		SPIFIDEBUG("SPIFLASH check: initialization failed\r\n");
		return 0;
	}
	
	SpiGood = SPIFLASH_BASE_ADDRESS;
	/* Limit regions to map to what's available */
	if (avail > PROGRAM_REGIONS) {
		avail = PROGRAM_REGIONS;
	}
	
	/* Setup basic regions even if they don't exist */
	for (idx = 0; idx < avail; idx++) {
		reg[idx] = pregions[idx];
		reg[idx].region_size = spifiDevGetInfo(pSpifi, SPIFI_INFO_DEVSIZE);
	}

	SPIFIDEBUG("SPIFLASH check: initialization complete!\r\n");
	return avail;
}
