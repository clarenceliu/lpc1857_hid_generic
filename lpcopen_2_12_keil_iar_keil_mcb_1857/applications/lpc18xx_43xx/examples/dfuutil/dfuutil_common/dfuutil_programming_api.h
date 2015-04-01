/*
 * @brief Programming API used with DFU Utility programmer
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

#ifndef __DFUSEC_PROGRAMMING_API_H_
#define __DFUSEC_PROGRAMMING_API_H_

#include "board.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* Forward reference */
struct DFUPROG_REGION;

/*
 * Initializes device programming capability
 * First parameter is a pointer to a free region structure array
 * Second parameter is the number of free elements in the region structure array
 * Returns number of regions added or 0
 * Initializes device programming capability. Returns a pointer to the
 * programming buffer, the programming buffer size, and a pointer to the
 * DFU programming region/API structure used by the DFU streamer.
 */
typedef int32_t (*progalgo_init)(struct DFUPROG_REGION *, int32_t);

/*
 * Pointer to programming algorithm region erase function
 * First parameter is a pointer to memory to erase
 * Second parameter is the number of bytes to erase
 * Returns bytes erased or 0 on failure
 */
typedef int32_t (*progalgo_erase_region)(uint32_t, uint32_t);

/*
 * Pointer to programming algorithm erase all function
 * First parameter is a pointer to memory base address to erase
 * First parameter is a number of bytes of the region
 * Returns bytes erased or 0 on failure
 */
typedef int32_t (*progalgo_erase_all)(uint32_t, uint32_t);

/*
 * Pointer to programming algorithm write function
 * First parameter is the pointer to the buffer to write
 * Second parameter is the starting address to write (32-bit aligned)
 * Third parameter is the number of bytes to write
 * Returns bytes written or 0 on failure
 */
typedef int32_t (*progalgo_write)(void *, uint32_t, uint32_t);

/*
 * Pointer to programming algorithm read function
 * First parameter is the pointer to the buffer to fill
 * Second parameter is the starting address to read (32-bit algined)
 * Third parameter is the number of bytes to read
 * Returns bytes read or 0 on failure
 */
typedef int32_t (*progalgo_read)(void *, uint32_t, uint32_t);

/* Pointer to programming algorithm close function */
typedef void (*progalgo_close)(uint32_t start);

/* Pointer to programming algorithm region erase function */
typedef struct {
	progalgo_init			init;
	progalgo_erase_region	erase_region;
	progalgo_erase_all		erase_all;
	progalgo_write			write;
	progalgo_read			read;
	progalgo_close			close;
} PROGALGOS_T;

/* Maximum number of supported regions and algorithms */
#define MAXREGIONS 12

/* Data specific to a programming algorithm */
typedef struct DFUPROG_REGION {
	uint32_t region_addr;			/* Offset address */
	uint32_t region_size;			/* Size in bytes */
	const char *regname;			/* Region name */
	const PROGALGOS_T *pprogalgos;	/* Programming algos */
	/* This must be a minimum of 64 bytes and a max of 4096 bytes. This must
	   be a factor (1x, 2x, 3x, etc.) of MAXP size */
	uint32_t buffer_size;
} DFUPROG_REGION_T;

/* Memory and FLASH based region lists */
extern DFUPROG_REGION_T *regionIntFlash;
extern DFUPROG_REGION_T *regionSPIFIFlash;
extern DFUPROG_REGION_T *regionRAM;

/* Programming information structure defined for a algorithm */
typedef struct {
	/* Number of program regions on the platform */
	int32_t num_regions;						/* Number of supported regions */
	DFUPROG_REGION_T regionList[MAXREGIONS];	/* Address and region size array */
	uint32_t ver;								/* Versioning info */
} DFUPROG_REGIONLIST_T;

/* Discovered regions */
extern DFUPROG_REGIONLIST_T dfuRegionList;

/* Possible commands from the host machine */
typedef enum {
	DFU_HOSTCMD_READIDS,		/* Read device IDs */
	DFU_HOSTCMD_SETDEBUG,   	/* Enables/disables debug output */
	DFU_HOSTCMD_PROGOTP,		/* Program OTP key */
	DFU_HOSTCMD_READOTP,		/* Read OTP key */
	DFU_HOSTCMD_STARTNEWSESS,	/* Starts a program session */
	DFU_HOSTCMD_STARTENCSESS,	/* Starts an encryption session */
	DFU_HOSTCMD_ERASE_ALL,		/* Erase the entire device */
	DFU_HOSTCMD_ERASE_REGION,	/* Erase a region defined with addr/size */
	DFU_HOSTCMD_PROGRAM,		/* Program a region defined with addr/size */
	DFU_HOSTCMD_READBACK,		/* Read a region defined with addr/size */
	DFU_HOSTCMD_RESET,			/* Reset the device/board */
	DFU_HOSTCMD_EXECUTE
} DFU_HOSTCMD_T;

/* Host DFU download packet header. This is appended to a data packet when
 * programming a region. */
typedef struct {
	/* Host command : A value of type DFU_HOSTCMD_T */
	uint32_t hostCmd;
	/* SETDEBUG command: 0xFFFFFFFF = USB based debug, 0-n is UART debug */
	/* PROGOTP command: Program OTP by key index, 1=AESKEY1, 2=AESKEY2, others=otp_ProgGP0 */
	/* READOTP command: Read OTP by key index, 1=AESKEY1, 2=AESKEY2, others=otp_ProgGP0 */
	/* READIDS command: Read device ID (2 words) */
	/* STARTENC command:  */
	/* ERASE/PROG/READ command: start of program/erase/read region, or execute address */
	/* RESET command: Argument not used */
	/* EXECUTE command: Address to jump to */
	uint32_t addr;
	uint32_t size;			/* Size of program/erase/read region */
	uint32_t magic;			/* Should be DFUPROG_VALIDVAL */
//	uint32_t options[4];	/* Option values */
} DFU_FROMHOST_PACKETHDR_T;

/*
 * Magic value used to indicate DFU programming algorithm and DFU Utility
 * support. This is used to lock algorithm support to DFU Utility tool
 * version to prevent issues with non-compatible versions. The upper
 * 16 bits contain a magic number and the lower 16 bits contain the
 * version number in x.y format (1.10 = 0x010B).
 */
#define DFUPROG_VALIDVAL (0x18430000 | (0x010B))

/*
 * DFU operational status returned from programming algorithm, used
 * by host to monitor status of board
 */
typedef enum {
	DFU_OPSTS_IDLE,			/* Idle, can accept new host command */
	DFU_OPSTS_ERRER,		/* Erase error */
	DFU_OPSTS_PROGER,		/* Program error */
	DFU_OPSTS_READER,		/* Readback error */
	DFU_OPSTS_ERRUN,		/* Unknown error */
	DFU_OPSTS_VERERR,		/* Mismatched version error */
	DFU_OPSTS_READBUSY,		/* Device is busy reading a block of data */
	DFU_OPSTS_READTRIG,		/* Device data is ready to read */
	DFU_OPSTS_READREADY,	/* Block of data is ready */
	DFU_OPSTS_ERASE_ALL_ST,	/* Device performing full erase */
	DFU_OPSTS_ERASE_ST,		/* Device performing region erase */
	DFU_OPSTS_ERASE,		/* Device is currently erasing */
	DFU_OPSTS_PROG,			/* Device is currently programming a range */
	DFU_OPSTS_PROG_RSVD,	/* Reserved state, not used */
	DFU_OPSTS_PROG_STREAM,	/* Device is in buffer streaming mode */
	DFU_OPSTS_RESET,		/* Will shutdown and reset */
	DFU_OPSTS_EXEC,			/* Will shutdown USB and start execution */
	DFU_OPSTS_LOOP			/* Loop on error after DFU status check */
} DFU_OPSTS_T;

/*
 * When sending data to the host machine, a packet header is appended to
 * the data payload to indicate the state of the target and any debug or
 * error messages.
 */
typedef struct {
	uint32_t cmdResponse;	/* Command responding from host */
	uint32_t progStatus;	/* Current status of system */
	uint32_t strBytes;		/* Number of bytes in string field */
	uint32_t reserved;
} DFU_TOHOST_PACKETHDR_T;

/* Function list for root level API calls */
DFUPROG_REGIONLIST_T *algo_root_init(void);
int32_t algo_root_erase_region(uint32_t addr, uint32_t size);
int32_t algo_root_erase_all(uint32_t addr);
int32_t algo_root_write(void *buff, uint32_t addr, uint32_t size);
int32_t algo_root_read(void *buff, uint32_t addr, uint32_t size);
bool algo_root_close(uint32_t addr);

/* Programming algorithm Init functins */
int32_t progalgo_intflash_init(struct DFUPROG_REGION *reg, int32_t avail);
int32_t progalgo_spiflash_init(struct DFUPROG_REGION *reg, int32_t avail);
int32_t progalgo_intiram_init(struct DFUPROG_REGION *reg, int32_t avail);
int32_t progalgo_inteeprom_init(struct DFUPROG_REGION *reg, int32_t avail);

/* Return a region index for a valid range, returns -1 if range is not valid */
int32_t algo_root_isRegionValid(uint32_t addr, uint32_t size);

/* Debug output macro */
#define DFUDEBUG(...) { \
	char tempSTR[256]; \
	sprintf(tempSTR, __VA_ARGS__); \
	usbDebug(tempSTR); \
	}

/**
 * @brief	Queues a message for DFU status transfer
 * @param	str	: Message to queue
 * @return	Nothing
 */
void usbDebug(char *str);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __DFUSEC_PROGRAMMING_API_H_ */
