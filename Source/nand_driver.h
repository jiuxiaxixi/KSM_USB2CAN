/****************************************Copyright (c)****************************************************
**                                      
**                                 http://www.powermcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               fsmc_nand.h
** Descriptions:            The FSMC NAND application function
**
**--------------------------------------------------------------------------------------------------------
** Created by:              AVRman
** Created date:            2011-2-16
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/

#ifndef __NAND_DRIVER_H
#define __NAND_DRIVER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <string.h>
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  uint8_t MakerID;
  uint8_t DeviceID;
  uint8_t ThirdID;
  uint8_t FourthID;
  uint8_t FiveID;
}
NAND_ID;

typedef struct 
{
  uint16_t Plane;
  uint16_t Block;
  uint16_t Page;
} 
NAND_ADDRESS;

typedef struct 
{
  uint32_t PlanePerChip;      //Plane number(Plane)
  uint32_t BlockPerPlane;     //Plane size(Block)
  uint32_t PagePerBlock;      //Block size(Page)
  uint32_t BytesPerPage;      //Page size(Byte)
  uint32_t SpareAreaSize;     //Spare area size(Byte)
} 
NAND_MEMORY_INFO;
/* Private define ------------------------------------------------------------*/

/* NAND Area definition  for STM3210E-EVAL Board RevD */
#define CMD_AREA                   (uint32_t)(1U<<16)  /* A16 = CLE  high */
#define ADDR_AREA                  (uint32_t)(1U<<17)  /* A17 = ALE high */

#define DATA_AREA                  ((uint32_t)0x00000000) 

/* FSMC NAND memory command */
#define	NAND_CMD_READ_1             ((uint8_t)0x00)
#define	NAND_CMD_READ_TRUE          ((uint8_t)0x30)

#define	NAND_CMD_RDCOPYBACK         ((uint8_t)0x00)
#define	NAND_CMD_RDCOPYBACK_TRUE    ((uint8_t)0x35)

#define NAND_CMD_PAGEPROGRAM        ((uint8_t)0x80)
#define NAND_CMD_PAGEPROGRAM_TRUE   ((uint8_t)0x10)

#define NAND_CMD_COPYBACKPGM        ((uint8_t)0x85)
#define NAND_CMD_COPYBACKPGM_TRUE   ((uint8_t)0x10)
	
#define NAND_CMD_ERASE0             ((uint8_t)0x60)
#define NAND_CMD_ERASE1             ((uint8_t)0xD0)

#define NAND_CMD_READID             ((uint8_t)0x90)
#define NAND_CMD_STATUS             ((uint8_t)0x70)
#define NAND_CMD_RESET              ((uint8_t)0xFF)

#define NAND_CMD_CACHEPGM           ((uint8_t)0x80)
#define NAND_CMD_CACHEPGM_TRUE      ((uint8_t)0x15)

#define NAND_CMD_RANDOMIN           ((uint8_t)0x85)
#define NAND_CMD_RANDOMOUT          ((uint8_t)0x05)
#define NAND_CMD_RANDOMOUT_TRUE     ((uint8_t)0xE0)

#define NAND_CMD_CACHERD_START      ((uint8_t)0x00)
#define NAND_CMD_CACHERD_START2     ((uint8_t)0x31)
#define NAND_CMD_CACHERD_EXIT       ((uint8_t)0x34)

/* NAND memory status */
#define NAND_VALID_ADDRESS         ((uint32_t)0x00000100)
#define NAND_INVALID_ADDRESS       ((uint32_t)0x00000200)
#define NAND_TIMEOUT_ERROR         ((uint32_t)0x00000400)
#define NAND_BUSY                  ((uint32_t)0x00000000)
#define NAND_ERROR                 ((uint32_t)0x00000001)
#define NAND_READY                 ((uint32_t)0x00000040)

/* FSMC NAND memory parameters */
/* for K9F1G08 */
//#define NAND_PAGE_SIZE             ((uint16_t)0x0800) /* 2 * 1024 bytes per page without Spare Area */
//#define NAND_BLOCK_SIZE            ((uint16_t)0x0040) /* 64 pages per block */
//#define NAND_ZONE_SIZE             ((uint16_t)0x0400) /* 1024 Block per zone */
//#define NAND_SPARE_AREA_SIZE       ((uint16_t)0x0040) /* last 64 bytes as spare area */
//#define NAND_MAX_ZONE              ((uint16_t)0x0001) /* 1 zones of 1024 block */

/* FSMC NAND memory address computation */
#define ADDR_1st_CYCLE(ADDR)       (uint8_t)((ADDR)& 0xFF)               /* 1st addressing cycle */
#define ADDR_2nd_CYCLE(ADDR)       (uint8_t)(((ADDR)& 0xFF00) >> 8)      /* 2nd addressing cycle */
#define ADDR_3rd_CYCLE(ADDR)       (uint8_t)(((ADDR)& 0xFF0000) >> 16)   /* 3rd addressing cycle */
#define ADDR_4th_CYCLE(ADDR)       (uint8_t)(((ADDR)& 0xFF000000) >> 24) /* 4th addressing cycle */

/* Private function prototypes -----------------------------------------------*/
void FSMC_NAND_Init(uint8_t ChipIndex,uint32_t ECCPageSize,uint8_t SetupTime);
void FSMC_NAND_ReadID(uint8_t ChipIndex,NAND_ID* pNandId);
void FSMC_NAND_GetMemoryInfo(NAND_ID* pNandId,NAND_MEMORY_INFO *pNandMemInfo);
uint32_t FSMC_NAND_WritePage(uint8_t ChipIndex, NAND_ADDRESS Address,uint8_t *pBuffer, uint32_t NumPageToWrite,uint32_t *pECC);
uint32_t FSMC_NAND_ReadPage(uint8_t ChipIndex, NAND_ADDRESS Address,uint8_t *pBuffer, uint32_t NumPageToRead,uint32_t *pECC);
uint32_t FSMC_NAND_WriteSpareArea(uint8_t ChipIndex, NAND_ADDRESS Address,uint8_t *pBuffer, uint32_t NumSpareAreaTowrite);
uint32_t FSMC_NAND_ReadSpareArea(uint8_t ChipIndex, NAND_ADDRESS Address,uint8_t *pBuffer, uint32_t NumSpareAreaToRead);
uint32_t FSMC_NAND_EraseBlock(uint8_t ChipIndex,NAND_ADDRESS Address,uint32_t NumBlockToErase);
//uint32_t FSMC_NAND_GetBadBlock(uint8_t *pBuffer, NAND_ADDRESS Address, uint32_t NumBlockToGet);
//uint32_t FSMC_NAND_CheckBadBlock(uint8_t *pBuffer, NAND_ADDRESS Address, uint32_t NumBlockToCheck);
uint32_t FSMC_NAND_Reset(uint8_t ChipIndex);
uint32_t FSMC_NAND_GetStatus(uint8_t ChipIndex);
uint32_t FSMC_NAND_ReadStatus(uint8_t ChipIndex);
uint32_t FSMC_NAND_AddressIncrement(uint8_t ChipIndex,NAND_ADDRESS* Address);

void FSMC_NAND_Test(void);
#endif /* __FSMC_NAND_H */

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
