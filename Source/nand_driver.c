/****************************************Copyright (c)****************************************************
**                                      
**                                 http://www.powermcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               fsmc_nand.c
** Descriptions:            The FSMC Nand application function
**
**--------------------------------------------------------------------------------------------------------
** Created by:              AVRman
** Created date:            2011-12-30
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

/* Includes ------------------------------------------------------------------*/
#include "nand_driver.h"
#include "delay.h"
#include <stdlib.h>

/* Private define ------------------------------------------------------------*/
uint32_t NAND_FlashStartAddr[2]={0x70000000,0x80000000};
NAND_MEMORY_INFO NAND_MemInfo[2];

/* Private variables ---------------------------------------------------------*/
//static uint8_t TxBuffer [NAND_PAGE_SIZE];
//static uint8_t RxBuffer [NAND_PAGE_SIZE];

uint32_t NAND_GetRowAddr(uint8_t ChipIndex,NAND_ADDRESS Address)
{
  return (Address.Page + (Address.Block + (Address.Plane * NAND_MemInfo[ChipIndex].BlockPerPlane)) * NAND_MemInfo[ChipIndex].PagePerBlock);
}

/*******************************************************************************
* Function Name  : FSMC_NAND_Init
* Description    : Configures the FSMC and GPIOs to interface with the NAND memory.
*                  This function must be called before any write/read operation
*                  on the NAND.
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void FSMC_NAND_Init(uint8_t ChipIndex,uint32_t ECCPageSize,uint8_t SetupTime)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  FSMC_NANDInitTypeDef FSMC_NANDInitStructure;
  FSMC_NAND_PCCARDTimingInitTypeDef  p;
  if(ChipIndex == 0){
    /* FSMC NAND Bank Cmd Test */
    FSMC_NANDCmd(FSMC_Bank2_NAND, DISABLE);
  }else{
    /* FSMC NAND Bank Cmd Test */
    FSMC_NANDCmd(FSMC_Bank3_NAND, DISABLE);
  }
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE , ENABLE);
  if(ChipIndex > 0){
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG , ENABLE);
  }
  /*-- GPIO Configuration ------------------------------------------------------*/
  /* CLE, ALE, D0->D3, NOE, NWE and NCE2  NAND pin configuration  */

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); 
  RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC,ENABLE);

  /* D0->D3,*/
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource0 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource1 , GPIO_AF_FSMC);

   /* D4->D7 NAND pin configuration  */ 
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource7 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource8 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource9 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource10 , GPIO_AF_FSMC);



  /*-- GPIO Configuration ------------------------------------------------------*/
  /* CLE, ALE, D0->D7, NOE, NWE and NCE2  NAND pin configuration  */
  GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* D4->D7 NAND pin configuration  */  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  

  /*CLE, ALE */
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource11 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12 , GPIO_AF_FSMC);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
 
  /*NOE, NWE*/
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource4 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource5 , GPIO_AF_FSMC); 

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  /*PD7 -> NCE2    PG9 -> NCE3*/
  if(ChipIndex == 0){
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource7 , GPIO_AF_FSMC); 
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
  }else{
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource9 , GPIO_AF_FSMC);
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
    GPIO_Init(GPIOG, &GPIO_InitStructure);
  }

  /*NWAIT*/  
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource6 , GPIO_AF_FSMC);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN ;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /*-- FSMC Configuration ------------------------------------------------------*/
  p.FSMC_SetupTime = 1*SetupTime;
  p.FSMC_WaitSetupTime = 3*SetupTime;
  p.FSMC_HoldSetupTime = 2*SetupTime;
  p.FSMC_HiZSetupTime = 1*SetupTime;
  if(ChipIndex == 0){
    FSMC_NANDInitStructure.FSMC_Bank = FSMC_Bank2_NAND;
  }else{
    FSMC_NANDInitStructure.FSMC_Bank = FSMC_Bank3_NAND;
  }
  FSMC_NANDInitStructure.FSMC_Waitfeature = FSMC_Waitfeature_Disable;
  FSMC_NANDInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_8b;
  FSMC_NANDInitStructure.FSMC_ECC = FSMC_ECC_Enable;
  switch(ECCPageSize){
    case 256:
      FSMC_NANDInitStructure.FSMC_ECCPageSize = FSMC_ECCPageSize_256Bytes;
      break;
    case 512:
      FSMC_NANDInitStructure.FSMC_ECCPageSize = FSMC_ECCPageSize_512Bytes;
      break;
    case 1024:
      FSMC_NANDInitStructure.FSMC_ECCPageSize = FSMC_ECCPageSize_1024Bytes;
      break;
    case 2048:
      FSMC_NANDInitStructure.FSMC_ECCPageSize = FSMC_ECCPageSize_2048Bytes;
      break;
    case 4096:
      FSMC_NANDInitStructure.FSMC_ECCPageSize = FSMC_ECCPageSize_4096Bytes;
      break;
    case 8192:
      FSMC_NANDInitStructure.FSMC_ECCPageSize = FSMC_ECCPageSize_8192Bytes;
      break;
    default:
      FSMC_NANDInitStructure.FSMC_ECCPageSize = FSMC_ECCPageSize_2048Bytes;
  }
  FSMC_NANDInitStructure.FSMC_TCLRSetupTime = 0;
  FSMC_NANDInitStructure.FSMC_TARSetupTime = 0;
  FSMC_NANDInitStructure.FSMC_CommonSpaceTimingStruct = &p;
  FSMC_NANDInitStructure.FSMC_AttributeSpaceTimingStruct = &p;

  FSMC_NANDInit(&FSMC_NANDInitStructure);
  if(ChipIndex == 0){
    /* FSMC NAND Bank Cmd Test */
    FSMC_NANDCmd(FSMC_Bank2_NAND, ENABLE);
  }else{
    /* FSMC NAND Bank Cmd Test */
    FSMC_NANDCmd(FSMC_Bank3_NAND, ENABLE);
  }
  //FSMC_NAND_Reset(ChipIndex);
}

/******************************************************************************
* Function Name  : FSMC_NAND_ReadID
* Description    : Reads NAND memory's ID.
* Input          : - NAND_ID: pointer to a NAND_IDTypeDef structure which will hold
*                    the Manufacturer and Device ID.
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void FSMC_NAND_ReadID(uint8_t ChipIndex,NAND_ID* pNandId)
{
  /* Send Command to the command area */ 	
  *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | CMD_AREA) = NAND_CMD_READID;
  *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = NAND_CMD_READ_1;
//  delay_us(1);
//  /* Sequence to read ID from NAND flash */	
//  data = *(vu32 *)(NAND_FlashStartAddr[ChipIndex] | DATA_AREA);

//  NandId->Maker_ID   = ADDR_1st_CYCLE (data);
//  NandId->Device_ID  = ADDR_2nd_CYCLE (data);
//  NandId->Third_ID   = ADDR_3rd_CYCLE (data);
//  NandId->Fourth_ID  = ADDR_4th_CYCLE (data);  
  pNandId->MakerID   = *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | DATA_AREA);
  pNandId->DeviceID  = *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | DATA_AREA);
  pNandId->ThirdID   = *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | DATA_AREA);
  pNandId->FourthID  = *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | DATA_AREA);
  pNandId->FiveID = *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | DATA_AREA);
  
}

/******************************************************************************
* Function Name  : FSMC_NAND_GetMemoryInfo
* Description    : Reads NAND memory's ID.
* Input          : - NAND_ID: pointer to a NAND_IDTypeDef structure which will hold
*                    the Manufacturer and Device ID.
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void FSMC_NAND_GetMemoryInfo(NAND_ID* pNandId,NAND_MEMORY_INFO *pNandMemInfo)
{
  int i=0;
  pNandMemInfo->BytesPerPage = 1;
  for(i=0;i<(pNandId->FourthID&0x03);i++){
    pNandMemInfo->BytesPerPage *= 2;
  }
  pNandMemInfo->BytesPerPage *= 1024;
  
  pNandMemInfo->PagePerBlock = 64;
  for(i=0;i<((pNandId->FourthID>>4)&0x03);i++){
    pNandMemInfo->PagePerBlock *= 2;
  }
  pNandMemInfo->PagePerBlock *= 1024;
  pNandMemInfo->PagePerBlock /= pNandMemInfo->BytesPerPage;
  
  pNandMemInfo->SpareAreaSize = (pNandMemInfo->BytesPerPage/512)*((((pNandId->FourthID>>2)&0x01)>0?16:8));
  
  pNandMemInfo->BlockPerPlane = 8;
  for(i=0;i<((pNandId->FiveID>>4)&0x07);i++){
    pNandMemInfo->BlockPerPlane *= 2;
  }
  pNandMemInfo->BlockPerPlane *= 1024;
  pNandMemInfo->BlockPerPlane /= (pNandMemInfo->BytesPerPage/1024)*pNandMemInfo->PagePerBlock;
  
  pNandMemInfo->PlanePerChip = 1;
  for(i=0;i<((pNandId->FiveID>>3)&0x03);i++){
    pNandMemInfo->PlanePerChip *= 2;
  }
}

/******************************************************************************
* Function Name  : FSMC_NAND_WriteSmallPage
* Description    : This routine is for writing one or several 512 Bytes Page size.
* Input          : - pBuffer: pointer on the Buffer containing data to be written   
*                  - Address: First page address
*                  - NumPageToWrite: Number of page to write  
* Output         : None
* Return         : New status of the NAND operation. This parameter can be:
*                   - NAND_TIMEOUT_ERROR: when the previous operation generate 
*                     a Timeout error
*                   - NAND_READY: when memory is ready for the next operation 
*                  And the new status of the increment address operation. It can be:
*                  - NAND_VALID_ADDRESS: When the new address is valid address
*                  - NAND_INVALID_ADDRESS: When the new address is invalid address
* Attention		 : None
*******************************************************************************/
uint32_t FSMC_NAND_WritePage(uint8_t ChipIndex, NAND_ADDRESS Address,uint8_t *pBuffer, uint32_t NumPageToWrite,uint32_t *pECC)
{
  uint32_t index = 0x00, numpagewritten = 0x00, addressstatus = NAND_VALID_ADDRESS;
  uint32_t status = NAND_READY, size = 0x00;
  while((NumPageToWrite != 0x00) && (addressstatus == NAND_VALID_ADDRESS) && (status == NAND_READY))
  {
    /* Page write command and address */
    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | CMD_AREA) = NAND_CMD_PAGEPROGRAM;

    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = 0x00;
    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = 0X00;
    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = ADDR_1st_CYCLE(NAND_GetRowAddr(ChipIndex,Address));
    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = ADDR_2nd_CYCLE(NAND_GetRowAddr(ChipIndex,Address));
    if((NAND_MemInfo[ChipIndex].PagePerBlock*NAND_MemInfo[ChipIndex].BlockPerPlane)&0xFF0000){
      *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = ADDR_3rd_CYCLE(NAND_GetRowAddr(ChipIndex,Address));
    }else if((NAND_MemInfo[ChipIndex].PagePerBlock*NAND_MemInfo[ChipIndex].BlockPerPlane)&0xFF000000){
      *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = ADDR_4th_CYCLE(NAND_GetRowAddr(ChipIndex,Address));
    }
    /* Calculate the size */
    size = NAND_MemInfo[ChipIndex].BytesPerPage + (NAND_MemInfo[ChipIndex].BytesPerPage * numpagewritten);
    
    if(ChipIndex == 0){
      //使能ECC
      FSMC_NANDECCCmd(FSMC_Bank2_NAND, ENABLE);
    }else{
      //使能ECC
      FSMC_NANDECCCmd(FSMC_Bank3_NAND, ENABLE);
    }
    /* Write data */
    for(; index < size; index++)
    {
      *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | DATA_AREA) = pBuffer[index];
    }
    if(ChipIndex == 0){
      while ( FSMC_GetFlagStatus (FSMC_Bank2_NAND , FSMC_FLAG_FEMPT)  == RESET  );///  等待标志位
      *pECC++ = FSMC_GetECC(FSMC_Bank2_NAND); 
      //复位ECC
      FSMC_NANDECCCmd(FSMC_Bank2_NAND, DISABLE);
    }else{
      while ( FSMC_GetFlagStatus (FSMC_Bank3_NAND , FSMC_FLAG_FEMPT)  == RESET  );///  等待标志位
      *pECC++ = FSMC_GetECC(FSMC_Bank3_NAND);
      //复位ECC
      FSMC_NANDECCCmd(FSMC_Bank3_NAND, DISABLE);
    }
    //printf("\r\nWrite ECC: 0x%X \r\n", *pECC);
    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | CMD_AREA) = NAND_CMD_PAGEPROGRAM_TRUE;
    delay_us(1);
    /* 读忙脚 */
    while( GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6) == 0 );
    
    /* Check status for successful operation */
    status = FSMC_NAND_GetStatus(ChipIndex);
    
    if(status == NAND_READY)
    {
      numpagewritten++;

      NumPageToWrite--;

      /* Calculate Next small page Address */
      addressstatus = FSMC_NAND_AddressIncrement(ChipIndex,&Address);
    }    
  }
  
  return (status | addressstatus);
}

/******************************************************************************
* Function Name  : FSMC_NAND_ReadSmallPage
* Description    : This routine is for sequential read from one or several 
*                  512 Bytes Page size.
* Input          : - pBuffer: pointer on the Buffer to fill  
*                  - Address: First page address
*                  - NumPageToRead: Number of page to read
* Output         : None
* Return         : New status of the NAND operation. This parameter can be:
*                   - NAND_TIMEOUT_ERROR: when the previous operation generate 
*                     a Timeout error
*                   - NAND_READY: when memory is ready for the next operation 
*                  And the new status of the increment address operation. It can be:
*                  - NAND_VALID_ADDRESS: When the new address is valid address
*                  - NAND_INVALID_ADDRESS: When the new address is invalid address
* Attention		 : None
*******************************************************************************/
uint32_t FSMC_NAND_ReadPage(uint8_t ChipIndex, NAND_ADDRESS Address,uint8_t *pBuffer, uint32_t NumPageToRead,uint32_t *pECC)
{
  uint32_t index = 0x00, numpageread = 0x00, addressstatus = NAND_VALID_ADDRESS;
  uint32_t status = NAND_READY, size = 0x00;
//  uint32_t delay;
  while((NumPageToRead != 0x0) && (addressstatus == NAND_VALID_ADDRESS))
  {
    /* Page Read command and page address */
    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | CMD_AREA) = NAND_CMD_READ_1; 
   
    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = 0x00; 
    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = 0X00; 
    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = ADDR_1st_CYCLE(NAND_GetRowAddr(ChipIndex,Address));  
    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = ADDR_2nd_CYCLE(NAND_GetRowAddr(ChipIndex,Address));  
    if((NAND_MemInfo[ChipIndex].PagePerBlock*NAND_MemInfo[ChipIndex].BlockPerPlane)&0xFF0000){
      *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = ADDR_3rd_CYCLE(NAND_GetRowAddr(ChipIndex,Address));
    }else if((NAND_MemInfo[ChipIndex].PagePerBlock*NAND_MemInfo[ChipIndex].BlockPerPlane)&0xFF000000){
      *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = ADDR_4th_CYCLE(NAND_GetRowAddr(ChipIndex,Address));
    }
    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | CMD_AREA) = NAND_CMD_READ_TRUE; 
    delay_us(1);//延时
    /* 读忙脚 */
    while( GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6) == 0 );
    
    /* Calculate the size */
    size = NAND_MemInfo[ChipIndex].BytesPerPage + (NAND_MemInfo[ChipIndex].BytesPerPage * numpageread);
    if(ChipIndex == 0){
      //使能ECC
      FSMC_NANDECCCmd(FSMC_Bank2_NAND, ENABLE);
    }else{
      //使能ECC
      FSMC_NANDECCCmd(FSMC_Bank3_NAND, ENABLE);
    }
    /* Get Data into Buffer */    
    for(; index < size; index++)
    {
      pBuffer[index]= *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | DATA_AREA);
    }
    if(ChipIndex == 0){
      while ( FSMC_GetFlagStatus (FSMC_Bank2_NAND , FSMC_FLAG_FEMPT)  == RESET  );///  等待标志位
      *pECC++ = FSMC_GetECC(FSMC_Bank2_NAND);
      //复位ECC
      FSMC_NANDECCCmd(FSMC_Bank2_NAND, DISABLE);
    }else{
      while ( FSMC_GetFlagStatus (FSMC_Bank3_NAND , FSMC_FLAG_FEMPT)  == RESET  );///  等待标志位
      *pECC++ = FSMC_GetECC(FSMC_Bank3_NAND); 
      //复位ECC
      FSMC_NANDECCCmd(FSMC_Bank3_NAND, DISABLE);
    }
    //printf("\r\nRead ECC: 0x%X \r\n", *pECC);
    numpageread++;
    NumPageToRead--;

    /* Calculate page address */
    addressstatus = FSMC_NAND_AddressIncrement(ChipIndex,&Address);
  }

  status = FSMC_NAND_GetStatus(ChipIndex);
  
  return (status | addressstatus);
}

/******************************************************************************
* Function Name  : FSMC_NAND_WriteSpareArea
* Description    : This routine write the spare area information for the specified 
*                  pages addresses.
* Input          : - pBuffer: pointer on the Buffer containing data to be written 
*                  - Address: First page address
*                  - NumSpareAreaTowrite: Number of Spare Area to write
* Output         : None
* Return         : New status of the NAND operation. This parameter can be:
*                   - NAND_TIMEOUT_ERROR: when the previous operation generate 
*                     a Timeout error
*                   - NAND_READY: when memory is ready for the next operation 
*                  And the new status of the increment address operation. It can be:
*                  - NAND_VALID_ADDRESS: When the new address is valid address
*                  - NAND_INVALID_ADDRESS: When the new address is invalid address
* Attention		 : None
*******************************************************************************/
uint32_t FSMC_NAND_WriteSpareArea(uint8_t ChipIndex, NAND_ADDRESS Address,uint8_t *pBuffer, uint32_t NumSpareAreaTowrite)
{
  uint32_t index = 0x00, numsparesreawritten = 0x00, addressstatus = NAND_VALID_ADDRESS;
  uint32_t status = NAND_READY, size = 0x00; 

  while((NumSpareAreaTowrite != 0x00) && (addressstatus == NAND_VALID_ADDRESS) && (status == NAND_READY))
  {
    /* Page write Spare area command and address */
    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | CMD_AREA) = NAND_CMD_PAGEPROGRAM;

    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = 0x00; 
    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = 0x08; 
    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = ADDR_1st_CYCLE(NAND_GetRowAddr(ChipIndex,Address));  
    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = ADDR_2nd_CYCLE(NAND_GetRowAddr(ChipIndex,Address));   
    if((NAND_MemInfo[ChipIndex].PagePerBlock*NAND_MemInfo[ChipIndex].BlockPerPlane)&0xFF0000){
      *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = ADDR_3rd_CYCLE(NAND_GetRowAddr(ChipIndex,Address));
    }else if((NAND_MemInfo[ChipIndex].PagePerBlock*NAND_MemInfo[ChipIndex].BlockPerPlane)&0xFF000000){
      *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = ADDR_4th_CYCLE(NAND_GetRowAddr(ChipIndex,Address));
    }
    /* Calculate the size */ 
    size = NAND_MemInfo[ChipIndex].SpareAreaSize + (NAND_MemInfo[ChipIndex].SpareAreaSize * numsparesreawritten);

    /* Write the data */ 
    for(; index < size; index++)
    {
      *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | DATA_AREA) = pBuffer[index];
    }
    
    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | CMD_AREA) = NAND_CMD_PAGEPROGRAM_TRUE;

    /* 读忙脚 */
    while( GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6) == 0 );

    /* Check status for successful operation */
    status = FSMC_NAND_GetStatus(ChipIndex);

    if(status == NAND_READY)
    {
      numsparesreawritten++;      

      NumSpareAreaTowrite--;  
    
      /* Calculate Next page Address */
      addressstatus = FSMC_NAND_AddressIncrement(ChipIndex,&Address);
    }       
  }
  
  return (status | addressstatus);
}

/******************************************************************************
* Function Name  : FSMC_NAND_ReadSpareArea
* Description    : This routine read the spare area information from the specified
*                  pages addresses.
* Input          : - pBuffer: pointer on the Buffer to fill  
*                  - Address: First page address
*                  - NumSpareAreaToRead: Number of Spare Area to read
* Output         : None
* Return         : New status of the NAND operation. This parameter can be:
*                   - NAND_TIMEOUT_ERROR: when the previous operation generate 
*                     a Timeout error
*                   - NAND_READY: when memory is ready for the next operation 
*                  And the new status of the increment address operation. It can be:
*                  - NAND_VALID_ADDRESS: When the new address is valid address
*                  - NAND_INVALID_ADDRESS: When the new address is invalid address
* Attention		 : None
*******************************************************************************/
uint32_t FSMC_NAND_ReadSpareArea(uint8_t ChipIndex, NAND_ADDRESS Address,uint8_t *pBuffer, uint32_t NumSpareAreaToRead)
{
  uint32_t numsparearearead = 0x00, index = 0x00, addressstatus = NAND_VALID_ADDRESS;
  uint32_t status = NAND_READY, size = 0x00,delay = 0;

  while((NumSpareAreaToRead != 0x0) && (addressstatus == NAND_VALID_ADDRESS))
  {     
    /* Page Read command and page address */     
    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | CMD_AREA) = NAND_CMD_READ_1;

    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = 0x00; 
    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = 0x08;     
    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = ADDR_1st_CYCLE(NAND_GetRowAddr(ChipIndex,Address));  
    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = ADDR_2nd_CYCLE(NAND_GetRowAddr(ChipIndex,Address));    
    if((NAND_MemInfo[ChipIndex].PagePerBlock*NAND_MemInfo[ChipIndex].BlockPerPlane)&0xFF0000){
      *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = ADDR_3rd_CYCLE(NAND_GetRowAddr(ChipIndex,Address));
    }else if((NAND_MemInfo[ChipIndex].PagePerBlock*NAND_MemInfo[ChipIndex].BlockPerPlane)&0xFF000000){
      *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = ADDR_4th_CYCLE(NAND_GetRowAddr(ChipIndex,Address));
    }
    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | CMD_AREA) = NAND_CMD_READ_TRUE;

    for(delay=0; delay < 0x500; delay++);//延时
    /* 读忙脚 */
    while( GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6) == 0 );
    
    /* Data Read */
    size = NAND_MemInfo[ChipIndex].SpareAreaSize +  (NAND_MemInfo[ChipIndex].SpareAreaSize * numsparearearead);
	
    /* Get Data into Buffer */
    for ( ;index < size; index++)
    {
      pBuffer[index] = *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | DATA_AREA);
    }
    
    numsparearearead++;
    
    NumSpareAreaToRead--;

    /* Calculate page address */
    addressstatus = FSMC_NAND_AddressIncrement(ChipIndex,&Address);
  }

  status = FSMC_NAND_GetStatus(ChipIndex);

  return (status | addressstatus);
}

/******************************************************************************
* Function Name  : FSMC_NAND_ReadSpareArea
* Description    : This routine read the spare area information from the specified
*                  pages addresses.
* Input          : - pBuffer: pointer on the Buffer to fill  
*                  - Address: First page address
*                  - NumSpareAreaToRead: Number of Spare Area to read
* Output         : None
* Return         : New status of the NAND operation. This parameter can be:
*                   - NAND_TIMEOUT_ERROR: when the previous operation generate 
*                     a Timeout error
*                   - NAND_READY: when memory is ready for the next operation 
*                  And the new status of the increment address operation. It can be:
*                  - NAND_VALID_ADDRESS: When the new address is valid address
*                  - NAND_INVALID_ADDRESS: When the new address is invalid address
* Attention		 : None
*******************************************************************************/
uint32_t FSMC_NAND_GetBadBlock(uint8_t ChipIndex,uint8_t *pBuffer, NAND_ADDRESS Address, uint32_t NumBlockToGet)
{
//  uint8_t *pSpareAreaBuffer = malloc(NAND_MemInfo[ChipIndex].SpareAreaSize*2);
//  uint32_t Index = 0x00, AddressStatus = NAND_VALID_ADDRESS;
//  Address.Page = 0;
//  while((NumBlockToGet != 0x0)&&(AddressStatus | NAND_VALID_ADDRESS)){
//    AddressStatus = FSMC_NAND_ReadSpareArea(pSpareAreaBuffer,Address,2);
//    //printf("AddressStatus = %X\n\r",AddressStatus);
//    if((pSpareAreaBuffer[0]==0xFF)&&(pSpareAreaBuffer[NAND_MemInfo[ChipIndex].SpareAreaSize]==0xFF)){
//      pBuffer[Index++] = 0;
//    }else{
//      pBuffer[Index++] = 1;
//    }
//    Address.Block++;
//    Address.Page = 0;
//    NumBlockToGet--;
//  }
//  return 0;
}

/******************************************************************************
* Function Name  : FSMC_NAND_ReadSpareArea
* Description    : This routine read the spare area information from the specified
*                  pages addresses.
* Input          : - pBuffer: pointer on the Buffer to fill  
*                  - Address: First page address
*                  - NumSpareAreaToRead: Number of Spare Area to read
* Output         : None
* Return         : New status of the NAND operation. This parameter can be:
*                   - NAND_TIMEOUT_ERROR: when the previous operation generate 
*                     a Timeout error
*                   - NAND_READY: when memory is ready for the next operation 
*                  And the new status of the increment address operation. It can be:
*                  - NAND_VALID_ADDRESS: When the new address is valid address
*                  - NAND_INVALID_ADDRESS: When the new address is invalid address
* Attention		 : None
*******************************************************************************/
uint32_t FSMC_NAND_CheckBadBlock(uint8_t *pBuffer, NAND_ADDRESS Address, uint32_t NumBlockToCheck)
{
//  static uint8_t TxBuffer [NAND_PAGE_SIZE]={0xAA};
//  static uint8_t RxBuffer [NAND_PAGE_SIZE];
//  uint8_t SpareAreaBuffer[NAND_SPARE_AREA_SIZE*2]={0};
//  uint32_t PageIndex = 0x00, BlockIndex = 0x00,AddressStatus = NAND_VALID_ADDRESS;
//  uint32_t WriteEcc,ReadEcc;
//  uint32_t Status;
//  Address.Page = 0;
//  while(NumBlockToCheck != 0x0){
//    FSMC_NAND_ReadSpareArea(SpareAreaBuffer,Address,2);
//    //printf("AddressStatus = %X\n\r",AddressStatus);
//    if((SpareAreaBuffer[0]==0xFF)&&(SpareAreaBuffer[NAND_SPARE_AREA_SIZE]==0xFF)){
//      /* Erase the NAND first Block */
//      Status = FSMC_NAND_EraseBlock(Address);
//      if(Status&NAND_ERROR){
//        printf("FSMC_NAND_EraseBlock = %X\n\r",Status);
//        SpareAreaBuffer[0] = 0;
//        SpareAreaBuffer[NAND_SPARE_AREA_SIZE] = 0;
//        FSMC_NAND_WriteSpareArea(SpareAreaBuffer,Address,2);
//        pBuffer[BlockIndex] = 1;
//      }else{
//        for(PageIndex=0;PageIndex<NAND_BLOCK_SIZE;PageIndex++){
//          Status = FSMC_NAND_WritePage(TxBuffer, Address, 1,&WriteEcc);
//          if(Status&NAND_ERROR){
//            printf("FSMC_NAND_WritePage = %X\n\r",Status);
//            SpareAreaBuffer[0] = 0;
//            SpareAreaBuffer[NAND_SPARE_AREA_SIZE] = 0;
//            FSMC_NAND_WriteSpareArea(SpareAreaBuffer,Address,2);
//            pBuffer[BlockIndex] = 1;
//            break;
//          }else{
//            FSMC_NAND_ReadPage(RxBuffer, Address, 1,&ReadEcc);
//            if(WriteEcc != ReadEcc){
//              SpareAreaBuffer[0] = 0;
//              SpareAreaBuffer[NAND_SPARE_AREA_SIZE] = 0;
//              FSMC_NAND_WriteSpareArea(SpareAreaBuffer,Address,2);
//              pBuffer[BlockIndex] = 1;
//              break;
//            }
//          }
//        }
//        if(PageIndex == NAND_BLOCK_SIZE){
//          pBuffer[BlockIndex] = 0;
//        }
//      }
//    }else{
//      pBuffer[BlockIndex] = 1;
//    }
//    if((((Address.Block+1)%32)==0)){
//      printf("\n\r");
//    }else{
//      if(pBuffer[BlockIndex]){
//        printf("1 ");
//      }else{
//        printf("* ");
//      }
//    }
//    BlockIndex++;
//    Address.Block++;
//    Address.Page = 0;
//    NumBlockToCheck--;
//  }
//  return 0;
}

/******************************************************************************
* Function Name  : FSMC_NAND_EraseBlock
* Description    : This routine erase complete block from NAND FLASH
* Input          : - Address: Any address into block to be erased
* Output         : None
* Return         : New status of the NAND operation. This parameter can be:
*                   - NAND_TIMEOUT_ERROR: when the previous operation generate 
*                     a Timeout error
*                   - NAND_READY: when memory is ready for the next operation 
* Attention		 : None
*******************************************************************************/
uint32_t FSMC_NAND_EraseBlock(uint8_t ChipIndex,NAND_ADDRESS Address,uint32_t NumBlockToErase)
{
  uint32_t Status = NAND_READY;
  while((NumBlockToErase--)&&(Status == NAND_READY)){
    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | CMD_AREA) = NAND_CMD_ERASE0;
    
    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = ADDR_1st_CYCLE(NAND_GetRowAddr(ChipIndex,Address));  
    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = ADDR_2nd_CYCLE(NAND_GetRowAddr(ChipIndex,Address));  
    if((NAND_MemInfo[ChipIndex].PagePerBlock*NAND_MemInfo[ChipIndex].BlockPerPlane)&0xFF0000){
      *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = ADDR_3rd_CYCLE(NAND_GetRowAddr(ChipIndex,Address));
    }else if((NAND_MemInfo[ChipIndex].PagePerBlock*NAND_MemInfo[ChipIndex].BlockPerPlane)&0xFF000000){
      *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | ADDR_AREA) = ADDR_4th_CYCLE(NAND_GetRowAddr(ChipIndex,Address));
    }
    *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | CMD_AREA) = NAND_CMD_ERASE1;
    delay_us(1);
    /* 读忙脚 */
    while( GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6) == 0 );
    Status = FSMC_NAND_GetStatus(ChipIndex);
    if(Status != NAND_READY){
      return Status;
    }else{
      Address.Block++;
      if(Address.Block == NAND_MemInfo[ChipIndex].BlockPerPlane)
      {
        Address.Block = 0;
        Address.Plane++;

        if(Address.Plane == NAND_MemInfo[ChipIndex].PlanePerChip)
        {
          Status = NAND_INVALID_ADDRESS;
        }
      }
    }
  }
  
  //while( GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6) == 0 );
  
  return Status;
}

/******************************************************************************
* Function Name  : FSMC_NAND_Reset
* Description    : This routine reset the NAND FLASH
* Input          : None
* Output         : None
* Return         : NAND_READY
* Attention		 : None
*******************************************************************************/
uint32_t FSMC_NAND_Reset(uint8_t ChipIndex)
{
  *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | CMD_AREA) = NAND_CMD_RESET;

  return (NAND_READY);
}

/******************************************************************************
* Function Name  : FSMC_NAND_GetStatus
* Description    : Get the NAND operation status
* Input          : None
* Output         : None
* Return         : New status of the NAND operation. This parameter can be:
*                   - NAND_TIMEOUT_ERROR: when the previous operation generate 
*                     a Timeout error
*                   - NAND_READY: when memory is ready for the next operation  
* Attention		 : None  
*******************************************************************************/
uint32_t FSMC_NAND_GetStatus(uint8_t ChipIndex)
{
  uint32_t timeout = 0x1000000, status = NAND_READY;

  status = FSMC_NAND_ReadStatus(ChipIndex); 

  /* Wait for a NAND operation to complete or a TIMEOUT to occur */
  while ((status != NAND_READY) &&( timeout != 0x00))
  {
    if(status == NAND_ERROR){
      return NAND_ERROR;
    }
     status = FSMC_NAND_ReadStatus(ChipIndex);
     timeout --;      
  }

  if(timeout == 0x00)
  {          
    status =  NAND_TIMEOUT_ERROR;      
  } 

  /* Return the operation status */
  return (status);      
}
/******************************************************************************
* Function Name  : FSMC_NAND_ReadStatus
* Description    : Reads the NAND memory status using the Read status command 
* Input          : None
* Output         : None
* Return         : The status of the NAND memory. This parameter can be:
*                   - NAND_BUSY: when memory is busy
*                   - NAND_READY: when memory is ready for the next operation    
*                   - NAND_ERROR: when the previous operation gererates error  
* Attention		 : None 
*******************************************************************************/
uint32_t FSMC_NAND_ReadStatus(uint8_t ChipIndex)
{
  uint32_t data = 0x00, status = NAND_BUSY;

  /* Read status operation ------------------------------------ */
  *(vu8 *)(NAND_FlashStartAddr[ChipIndex] | CMD_AREA) = NAND_CMD_STATUS;
  data = *(vu8 *)(NAND_FlashStartAddr[ChipIndex]);

  if((data & NAND_ERROR) == NAND_ERROR)
  {
    status = NAND_ERROR;
  } 
  else if((data & NAND_READY) == NAND_READY)
  {
    status = NAND_READY;
  }
  else
  {
    status = NAND_BUSY; 
  }
  
  return (status);
}

/******************************************************************************
* Function Name  : NAND_AddressIncrement
* Description    : Increment the NAND memory address
* Input          : - Address: address to be incremented.
* Output         : None
* Return         : The new status of the increment address operation. It can be:
*                  - NAND_VALID_ADDRESS: When the new address is valid address
*                  - NAND_INVALID_ADDRESS: When the new address is invalid address
* Attention		 : None
*******************************************************************************/
uint32_t FSMC_NAND_AddressIncrement(uint8_t ChipIndex,NAND_ADDRESS* Address)
{
  uint32_t status = NAND_VALID_ADDRESS;
 
  Address->Page++;

  if(Address->Page == NAND_MemInfo[ChipIndex].PagePerBlock)
  {
    Address->Page = 0;
    Address->Block++;
    
    if(Address->Block == NAND_MemInfo[ChipIndex].BlockPerPlane)
    {
      Address->Block = 0;
      Address->Plane++;

      if(Address->Plane == NAND_MemInfo[ChipIndex].PlanePerChip)
      {
        status = NAND_INVALID_ADDRESS;
      }
    }
  } 
  
  return (status);
}


/******************************************************************************
* Function Name  : FSMC_NAND_Test
* Description    : NAND test
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void FSMC_NAND_Test(void)
{
  uint16_t index;
  uint32_t TxSum=0,RxSum=0,Status;
  uint32_t TxEcc,RxEcc;
  static uint8_t BadBlockFlag[1024];
  static uint8_t TxBuffer[2048];
  static uint8_t RxBuffer[2048];
  static NAND_ID NandId;
  NAND_MEMORY_INFO NandMemInfo;
  NAND_ADDRESS WriteReadAddr;

  //FSMC_NANDDeInit(FSMC_Bank2_NAND);
  FSMC_NAND_Init(0,2048,10);

  /* 检测NAND Flash */
  FSMC_NAND_ReadID(0,&NandId);
  printf("Nand Flash ID = %02X,%02X,%02X,%02X,%02X\n\r",NandId.MakerID, NandId.DeviceID, NandId.ThirdID, NandId.FourthID,NandId.FiveID);
  if ((NandId.MakerID == 0xEC) && (NandId.DeviceID == 0xF1) && (NandId.ThirdID == 0x80) && (NandId.FourthID == 0x15)){
    printf("Type = K9F1G08U0A\r\n");
  }else if ((NandId.MakerID == 0xEC) && (NandId.DeviceID == 0xF1) && (NandId.ThirdID == 0x00) && (NandId.FourthID == 0x95)){
    printf("Type = K9F1G08U0B\r\n");
  }else if ((NandId.MakerID == 0xAD) && (NandId.DeviceID == 0xF1) && (NandId.ThirdID == 0x80) && (NandId.FourthID == 0x1D)){
    printf("Type = HY27UF081G2A\r\n");
  }else{
    printf("Type = Unknow\r\n");
    return;
  }

  //获取存储信息
  FSMC_NAND_GetMemoryInfo(&NandId,&NandMemInfo);
  printf("NandMemInfo.PlanePerChip = %d\n\r",NandMemInfo.PlanePerChip);
  printf("NandMemInfo.BlockPerPlane = %d\n\r",NandMemInfo.BlockPerPlane);
  printf("NandMemInfo.PagePerBlock = %d\n\r",NandMemInfo.PagePerBlock);
  printf("NandMemInfo.BytesPerPage = %d\n\r",NandMemInfo.BytesPerPage);
  printf("NandMemInfo.SpareAreaSize = %d\n\r",NandMemInfo.SpareAreaSize);
  //重新初始化NAND
  FSMC_NANDDeInit(FSMC_Bank2_NAND);
  FSMC_NAND_Init(0,NandMemInfo.BytesPerPage,20);
  NAND_MemInfo[0] = NandMemInfo;
  /* NAND memory address to write to */
  WriteReadAddr.Plane = 0x00;
  WriteReadAddr.Block = 0x00;
  WriteReadAddr.Page = 0x00;


  /* Erase the NAND first Block */
  Status = FSMC_NAND_EraseBlock(0,WriteReadAddr,1);
  //printf("Erase Block Status = %08X\n\r",Status);
  /* Write data to FSMC NOR memory */
  /* Fill the buffer to send */
  srand(1);
  for (index = 0; index < NandMemInfo.BytesPerPage; index++ ){
    TxBuffer[index] = 1;
    TxSum += TxBuffer[index];
  }
  TxBuffer[0] = 0x5;
  Status = FSMC_NAND_WritePage(0,WriteReadAddr,TxBuffer,1,&TxEcc);
  //printf("Write Page Status = %08X\n\r",Status);
  printf("TxSum = %08X\n\r",TxSum);
  printf("TX ECC = %08X\n\r",TxEcc);
  
//  printf("\r\nWritten to the number of:\r\n");
//  for(j = 0; j < 128; j++){
//    printf("%02X ",TxBuffer[j]);
//    if((((j+1)%32)==0)){
//      printf("\n\r");
//    }
//  }

  /* Read back the written data */
  Status = FSMC_NAND_ReadPage (0,WriteReadAddr,RxBuffer,1,&RxEcc);
  //printf("Read Page Status = %08X\n\r",Status);
  for (index = 0; index < NandMemInfo.BytesPerPage; index++ ){
    RxSum += RxBuffer[index];
  }
  printf("RxSum = %08X\n\r",RxSum);
  printf("RX ECC = %08X\n\r",RxEcc);
  printf("TXECC^RXECC = %08X\n\r",TxEcc^RxEcc);

//  printf("\r\nRead several:\r\n");
//  for(j = 0; j < 128; j++){
//    printf("%02X ",RxBuffer[j]);
//    if((((j+1)%32)==0)){
//      printf("\n\r");
//    }
//  }

}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
