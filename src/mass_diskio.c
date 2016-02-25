/**
  ******************************************************************************
  * @file    mass_diskio.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Medium Access Layer interface
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "platform_config.h"
#include "mass_diskio.h"
#include "string.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t Mass_Memory_Size[2];
uint32_t Mass_Block_Size[2];
uint32_t Mass_Block_Count[2];
__IO uint32_t Status = 0;

#define BLOCK_SIZE      512

#define BUFF_SIZE       (16*1024)
uint8_t  _mem_disk[BUFF_SIZE]={0};

uint8_t  _mem_nbr[512]={0};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : DISKIO_Init
* Description    : Initializes the Media on the STM32
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t DISKIO_Init(uint8_t lun)
{
	uint16_t status = DISKIO_OK;

	return status;
}

/*******************************************************************************
* Function Name  : DISKIO_Write
* Description    : Write sectors
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t DISKIO_Write(uint8_t lun, uint32_t Memory_Offset, uint32_t * Writebuff,
		   uint16_t Transfer_Length)
{
    if(Memory_Offset == 0)
    {
        memcpy(_mem_nbr, Writebuff ,Transfer_Length);
    }

    if(Memory_Offset > 0xe00)
    {
        Memory_Offset -=0xe00;

        if( (Memory_Offset) < BUFF_SIZE)
        {
            memcpy(&_mem_disk[Memory_Offset], Writebuff, Transfer_Length);
        }
    }

    return DISKIO_OK;
}

/*******************************************************************************
* Function Name  : DISKIO_Read
* Description    : Read sectors
* Input          : None
* Output         : None
* Return         : Buffer pointer
*******************************************************************************/
uint16_t DISKIO_Read(uint8_t lun, uint32_t Memory_Offset, uint32_t * Readbuff,
		  uint16_t Transfer_Length)
{
    if(Memory_Offset == 0)
    {
        memcpy(Readbuff, _mem_nbr, Transfer_Length);
    }

    if(Memory_Offset > 0xe00)
    {
        Memory_Offset -=0xe00;
        if( (Memory_Offset) < BUFF_SIZE)
        {

            memcpy(Readbuff, &_mem_disk[Memory_Offset], Transfer_Length);

            return DISKIO_OK;
        }
    }


    return DISKIO_FAIL;
}

/*******************************************************************************
* Function Name  : DISKIO_GetStatus
* Description    : Get status
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t DISKIO_GetStatus(uint8_t lun)
{

        /* 1MB = 512b*2000*/
		Mass_Block_Count[0] = (BUFF_SIZE + 16*1024)/BLOCK_SIZE;
		Mass_Block_Size[0] = BLOCK_SIZE;
		Mass_Memory_Size[0] = (Mass_Block_Count[0] * Mass_Block_Size[0]);


		return DISKIO_OK;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
