#include "stm32f4xx_hal_def.h"

// 512KB
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */

#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_4   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     FLASH_USER_START_ADDR  +  GetSectorSize(FLASH_SECTOR_4) -1 /* End @ of user Flash area : sector start address + sector size -1 */

#define USR_MEM_CNT			512
#define USR_ASIGN_ID		0xA5
#define USR_ASIGN_Add		0
#define USR_DATA0_Add		1


typedef enum {
  FLASH_OK   = 0,
  FLASH_BUSY,
  FLASH_FAIL,
}FLASH_StatusTypeDef;


uint16_t MEM_If_Init_FS(void);
uint16_t MEM_If_Erase_FS (uint32_t Add);
uint16_t MEM_If_Write_FS (uint8_t *src, uint8_t *dest, uint32_t Len);
uint8_t *MEM_If_Read_FS  (uint8_t *src, uint8_t *dest, uint32_t Len);
uint16_t MEM_If_DeInit_FS(void);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
uint32_t GetSectorSize(uint32_t Sector);
uint32_t GetSector(uint32_t Address);
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */
