#include "stm32f4xx_hal.h"
#include "fatfs.h"
#include "string.h"
#include "user_def.h"
#include "mem_if.h"

//extern TCHAR SD_Path[4]; // SD card logical drive path 
extern FRESULT res;
extern FILINFO fno;
extern DIR dir;
extern uint32_t byteswritten, bytesread;	//File write/read counts

extern uint8_t retSD; /* Return value for SD */
extern char SDPath[4]; /* SD logical drive path */
extern FATFS SDFatFS; /* File system object for SD logical drive */
extern FIL SDFile; /* File object for SD */

HEX_L_T hex_line_dat;

void f_opendir_scan(void)
{
		TCHAR path[200] = "";
	
    res = f_mount(&SDFatFS,(TCHAR const*)SDPath,1);
    printf("SD Mount : res f_mount : %02X\n\r",res);
    
    if (res == FR_OK)
    {
			res = f_opendir(&dir,path);
			printf("res f_open : %02X\n\r",res);
			
			if (res == FR_OK)
			{
				while(1)
				{
					TCHAR *fn;
					 
					res = f_readdir(&dir, &fno);
					 
					if (res != FR_OK)
							printf("res = %d f_readdir\n\r", res);

					if ((res != FR_OK) || (fno.fname[0] == 0))
							break;

					fn = fno.fname;
					printf("%c%c%c%c ",
							((fno.fattrib & AM_DIR) ? 'D' : '-'),
							((fno.fattrib & AM_RDO) ? 'R' : '-'),
							((fno.fattrib & AM_SYS) ? 'S' : '-'),
							((fno.fattrib & AM_HID) ? 'H' : '-') );

					printf("%10d ", fno.fsize);
					printf("%s/%s\n\r", path, fn);
				}        
				f_closedir(&dir);
		}
	
		res = f_mount(0,SDPath,0);
		printf("SD Unmount : res f_mount : %02X\n\r",res);
	}
}

uint8_t Is_patterned_file(FIL *fp,TCHAR *f_pattern)
{
	uint16_t cnt_file=0,i;
	
	//res = f_findfirst(&dir,&fno,"","740*.jpg");
	//res = f_findfirst(&dir,&fno,"","*.hex");
	res = f_findfirst(&dir,&fno,"",f_pattern);
	while (res == FR_OK && fno.fname[0]) 
	{         /* Repeat while an item is found */
		cnt_file++;
		printf("%s\r\n", fno.fname);                /* Display the object name */
		res = f_findnext(&dir, &fno);               /* Search for next item */
	}
	f_closedir(&dir);
	return cnt_file;
}

void delete_hex_file(FIL *fp,TCHAR *f_pattern)
{
	uint16_t cnt_file=0,i;
	
	//res = f_findfirst(&dir,&fno,"","740*.jpg");
	//res = f_findfirst(&dir,&fno,"","*.hex");
	res = f_findfirst(&dir,&fno,"",f_pattern);
	while (res == FR_OK && fno.fname[0]) 
	{         /* Repeat while an item is found */
		//dir_remove(&dir);
		f_unlink(fno.fname);
		cnt_file++;
		printf("%s\r\n", fno.fname);                /* Display the object name */
		res = f_findnext(&dir, &fno);               /* Search for next item */
	}
	f_closedir(&dir);
}
void Write_hexToFlash(FIL *fp,TCHAR *hFname)
{
	char read_buf[80];
	uint8_t ret_lfunc=0;	// 0:OK , 1:Error excepted FR_OK , 2:EOF , 3:CRC Error
	uint16_t cnt_line=0,i;
	uint8_t *p_upper_address,*p_dest_address,*p_src_address;

	f_open(fp,hFname,FA_READ);
	while(1)
	{
		ret_lfunc = read_hex_line(fp,read_buf,&bytesread,&hex_line_dat);
		if (ret_lfunc == 0)
		{
			#ifdef BOOT_DEBUG
			printf("%02d.[rt:%d,cnt:%2d] :%02X:%04X:%02X:",
			cnt_line++,ret_lfunc,bytesread,hex_line_dat.length,hex_line_dat.Addr,hex_line_dat.Type);
			for(i=0;i<hex_line_dat.length;i++)
			{
				printf("%02X:",hex_line_dat.Dat[i]);
			}
			printf("%02X\r\n",hex_line_dat.CRC_d);
			#endif
			
			switch (hex_line_dat.Type)
			{
				case 0:		// data
					p_dest_address = p_upper_address+(uint32_t)hex_line_dat.Addr;
					#ifdef BOOT_DEBUG
					printf("d_adr[%08X]:",(uint32_t)p_dest_address);
					#endif
					p_src_address = (uint8_t *)&hex_line_dat.Dat[0];
					MEM_If_Write_FS(p_src_address,p_dest_address,hex_line_dat.length);
					#ifdef BOOT_DEBUG
					for(i=0;i<hex_line_dat.length;i++)
					{
						printf("%02X:",*p_dest_address);p_dest_address++;
					}
					printf("\r\n");
					#endif
					//while(1);
					break;
				case 1:		// an end-of-file record
					break;
				case 2:		// ..
					break;
				case 3:		// ..
					break;
				case 4:		// an extended linear address record
					p_upper_address = (uint8_t *)(((uint32_t)hex_line_dat.Dat[0])*0x1000000
													             +((uint32_t)hex_line_dat.Dat[1])*0x10000);
					#ifdef BOOT_DEBUG
					printf("p_dest_address = %08X\r\n",(uint32_t)p_upper_address);
					#endif
					break;
				case 5:		// a start linear address record
					break;
			}
		}
		else
		{
			#ifdef BOOT_DEBUG
			printf("End Of File\r\n");
			#endif
			break;
		}
	}
	f_close(fp);
}

uint8_t read_hex_line(FIL *fp,TCHAR *buff,UINT *br,HEX_L_T *hexL)
{
	uint16_t i,cnt_line_length;
	FSIZE_t tm_fptr;
	
	tm_fptr = fp->fptr;	// prev File point
	
	memset(buff,0,HEX_RDLINE_MAX+10);
	res=f_read(fp,buff,3,br);
	if (res == FR_OK)
	{
		if (*br == 3)
		{
			if (((TCHAR *)buff)[0] == ':')
			{
				hexL->length = (uint16_t)atoh(((uint8_t)buff[1]))*0x10 
									   + (uint16_t)atoh(((uint8_t)buff[2]));
				
				// 4:address + 2:Type + 2:CRC + 2:'\r'+'\n'
				cnt_line_length = 2*(uint16_t)hexL->length + 10; 
				res=f_read(fp,&buff[3],cnt_line_length,br);
				if (res == FR_OK)
				{
					hexL->Addr = (uint16_t)atoh(((uint8_t)buff[3]))*0x1000;
					hexL->Addr += (uint16_t)atoh(((uint8_t)buff[4]))*0x0100;
					hexL->Addr += (uint16_t)atoh(((uint8_t)buff[5]))*0x0010;
					hexL->Addr += (uint16_t)atoh(((uint8_t)buff[6]))*0x0001;
					
					hexL->Type = (uint16_t)atoh(((uint8_t)buff[7]))*0x0010 + 
											 (uint16_t)atoh(((uint8_t)buff[8]));
					hexL->CRC_d = hexL->length;
					//printf("1.prep CRC = %02X\r\n",hexL->CRC_d);
					hexL->CRC_d += (uint8_t)((hexL->Addr >> 8)&0x00FF);
					//printf("2.prep CRC = %02X\r\n",hexL->CRC_d);
					hexL->CRC_d += (uint8_t)((hexL->Addr >> 0)&0x00FF);
					//printf("3.prep CRC = %02X\r\n",hexL->CRC_d);
					hexL->CRC_d += hexL->Type;
					//printf("4.prep CRC = %02X\r\n",hexL->CRC_d);
					
					for (i=0;i<hexL->length;i++)
					{
						hexL->Dat[i] = ( (uint16_t)atoh(((uint8_t)buff[9+i*2]))*0x0010 
												   + (uint16_t)atoh(((uint8_t)buff[9+i*2+1])) );
						hexL->CRC_d +=  hexL->Dat[i];
						//printf("%02d.prep CRC = %02X\r\n",i+5,hexL->CRC_d);
					}
					//printf("last.prep CRC = %02X,%02X\r\n",hexL->CRC_d,(~(hexL->CRC_d)+1));
					hexL->CRC_d = ~(hexL->CRC_d)+1;
					//*br = *br+3;
					hexL->Dat[i] = ( (uint16_t)atoh(((uint8_t)buff[9+i*2]))*0x0010 + 
												   (uint16_t)atoh(((uint8_t)buff[9+i*2+1])) );
					if (hexL->CRC_d == hexL->Dat[hexL->length])
					{
						return 0;
					}
					else
					{
						return 3;	// Error : CRC not matching
					}
				}
				else
				{
					return 1;	// Error: File is strange
				}
			}
		}
		else	// EOF
		{
			if (*br)
			{
				return 1;	// Error: File is strange
			}
			else
			{
				return 2; //End of File
			}
		}
	}
}

uint8_t atod(uint8_t in_ascii)
{
    uint8_t rtn_val;
    
    if ( (in_ascii >= '0') || (in_ascii <= '9') )
    {
        rtn_val = in_ascii - '0';
    }
    else
    {
        rtn_val = 0xFF;
    }
    
    return rtn_val;
}

uint8_t atoh(uint8_t in_ascii)
{
    uint8_t rtn_val;
    
    if ( (in_ascii >= '0') && (in_ascii <= '9') )
    {
        rtn_val = in_ascii - '0';
    }
    else if ( (in_ascii >= 'a') && (in_ascii <= 'f') )
    {
        rtn_val = in_ascii - 'a' + 0x0A;
    }
    else if ( (in_ascii >= 'A') && (in_ascii <= 'F') )
    {
        rtn_val = in_ascii - 'A' + 0x0A;
    }
    else
    {
        rtn_val = 0xFF;
    }
    
    return rtn_val;
}
