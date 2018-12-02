#define HEX_RDLINE_MAX	50
//#define BOOT_DEBUG

typedef struct {
    uint8_t length;     // ring buffer
    uint16_t Addr;                    // load pointer
    uint8_t Type;                   // ring buffer length
    uint8_t Dat[20];                    // consume pointer
    uint8_t CRC_d;                    // consume pointer
} HEX_L_T;


void f_opendir_scan(void);
uint8_t read_hex_line(FIL *fp,TCHAR *buff,UINT *br,HEX_L_T *hexL);
uint8_t atod(uint8_t in_ascii);
uint8_t atoh(uint8_t in_ascii);
uint8_t Is_patterned_file(FIL *fp,TCHAR *f_pattern);
void Write_hexToFlash(FIL *fp,TCHAR *hFname);
void delete_hex_file(FIL *fp,TCHAR *f_pattern);

