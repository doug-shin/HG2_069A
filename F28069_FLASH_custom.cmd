/*
// Custom F28069 FLASH Linker Command File
// Based on C2000Ware F28069.cmd with memory conflict fixes
//###########################################################################
*/

MEMORY
{
PAGE 0 :   /* Program Memory */
   RAML0       : origin = 0x008000, length = 0x000800     /* on-chip RAM block L0 */
   RAML1       : origin = 0x008800, length = 0x000400     /* on-chip RAM block L1 */
   RAML3_CLA   : origin = 0x009000, length = 0x000400     /* CLA 프로그램 실행 영역 */
   OTP         : origin = 0x3D7800, length = 0x000400     /* on-chip OTP */

   FLASHH      : origin = 0x3D8000, length = 0x004000     /* on-chip FLASH */
   FLASHG      : origin = 0x3DC000, length = 0x004000     /* on-chip FLASH */
   FLASHF      : origin = 0x3E0000, length = 0x004000     /* on-chip FLASH */
   FLASHE      : origin = 0x3E4000, length = 0x004000     /* on-chip FLASH */   
   FLASHD      : origin = 0x3E8000, length = 0x004000     /* on-chip FLASH */
   FLASHC      : origin = 0x3EC000, length = 0x004000     /* on-chip FLASH */
   FLASHB      : origin = 0x3F0000, length = 0x004000     /* on-chip FLASH */
   FLASHA      : origin = 0x3F4000, length = 0x003F80     /* on-chip FLASH */
   CSM_RSVD    : origin = 0x3F7F80, length = 0x000076     /* Part of FLASHA.  Program with all 0x0000 when CSM is in use. */
   BEGIN       : origin = 0x3F7FF6, length = 0x000002     /* Part of FLASHA.  Used for "boot to Flash" bootloader mode. */
   CSM_PWL_P0  : origin = 0x3F7FF8, length = 0x000008     /* Part of FLASHA.  CSM password locations in FLASHA */

   FPUTABLES   : origin = 0x3FD860, length = 0x0006A0	  /* FPU Tables in Boot ROM */
   IQTABLES    : origin = 0x3FDF00, length = 0x000B50     /* IQ Math Tables in Boot ROM */
   IQTABLES2   : origin = 0x3FEA50, length = 0x00008C     /* IQ Math Tables in Boot ROM */
   IQTABLES3   : origin = 0x3FEADC, length = 0x0000AA	  /* IQ Math Tables in Boot ROM */

   ROM         : origin = 0x3FF3B0, length = 0x000C10     /* Boot ROM */
   RESET       : origin = 0x3FFFC0, length = 0x000002     /* part of boot ROM  */
   VECTORS     : origin = 0x3FFFC2, length = 0x00003E     /* part of boot ROM  */

PAGE 1 :   /* Data Memory */
   BOOT_RSVD   : origin = 0x000000, length = 0x000050     /* Part of M0, BOOT rom will use this for stack */
   RAMM0       : origin = 0x000050, length = 0x0003B0     /* on-chip RAM block M0 */
   RAMM1       : origin = 0x000400, length = 0x000400     /* on-chip RAM block M1 */
   
   /* CPU 전용 RAM 영역 */
   RAML3       : origin = 0x009C00, length = 0x000400     /* CPU 전용 RAML3 나머지 */
   RAML4       : origin = 0x00A000, length = 0x002000     /* on-chip RAM block L4 */
   RAML5       : origin = 0x00C000, length = 0x002000     /* on-chip RAM block L5 */
   RAML6       : origin = 0x00E000, length = 0x002000     /* on-chip RAM block L6 */
   RAML7       : origin = 0x010000, length = 0x002000     /* on-chip RAM block L7 */
   RAML8       : origin = 0x012000, length = 0x002000     /* on-chip RAM block L8 */
   
   /* CLA 전용 메모리 영역 (기존 RAML1, RAML2 사용) */
   CLARAM0     : origin = 0x008800, length = 0x000400     /* CLA Data RAM 0 (RAML1 재용도) */
   CLARAM1     : origin = 0x008C00, length = 0x000400     /* CLA Data RAM 1 (RAML2 재용도) */
   CLARAM2     : origin = 0x009800, length = 0x000400     /* CLA Data RAM 2 (RAML3 상위 재용도) */
   CLA1_MSGRAMLOW  : origin = 0x001480, length = 0x000080     /* CLA to CPU Message RAM */
   CLA1_MSGRAMHIGH : origin = 0x001500, length = 0x000080     /* CPU to CLA Message RAM */
}

SECTIONS
{
   /* Allocate program areas: */
   .cinit              : > FLASHA,     PAGE = 0
   .pinit              : > FLASHA,     PAGE = 0
   .text               : >> FLASHH | FLASHG | FLASHF | FLASHE | FLASHD | FLASHC | FLASHB | FLASHA,     PAGE = 0
   dclfuncs            : >> FLASHH | FLASHG | FLASHF | FLASHE | FLASHD | FLASHC | FLASHB | FLASHA,     PAGE = 0
   codestart           : > BEGIN,      PAGE = 0
   ramfuncs            : LOAD = FLASHD,
                         RUN = RAML0,
                         LOAD_START(_RamfuncsLoadStart),
                         LOAD_END(_RamfuncsLoadEnd),
                         RUN_START(_RamfuncsRunStart),
						 LOAD_SIZE(_RamfuncsLoadSize),
                         PAGE = 0

   csmpasswds          : > CSM_PWL_P0, PAGE = 0
   csm_rsvd            : > CSM_RSVD,   PAGE = 0

   /* Allocate uninitialized data sections: */
   .stack              : > RAMM0,      PAGE = 1
   .ebss               : > RAML4,      PAGE = 1
   .esysmem            : > RAML3,      PAGE = 1

   /* Initialized sections to go in Flash */
   .econst             : > FLASHA,     PAGE = 0
   .switch             : > FLASHA,     PAGE = 0

   /* Allocate IQ math areas: */
   IQmath              : > FLASHA,     PAGE = 0
   IQmathTables        : > IQTABLES,   PAGE = 0, TYPE = NOLOAD
   
   /* Allocate FPU math areas: */
   FPUmathTables       : > FPUTABLES,  PAGE = 0, TYPE = NOLOAD
   
   /* DMA sections - separate from .cio to avoid conflict */
   DMARAML5	           : > RAML5,      PAGE = 1
   DMARAML6	           : > RAML6,      PAGE = 1
   DMARAML7	           : > RAML7,      PAGE = 1
   DMARAML8	           : > RAML8,      PAGE = 1   

   /* CLA Sections */
   .scratchpad         : > CLARAM0,   PAGE = 1
   .bss_cla	           : > CLARAM0,   PAGE = 1
   .const_cla	        : > CLARAM0,   PAGE = 1

   Cla1Prog            : LOAD = FLASHD,
                         RUN = RAML3_CLA,
                         LOAD_START(_Cla1funcsLoadStart),
                         LOAD_END(_Cla1funcsLoadEnd),
                         LOAD_SIZE(_Cla1funcsLoadSize),
                         RUN_START(_Cla1funcsRunStart),
                         PAGE = 0

   Cla1ToCpuMsgRAM     : > CLA1_MSGRAMLOW,   PAGE = 1
   CpuToCla1MsgRAM     : > CLA1_MSGRAMHIGH,  PAGE = 1
   Cla1DataRam0	       : > CLARAM0,	  PAGE = 1
   Cla1DataRam1	       : > CLARAM1,	  PAGE = 1
   Cla1DataRam2	       : > CLARAM2,	  PAGE = 1

   CLA1mathTables      : > CLARAM1,
                         LOAD_START(_Cla1mathTablesLoadStart),
                         LOAD_END(_Cla1mathTablesLoadEnd),
                         LOAD_SIZE(_Cla1mathTablesLoadSize),
                         RUN_START(_Cla1mathTablesRunStart),
                         PAGE = 1

   /* CLA 스크래치패드 섹션 (128 바이트 고정 할당) */
   CLAscratch          : > CLARAM0, PAGE = 1

   .reset              : > RESET,      PAGE = 0, TYPE = DSECT
   vectors             : > VECTORS,    PAGE = 0, TYPE = DSECT
} 