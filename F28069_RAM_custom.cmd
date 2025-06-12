/*
// Custom F28069 RAM Linker Command File
// Optimized for 35kW DCDC project - Fixed memory allocation
//###########################################################################
*/

MEMORY
{
PAGE 0 :   /* Program Memory */
   BEGIN       : origin = 0x000000, length = 0x000002     /* Boot to SARAM entry point */
   RAMM0       : origin = 0x000050, length = 0x0003B0     /* M0 RAM (944 bytes) */
   
   /* Expanded program memory - combine L0-L4 for large .text section */
   RAML0_L4    : origin = 0x008000, length = 0x004000     /* L0+L1+L2+L3+L4 combined (16KB) */
   
   /* Boot ROM tables */
   RESET       : origin = 0x3FFFC0, length = 0x000002
   FPUTABLES   : origin = 0x3FD590, length = 0x0006A0     /* FPU Tables in Boot ROM */
   IQTABLES    : origin = 0x3FDF00, length = 0x000B50     /* IQ Math Tables in Boot ROM */
   IQTABLES2   : origin = 0x3FEA50, length = 0x00008C     /* IQ Math Tables in Boot ROM */
   IQTABLES3   : origin = 0x3FEADC, length = 0x0000AA     /* IQ Math Tables in Boot ROM */
   BOOTROM     : origin = 0x3FF3B0, length = 0x000C10

PAGE 1 :   /* Data Memory */
   BOOT_RSVD   : origin = 0x000002, length = 0x00004E     /* Part of M0, BOOT rom will use this for stack */
   RAMM1       : origin = 0x000400, length = 0x000400     /* M1 RAM (1KB) */
   
   /* Data sections */
   RAML5       : origin = 0x00C000, length = 0x002000     /* L5 RAM (8KB) */
   RAML6       : origin = 0x00E000, length = 0x002000     /* L6 RAM (8KB) */
   RAML7       : origin = 0x010000, length = 0x002000     /* L7 RAM (8KB) */
   RAML8       : origin = 0x012000, length = 0x002000     /* L8 RAM (8KB) */
}

SECTIONS
{
   /* Program sections */
   codestart        : > BEGIN,         PAGE = 0
   .text            : > RAML0_L4,      PAGE = 0    /* Main program code (16KB available) */
   ramfuncs         : > RAML0_L4,      PAGE = 0,   /* RAM functions - same location as .text for RAM build */
                      LOAD_START(_RamfuncsLoadStart),
                      LOAD_END(_RamfuncsLoadEnd),
                      RUN_START(_RamfuncsRunStart),
                      LOAD_SIZE(_RamfuncsLoadSize)
   .cinit           : > RAMM0,         PAGE = 0    /* Initialization data */
   .pinit           : > RAMM0,         PAGE = 0    /* Constructor tables */
   .switch          : > RAMM0,         PAGE = 0    /* Switch tables */
   .reset           : > RESET,         PAGE = 0, TYPE = DSECT

   /* Data sections */
   .stack           : > RAMM1,         PAGE = 1    /* Stack (1KB) */
   .ebss            : > RAML5,         PAGE = 1    /* Uninitialized variables */
   .econst          : > RAML5,         PAGE = 1    /* Initialized constants */
   .esysmem         : > RAML5,         PAGE = 1    /* Dynamic memory allocation */

   /* Math libraries */
   IQmath           : > RAML0_L4,      PAGE = 0    /* IQ Math code */
   IQmathTables     : > IQTABLES,      PAGE = 0, TYPE = NOLOAD
   FPUmathTables    : > FPUTABLES,     PAGE = 0, TYPE = NOLOAD
   
   /* DMA sections - dedicated memory blocks */
   DMARAML6         : > RAML6,         PAGE = 1    /* DMA buffer 1 (8KB) */
   DMARAML7         : > RAML7,         PAGE = 1    /* DMA buffer 2 (8KB) */
   DMARAML8         : > RAML8,         PAGE = 1    /* DMA buffer 3 (8KB) */
} 