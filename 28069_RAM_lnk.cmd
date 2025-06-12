/*
// FILE:    28069_RAM_lnk.cmd
// TITLE:   Linker Command File For F28069 Device - RAM execution
// Optimized for RAM execution without CLA support
//###########################################################################

/* Define the memory block start/length for the F2806x
   PAGE 0 will be used to organize program sections
   PAGE 1 will be used to organize data sections
*/

MEMORY
{
PAGE 0 :
   /* Memory blocks for program allocation - RAM execution */
   RAML0       : origin = 0x008000, length = 0x000800     /* on-chip RAM block L0 */
   RAML1       : origin = 0x008800, length = 0x000400     /* on-chip RAM block L1 */
   RAML3       : origin = 0x009000, length = 0x001000     /* on-chip RAM block L3 */
   OTP         : origin = 0x3D7800, length = 0x000400     /* on-chip OTP */

   FLASHH      : origin = 0x3D8000, length = 0x004000     /* on-chip FLASH */
   FLASHG      : origin = 0x3DC000, length = 0x004000     /* on-chip FLASH */
   FLASHF      : origin = 0x3E0000, length = 0x004000     /* on-chip FLASH */
   FLASHE      : origin = 0x3E4000, length = 0x004000     /* on-chip FLASH */   
   FLASHD      : origin = 0x3E8000, length = 0x004000     /* on-chip FLASH */
   FLASHC      : origin = 0x3EC000, length = 0x004000     /* on-chip FLASH */
   FLASHA      : origin = 0x3F4000, length = 0x003F80     /* on-chip FLASH */
   CSM_RSVD    : origin = 0x3F7F80, length = 0x000076     /* Part of FLASHA */
   BEGIN       : origin = 0x3F7FF6, length = 0x000002     /* Part of FLASHA */
   CSM_PWL_P0  : origin = 0x3F7FF8, length = 0x000008     /* Part of FLASHA */

   FPUTABLES   : origin = 0x3FD860, length = 0x0006A0     /* FPU Tables in Boot ROM */
   IQTABLES    : origin = 0x3FDF00, length = 0x000B50     /* IQ Math Tables in Boot ROM */
   IQTABLES2   : origin = 0x3FEA50, length = 0x00008C     /* IQ Math Tables in Boot ROM */
   IQTABLES3   : origin = 0x3FEADC, length = 0x0000AA     /* IQ Math Tables in Boot ROM */

   ROM         : origin = 0x3FF3B0, length = 0x000C10     /* Boot ROM */
   RESET       : origin = 0x3FFFC0, length = 0x000002     /* part of boot ROM */
   VECTORS     : origin = 0x3FFFC2, length = 0x00003E     /* part of boot ROM */

PAGE 1 :   /* Data Memory */
   BOOT_RSVD   : origin = 0x000000, length = 0x000050     /* Part of M0, BOOT rom will use this for stack */
   RAMM0       : origin = 0x000050, length = 0x0003B0     /* on-chip RAM block M0 */
   RAMM1       : origin = 0x000400, length = 0x000400     /* on-chip RAM block M1 */
   RAML2       : origin = 0x008C00, length = 0x000400     /* on-chip RAM block L2 */
   RAML4       : origin = 0x00A000, length = 0x002000     /* on-chip RAM block L4 */
   RAML5       : origin = 0x00C000, length = 0x002000     /* on-chip RAM block L5 */
   RAML6       : origin = 0x00E000, length = 0x002000     /* on-chip RAM block L6 */
   RAML7       : origin = 0x010000, length = 0x002000     /* on-chip RAM block L7 */
   RAML8       : origin = 0x012000, length = 0x002000     /* on-chip RAM block L8 */   
   FLASHB      : origin = 0x3F0000, length = 0x004000     /* on-chip FLASH */
}

/* Allocate sections to memory blocks for RAM execution */
SECTIONS
{
   /* Allocate program areas to RAM for fast execution */
   .cinit              : > RAML3       PAGE = 0
   .pinit              : > RAML3       PAGE = 0
   .text               : > RAML3       PAGE = 0
   codestart           : > RAML0       PAGE = 0
   
   /* ramfuncs section for critical functions */
   ramfuncs            : > RAML1       PAGE = 0
   
   csmpasswds          : > CSM_PWL_P0  PAGE = 0
   csm_rsvd            : > CSM_RSVD    PAGE = 0

   /* Allocate uninitalized data sections */
   .stack              : > RAMM0       PAGE = 1
   .ebss               : > RAML4       PAGE = 1
   .esysmem            : > RAML2       PAGE = 1

   /* Initalized sections */
   .econst             : > RAML5       PAGE = 1
   .switch             : > RAML5       PAGE = 1

   /* Allocate IQ math areas */
   IQmath              : > RAML6       PAGE = 1
   IQmathTables        : > IQTABLES,   PAGE = 0, TYPE = NOLOAD
   
   /* Allocate FPU math areas */
   FPUmathTables       : > FPUTABLES,  PAGE = 0, TYPE = NOLOAD
} 