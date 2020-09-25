
#include "hal.h"

/* Bit definition for SAI_xCR1 register */
#define SAI_xCR1_MODE_MASTER_TX (0b00 << SAI_xCR1_MODE_Pos)
#define SAI_xCR1_MODE_MASTER_RX (0b01 << SAI_xCR1_MODE_Pos)
#define SAI_xCR1_MODE_SLAVE_TX  (0b10 << SAI_xCR1_MODE_Pos)
#define SAI_xCR1_MODE_SLAVE_RX  (0b11 << SAI_xCR1_MODE_Pos)

#define SAI_xCR1_DS_8  (0b010 << SAI_xCR1_DS_Pos)
#define SAI_xCR1_DS_10 (0b011 << SAI_xCR1_DS_Pos)
#define SAI_xCR1_DS_16 (0b100 << SAI_xCR1_DS_Pos)
#define SAI_xCR1_DS_20 (0b101 << SAI_xCR1_DS_Pos)
#define SAI_xCR1_DS_24 (0b110 << SAI_xCR1_DS_Pos)
#define SAI_xCR1_DS_32 (0b111 << SAI_xCR1_DS_Pos)

#define SAI_xCR1_SYNCEN_ASYNC   (0b00 << SAI_xCR1_SYNCEN_Pos)
#define SAI_xCR1_SYNCEN_SYNCINT (0b01 << SAI_xCR1_SYNCEN_Pos)
#define SAI_xCR1_SYNCEN_SYNCEXT (0b10 << SAI_xCR1_SYNCEN_Pos)

#define SAI_xCR1_MCKDIV_(x) (((x / 2) << SAI_xCR1_MCKDIV_Pos) & SAI_xCR1_MCKDIV_Msk) // must be 1 or even and <=32

/* Bit definition for SAI_xFRCR register */
#define SAI_xFRCR_FRL_(x) (((x - 1) << SAI_xFRCR_FRL_Pos) & SAI_xFRCR_FRL_Msk)
#define SAI_xFRCR_FSALL_(x) (((x - 1) << SAI_xFRCR_FSALL_Pos) & SAI_xFRCR_FSALL_Msk)
