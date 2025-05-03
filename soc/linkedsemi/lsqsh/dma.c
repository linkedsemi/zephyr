#include "soc_dma.h"
#include "field_manipulate.h"
#include "reg_base_addr.h"
#include "reg_sysc_app_cpu.h"

void soc_dma_channel_handshake_set(uint32_t base, uint8_t ch_idx, uint8_t handshake)
{
    switch (base)
    {
    case APP_DWDMAC1_ADDR:
        MODIFY_REG(SYSC_APP_CPU->DMAC1_CH_SEL[ch_idx / 2], 0xff << ((ch_idx % 2) * 16), (handshake | 0x100) << ((ch_idx % 2) * 16));
        break;
    case APP_DWDMAC2_ADDR:
        MODIFY_REG(SYSC_APP_CPU->DMAC2_CH_SEL[ch_idx / 2], 0xff << ((ch_idx % 2) * 16), (handshake | 0x100) << ((ch_idx % 2) * 16));
        break;
    default:
        break;
    }
}
