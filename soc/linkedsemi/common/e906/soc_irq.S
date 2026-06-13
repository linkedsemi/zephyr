.section .text.SWINT_Handler_ASM
.global SWINT_Handler_ASM
SWINT_Handler_ASM:
    lw a0, 16(sp)
    la a1, Swint_Handler_C
    jr a1
