
.em (NOLOAD) :
{
    __em_start__ = .;
    *ls_ble_controller.a*(.bss*)  
    *ll_buf.*(.bss*)             
    *stack_var.*(.bss*) 
    *libkernel.a:mempool.*(.noinit.kheap_buf__system_heap .noinit.*.kheap_buf__system_heap)
    . = ALIGN(8 );         
    __em_end__ = .;     
} > AUXRAM

