MEMORY
{
	AUXRAM (rwx) : ORIGIN = 0x50004000, LENGTH = 0x4000
}

#include <zephyr/arch/arm/cortex_m/scripts/linker.ld>