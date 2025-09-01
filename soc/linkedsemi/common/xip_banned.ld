xip_banned_start = .;
KEEP(*(".exception.entry.*"))
*(".exception.other.*")
*(.xip_banned*)
*(.text*)
*(.rodata*)
xip_banned_end = .;
