xip_banned_start = .;
KEEP(*(".exception.entry.*"))
*(".exception.other.*")
*(.xip_banned*)
*(.text*)
xip_banned_end = .;
