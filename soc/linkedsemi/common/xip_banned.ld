xip_banned_start = .;
KEEP(*(".exception.entry.*"))
*(".exception.other.*")
*(.xip_banned*)
xip_banned_end = .;
