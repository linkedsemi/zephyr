

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/lpc.h)

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_LPC_LS lpc_ls.c)
