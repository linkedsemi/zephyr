

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/kcs.h)

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_KCS_LS kcs_ls.c)
