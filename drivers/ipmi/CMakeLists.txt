

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/ipmi.h)

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_IPMI_KCS_LS ipmi_kcs_ls.c)
