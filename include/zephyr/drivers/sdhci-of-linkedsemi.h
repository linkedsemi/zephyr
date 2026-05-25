#ifndef ZEPHYR_DRIVERS_SDHCI_OF_LINKEDSEMI_H
#define ZEPHYR_DRIVERS_SDHCI_OF_LINKEDSEMI_H

int linkedsemi_sdhci_reinit(const struct device *dev);
int linkedsemi_sdhci_deinit(const struct device *dev);
#if defined(CONFIG_DEBUG_COREDUMP_BACKEND_EMMC)
int linkedsemi_sdhci_get_coredump_info(const struct device *dev, uint32_t *start_block, uint32_t *block_count);
bool linkedsemi_sdhci_card_ready(const struct device *dev);
#endif

#endif /* ZEPHYR_DRIVERS_SDHCI_OF_LINKEDSEMI_H */
