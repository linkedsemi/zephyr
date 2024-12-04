#ifndef ZEPHYR_DRIVERS_SDHCI_OF_LINKEDSEMI_H
#define ZEPHYR_DRIVERS_SDHCI_OF_LINKEDSEMI_H

int linkedsemi_sdhci_reinit(const struct device *dev);
int linkedsemi_sdhci_deinit(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_SDHCI_OF_LINKEDSEMI_H */
