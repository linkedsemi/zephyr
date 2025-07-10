#ifndef _INCLUDE_ZEPHYR_DRIVERS_FLASH_SOC_FLASH_LS_H
#define _INCLUDE_ZEPHYR_DRIVERS_FLASH_SOC_FLASH_LS_H

/** \brief flash_ls_get_part_info
 *  \param[in] dev flash device
 *  \param[in] idx partition table index. start from 0
 *  \param[out] base
 *  \param[out] size
 *  \param[out] attr
 *  \return 0 for success
 */
int flash_ls_get_part_info(const struct device *dev, uint8_t idx, uint32_t *base,
                   uint32_t *size, uint32_t *attr);

/** \brief flash_ls_set_part_info
 *  \param[in] dev flash device
 *  \param[in] idx partition table index. start from 0
 *  \param[in] base
 *  \param[in] size
 *  \param[in] attr
 *  \return 0 for success
 */
int flash_ls_set_part_info(const struct device *dev, uint8_t idx, uint32_t base,
                   uint32_t size, uint32_t attr);

/** \brief flash_ls_set_part_attr
 *  \param[in] dev flash device
 *  \param[in] idx partition table index. start from 0
 *  \param[in] attr
 *  \return 0 for success
 */
int flash_ls_set_part_attr(const struct device *dev, uint8_t idx, uint32_t attr);

#endif /* _INCLUDE_ZEPHYR_DRIVERS_FLASH_SOC_FLASH_LS_H */
