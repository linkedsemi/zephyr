#ifndef __DEVFS_H__
#define __DEVFS_H__

#include <zephyr/fs/fs.h>

/* visible path */
int devfs_register(const char *path, const struct fs_file_system_t *devfs_ops);
int devfs_unregister(const char *path);

/* anonymous path */
int devfs_register_anon(const char *path, const struct fs_file_system_t *devfs_ops);
int devfs_unregister_anon(const char *path);

/*
 * Internal devfs file object. Exposed here so that the posix file layer
 * (lib/posix/options/fs.c) can resolve a leaked fd back to the driver's
 * per-device struct (e.g. i2c_device) stored in devfs_file_object.filp.filep
 * and reclaim it after an aborted command.
 */
struct devfs_file_object {
    const struct fs_file_system_t *ops;
    struct fs_file_t filp;
    char file_name[MAX_FILE_NAME + 1];
};

#endif // __DEVFS_H__