#ifndef __DEVFS_H__
#define __DEVFS_H__

int devfs_register(const char *path, const struct fs_file_system_t *devfs_ops);
int devfs_unregister(const char *path);

#endif // __DEVFS_H__