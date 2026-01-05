#ifndef __SYS_FS_H__
#define __SYS_FS_H__

struct sysfs_attribute;
struct sysfs_node;

#define SYSFS_ATTRIBUTE_NAME_MAX                    (32)

typedef struct sysfs_attribute* sysfs_attr_t;

struct sysfs_attribute_ops
{
    int (*open)(sysfs_attr_t attr);
    ssize_t (*read)(sysfs_attr_t attr, void *buf, size_t size);
    ssize_t (*write)(sysfs_attr_t attr, const void *buf, size_t size);
    int (*close)(sysfs_attr_t attr);
    int (*ioctl)(sysfs_attr_t attr, unsigned long cmd, va_list args);
};

struct sysfs_attribute
{
    struct sysfs_attribute_ops *ops;
    void *user_data;
    sys_dnode_t node;
    char name[SYSFS_ATTRIBUTE_NAME_MAX + 1];
};

struct sysfs_node *sysfs_mkdir(const char *path);
int sysfs_rmdir(const char *path);
int sysfs_add_attribute(struct sysfs_node *node, struct sysfs_attribute *attribute);
int sysfs_del_attribute(struct sysfs_attribute *attribute);

#endif //__SYS_FS_H__