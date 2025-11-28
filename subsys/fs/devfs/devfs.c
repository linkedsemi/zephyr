/*
 * Copyright 2025 Linkedsemi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdarg.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/fs_sys.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(devfs, LOG_LEVEL_INF);

#define DEVFS_FILE_DIR_NAME_MAX         (CONFIG_DEVFS_FILE_DIR_NAME_MAX+1)

struct devfs_dir
{
    struct devfs_node *cur;
    struct devfs_node *next;
};

struct devfs_node
{
    const struct fs_file_system_t *ops;
    struct fs_file_t *filp;
    struct devfs_node *parent;
    struct devfs_node *child;
    struct devfs_node *brother;
    const char *abs_path;
    int is_dir;
    char name[DEVFS_FILE_DIR_NAME_MAX];
};

struct devfs
{
    struct devfs_node root;
    struct devfs_dir dir;
};

static struct devfs devfs;

struct devfs_node* devfs_lookup(const char *path)
{
    if (strncmp(path, "/dev", 4) == 0)
    {
        if (path[4] == '\0')
            return &devfs.root;
        if (path[4] == '/' && path[5] == '\0')
            return &devfs.root;
    }

    if (strncmp(path, "/dev/", 5) != 0)
        return NULL;

    const char *p = path + 5;
    struct devfs_node *cur = &devfs.root;
    char name[64];

    while (*p)
    {
        char *t = name;
        while (*p && *p != '/')
            *t++ = *p++;
        *t = '\0';

        struct devfs_node *c = cur->child;
        while (c && strcmp(c->name, name) != 0)
            c = c->brother;

        if (!c)
            return NULL;

        cur = c;
        if (*p == '/') p++;
    }

    return cur;
}

static int devfs_open(struct fs_file_t *zfp, const char *path, fs_mode_t flags)
{
    int rc = -ENOTSUP;
    struct devfs_node *node = devfs_lookup(path);
    if (!node || node->is_dir)
        return -EINVAL;

    if (!(node->filp = k_malloc(sizeof (struct fs_file_t))))
        return -ENOMEM;

    if (node->ops->open == NULL)
        return rc;

    node->filp->filep = NULL;
    node->filp->mp = NULL;
    node->filp->flags = 0;

    rc = node->ops->open(node->filp, node->name, flags);
    if (rc < 0)
    {
        LOG_ERR("file open failed (%d)", rc);
        k_free(node->filp);
        return rc;
    }

    node->filp->flags = flags;
    zfp->filep = node;

    return rc;
}

static int devfs_close(struct fs_file_t *zfp)
{
    int rc = -ENOTSUP;
    struct devfs_node *node = zfp->filep;

    if (node->ops->close == NULL)
        return rc;

    rc = node->ops->close(node->filp);
    if (rc < 0)
    {
        LOG_ERR("file close error (%d)", rc);
        return rc;
    }
    k_free(node->filp);
    node->filp = NULL;

    return rc;
}

static ssize_t devfs_read(struct fs_file_t *zfp, void *dest, size_t nbytes)
{
    ssize_t rc = -ENOTSUP;
    struct devfs_node *node = zfp->filep;

    if (node->ops->read == NULL)
        return rc;

    rc = node->ops->read(node->filp, dest, nbytes);
    if (rc < 0)
    {
        LOG_ERR("file read error (%d)", rc);
        return rc;
    }

    return rc;
}

static ssize_t devfs_write(struct fs_file_t *zfp, const void *src, size_t nbytes)
{
    ssize_t rc = -ENOTSUP;
    struct devfs_node *node = zfp->filep;

    if (node->ops->write == NULL)
        return rc;

    rc = node->ops->write(node->filp, src, nbytes);
    if (rc < 0)
    {
        LOG_ERR("file write error (%d)", rc);
        return rc;
    }

    return rc;
}

static int devfs_ioctl(struct fs_file_t *zfp, unsigned long cmd, va_list args)
{
    int rc = -EINVAL;
    struct devfs_node *node = zfp->filep;

    if (node->ops->ioctl == NULL)
        return -ENOTSUP;

    rc = node->ops->ioctl(node->filp, cmd, args);
    if (rc < 0)
    {
        LOG_ERR("file ioctl error (%d)", rc);
        return rc;
    }

    return rc;
}

static int devfs_mount(struct fs_mount_t *mountp)
{
    memset(&devfs, 0, sizeof(struct devfs));
    devfs.root.is_dir = 1;
    devfs.root.abs_path = "/dev";
    strcpy(devfs.root.name, "dev");

    return 0;
}

static void devfs_free_node(struct devfs_node *node)
{
    if (!node)
        return;

    devfs_free_node(node->child);
    devfs_free_node(node->brother);

    if (node != &devfs.root)
        k_free(node);
}

static int devfs_unmount(struct fs_mount_t *mountp)
{
    /* free dir tree */
    devfs_free_node(devfs.root.child);
    return 0;
}

static int devfs_opendir(struct fs_dir_t *dirp, const char *fs_path)
{
    struct devfs_node *node = devfs_lookup(fs_path);
    if (!node || !node->is_dir)
        return -EINVAL;

    dirp->dirp = &devfs.dir;
    devfs.dir.cur = node;
    devfs.dir.next = NULL;

    return 0;
}

static int devfs_readdir(struct fs_dir_t *dirp, struct fs_dirent *entry)
{
    struct devfs_dir *d  = dirp->dirp;
    struct devfs_node *cur;

    if (d->next)
        cur = d->next->brother;
    else
        cur = d->cur->child;

    if (cur)
    {
        strncpy(entry->name, cur->name, sizeof(entry->name));
        entry->type = cur->is_dir ? FS_DIR_ENTRY_DIR : FS_DIR_ENTRY_FILE;
        d->next = cur;
    }
    else
    {
        memset(entry, 0, sizeof(*entry));
    }

    return 0;
}

static int devfs_clsoedir(struct fs_dir_t *dirp)
{
    struct devfs_dir *d  = dirp->dirp;
    if (d)
    {
        d->cur = NULL;
        d->next = NULL;
    }

    return 0;
}

static const struct fs_file_system_t devfs_fs = {
    .mount = devfs_mount,
    .unmount = devfs_unmount,
    .open = devfs_open,
    .read = devfs_read,
    .write = devfs_write,
    .close = devfs_close,
    .ioctl = devfs_ioctl,
    .opendir = devfs_opendir,
    .readdir = devfs_readdir,
    .closedir = devfs_clsoedir
};

int devfs_unregister(const char *path)
{
    if (!path)
        return -EINVAL;

    if (strncmp(path, "/dev/", 5) != 0)
        return -EINVAL;

    const char *p = path + 5;
    struct devfs_node *cur = &devfs.root;
    struct devfs_node *parent = NULL;
    struct devfs_node *prev = NULL;
    char name[DEVFS_FILE_DIR_NAME_MAX];

    while (*p)
    {
        char *t = name;
        while (*p && *p != '/')
            *t++ = *p++;
        *t = '\0';

        parent = cur;
        prev = NULL;
        cur = cur->child;

        while (cur && strcmp(cur->name, name) != 0)
        {
            prev = cur;
            cur = cur->brother;
        }

        if (!cur)
            return -ENOENT;

        if (*p == '/')
            p++;
    }

    if (cur == &devfs.root)
        return -EINVAL;

    if (cur->is_dir)
    {
        if (cur->child != NULL)
            return -ENOTEMPTY;
    }

    if (prev)
        prev->brother = cur->brother;
    else
        parent->child = cur->brother;

    k_free(cur);

    return 0;
}

/*
                           /dev
                           /|\
    jtag0  jtag1    i2c/        uart/  spi1 spi2     spi
                    /|\         /|\                 /|\
                i2c1 i2c2    uart0 uart1        sp1 sp2 sp3
*/

int devfs_register(const char *path, const struct fs_file_system_t *ops)
{
    if (strncmp(path, "/dev/", 5) != 0)
        return -EINVAL;

    const char *p = path + 5;   
    struct devfs_node *cur = &devfs.root;
    char name[64];

    while (*p)
    {
        char *t = name;
        while (*p && *p != '/')
            *t++ = *p++;
        *t = '\0';

        struct devfs_node *child = cur->child;
        struct devfs_node *prev = NULL;

        while (child && strcmp(child->name, name) != 0)
        {
            /* find brother node */
            prev = child;
            child = child->brother;
        }

        if (!child)
        {
            /* no brother node, alloc new node */
            child = k_malloc(sizeof(struct devfs_node));
            if (!child)
                return -ENOMEM;

            memset(child, 0, sizeof(*child));
            strncpy(child->name, name, DEVFS_FILE_DIR_NAME_MAX);
            child->is_dir = 1;
            child->parent = cur;
            /* link new brother */
            if (prev)
                prev->brother = child;
            else
                cur->child = child;
        }
        cur = child;

        if (*p == '/') p++;
    }

    cur->is_dir = 0;
    cur->abs_path = path;
    cur->ops = ops;

    return 0;
}

static int devfs_fs_init(void)
{
    static struct fs_mount_t devfs_fs_mnt = {
        .type = FS_DEVFS,
        .mnt_point = "/dev",
        .fs_data = &devfs
    };

    int ret = fs_register(FS_DEVFS, &devfs_fs);
    if (!ret)
        ret = fs_mount(&devfs_fs_mnt);

    return ret;
}
SYS_INIT(devfs_fs_init, POST_KERNEL, CONFIG_FILE_SYSTEM_INIT_PRIORITY);
