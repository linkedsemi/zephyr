#include <stdarg.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/fs_sys.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/dlist.h>
#include <zephyr/fs/sysfs.h>
#include <zephyr/sys/fdtable.h>

LOG_MODULE_REGISTER(sysfs, LOG_LEVEL_INF);

#define SYSFS_FILE_NUM_MAX              (CONFIG_SYSFS_FILE_NUM_MAX)
#define SYSFS_OPENDIR_NUM_MAX           (16)

struct sysfs_node
{
    struct sysfs_node *parent;
    struct sysfs_node *child;
    struct sysfs_node *brother;
    sys_dlist_t attributes;
    char name[MAX_FILE_NAME+1];
};

struct sysfs_dir
{
    struct sysfs_node *node;
    struct sysfs_node *next;
    sys_dnode_t *attr;
    bool in_attr;
};

struct sysfs
{
    struct sysfs_node root;
    struct k_spinlock lock;
    struct sysfs_dir dir;
};

static struct sysfs sysfs;

K_MEM_SLAB_DEFINE_STATIC(node_pool, sizeof(struct sysfs_node), SYSFS_FILE_NUM_MAX, 4);
K_MEM_SLAB_DEFINE_STATIC(dir_pool, sizeof(struct sysfs_dir), SYSFS_OPENDIR_NUM_MAX, 4);

static bool sysfs_path_is_valid(const char *path)
{
    const char *p = path + 5; // skip /sys/

    while (*p)
    {
        size_t len = 0;
        while (*p && *p != '/')
        {
            if (len >= MAX_FILE_NAME)
            {
                LOG_ERR("path too long '%s'", path);
                return false;
            }
            p++;
            len++;
        }
        if (*p == '/')
            p++;
    }

    return true;
}

static int sysfs_walk_full(const char *path,
                           struct sysfs_node **parent,
                           struct sysfs_node **cur,
                           struct sysfs_node **prev)
{
    if (!path)
        return -EINVAL;

    if (strncmp(path, "/sys", 4) == 0 &&
        (path[4] == '\0' || (path[4] == '/' && path[5] == '\0')))
    {
        *parent = NULL;
        *cur = &sysfs.root;
        *prev = NULL;
        return 0;
    }

    if (sysfs_path_is_valid(path) == false)
        return -EINVAL;

    const char *p = path + 5;
    struct sysfs_node *c = &sysfs.root;
    struct sysfs_node *pnode = NULL;
    struct sysfs_node *pprev = NULL;
    char name[MAX_FILE_NAME + 1];

    while (*p)
    {
        size_t len = 0;

        while (*p && *p != '/')
            name[len++] = *p++;
        name[len] = '\0';

        pnode = c;
        pprev = NULL;
        c = c->child;

        while (c && strcmp(c->name, name) != 0)
        {
            pprev = c;
            c = c->brother;
        }

        if (!c)
            return -ENOENT;

        if (*p == '/')
            p++;
    }

    *parent = pnode;
    *cur    = c;
    *prev   = pprev;
    return 0;
}

static int sysfs_walk_attribute(const char *path, struct sysfs_node **node, const char **attr_name)
{
    if (!path || strncmp(path, "/sys/", 5) != 0)
        return -EINVAL;

    const char *p = path + 5;
    struct sysfs_node *cur = &sysfs.root;
    char name[MAX_FILE_NAME + 1];
    const char *seg = p;

    while (*p)
    {
        seg = p;
        while (*p && *p != '/')
            p++;

        if (*p == '\0')
            break;

        size_t len = p - seg;
        if (len > MAX_FILE_NAME)
            return -ENAMETOOLONG;

        memcpy(name, seg, len);
        name[len] = '\0';

        struct sysfs_node *c = cur->child;
        while (c && strcmp(c->name, name) != 0)
            c = c->brother;

        if (!c)
            return -ENOENT;

        cur = c;
        p++;
    }

    *node = cur;
    *attr_name = seg;
    return 0;
}

static struct sysfs_node *sysfs_lookup(const char *path)
{
    int ret;
    struct sysfs_node *parent, *cur, *prev;

    k_spinlock_key_t key = k_spin_lock(&sysfs.lock);
    ret = sysfs_walk_full(path, &parent, &cur, &prev);
    k_spin_unlock(&sysfs.lock, key);

    return ret == 0 ? cur : NULL;
}

static struct sysfs_node *sysfs_lookup_attribute(const char *path, const char **attr_name)
{
    int ret;
    struct sysfs_node *node;

    k_spinlock_key_t key = k_spin_lock(&sysfs.lock);
    ret = sysfs_walk_attribute(path, &node, attr_name);
    k_spin_unlock(&sysfs.lock, key);

    return ret == 0 ? node : NULL;
}

struct sysfs_node *sysfs_mkdir(const char *path)
{
    if (!path || strncmp(path, "/sys/", 5) != 0 || !sysfs_path_is_valid(path))
    {
        LOG_ERR("invalid path '%s'", path);
        return NULL;
    }

    const char *p = path + 5;
    struct sysfs_node *cur = &sysfs.root;
    char name[MAX_FILE_NAME + 1];

    k_spinlock_key_t key = k_spin_lock(&sysfs.lock);

    while (*p)
    {
        size_t len = 0;

        while (*p && *p != '/')
            name[len++] = *p++;
        name[len] = '\0';

        bool is_last = (*p == '\0');

        struct sysfs_node *child = cur->child;
        struct sysfs_node *prev  = NULL;

        while (child && strcmp(child->name, name) != 0)
        {
            prev = child;
            child = child->brother;
        }

        if (!child)
        {
            if (k_mem_slab_alloc(&node_pool,
                                 (void **)&child,
                                 K_NO_WAIT))
            {
                LOG_ERR("sysfs node pool full!!");
                cur = NULL;
                goto out;
            }

            memset(child, 0, sizeof(*child));
            memcpy(child->name, name, len + 1);
            child->parent = cur;
            sys_dlist_init(&child->attributes);

            if (prev)
                prev->brother = child;
            else
                cur->child = child;
        }
        else if (is_last)
        {
            LOG_ERR("dir already exists '%s'", path);
            cur = NULL;
            goto out;
        }

        cur = child;

        if (*p == '/')
            p++;
    }

out:
    k_spin_unlock(&sysfs.lock, key);
    return cur;
}

int sysfs_rmdir(const char *path)
{
    int ret;
    struct sysfs_node *parent, *cur, *prev;

    k_spinlock_key_t key = k_spin_lock(&sysfs.lock);

    ret = sysfs_walk_full(path, &parent, &cur, &prev);
    if (ret)
        goto out;

    if (!parent)
    {
        ret = -EINVAL;
        goto out;
    }

    if (cur->child)
    {
        ret = -ENOTEMPTY;
        goto out;
    }

    if (prev)
        prev->brother = cur->brother;
    else
        parent->child = cur->brother;

out:
    k_spin_unlock(&sysfs.lock, key);

    if (ret == 0)
        k_mem_slab_free(&node_pool, cur);

    return ret;
}

int sysfs_add_attribute(struct sysfs_node *node, sysfs_attr_t attribute)
{
    sys_dlist_insert(&node->attributes, &attribute->node);
    return 0;
}

int sysfs_del_attribute(sysfs_attr_t attribute)
{
    sys_dlist_remove(&attribute->node);
    return 0;
}

static int sysfs_open(struct fs_file_t *filp, const char *fs_path, fs_mode_t flags)
{
    int ret = 0;
    sys_dnode_t *n;
    const char *attr_name;
    struct sysfs_node *node = sysfs_lookup_attribute(fs_path, &attr_name);

    if (!node)
    {
        LOG_ERR("invalid node for path '%s'", fs_path);
        return -EINVAL;
    }

    struct sysfs_attribute *attr = NULL;
    int name_len = strlen(attr_name);

    SYS_DLIST_FOR_EACH_NODE(&node->attributes, n)
    {
        struct sysfs_attribute *_attr = CONTAINER_OF(n, struct sysfs_attribute, node);
        if ((strlen(_attr->name) == name_len) && strncmp(_attr->name, attr_name, name_len) == 0)
        {
            attr = _attr;
            break;
        }
    }

    if (attr && attr->ops->open && attr->ops->open(attr) == 0)
    {
        filp->filep = attr;
    }
    else
    {
        LOG_ERR("not openable for path '%s'", fs_path);
        ret = -EINVAL;
    }

    return ret;
}

static int sysfs_close(struct fs_file_t *filp)
{
    int rc = -ENOTSUP;
    struct sysfs_attribute *attr = filp->filep;

    if (attr->ops->close == NULL)
    {
        LOG_ERR("invalid file pointer or close not supported"); 
        return rc;
    }

    rc = attr->ops->close(attr);
    if (rc < 0)
    {
        LOG_ERR("file close error (%d)", rc);
    }
    filp->filep = NULL;

    return rc;
}

static int sysfs_ioctl(struct fs_file_t *filp, unsigned long cmd, va_list args)
{
    int rc = -EINVAL;
    struct sysfs_attribute *attr = filp->filep;

    if (attr->ops->ioctl == NULL)
    {
        LOG_ERR("invalid file pointer or ioctl not supported");
        return -ENOTSUP;
    }

    if (cmd == ZFD_IOCTL_SET_LOCK)
        return -EOPNOTSUPP;

    rc = attr->ops->ioctl(attr, cmd, args);
    if (rc < 0)
    {
        LOG_ERR("file ioctl error (%d)", rc);
    }

    return rc;
}

static ssize_t sysfs_read(struct fs_file_t *filp, void *dest, size_t nbytes)
{
    ssize_t rc = -ENOTSUP;
    struct sysfs_attribute *attr = filp->filep;

    if (attr->ops->read == NULL)
    {
        LOG_ERR("invalid file pointer or read not supported");
        return rc;
    }

    rc = attr->ops->read(attr, dest, nbytes);
    if (rc < 0)
    {
        LOG_ERR("file read error (%d)", rc);
    }

    return rc;
}

static ssize_t sysfs_write(struct fs_file_t *filp, const void *src, size_t nbytes)
{
    ssize_t rc = -ENOTSUP;
    struct sysfs_attribute *attr = filp->filep;

    if (attr->ops->write == NULL)
    {
        LOG_ERR("invalid file pointer or write not supported");
        return rc;
    }

    rc = attr->ops->write(attr, src, nbytes);
    if (rc < 0)
    {
        LOG_ERR("file write error (%d)", rc);
    }

    return rc;
}

static int sysfs_opendir(struct fs_dir_t *dirp, const char *fs_path)
{
    struct sysfs_node *node = sysfs_lookup(fs_path);
    if (!node)
        return -EINVAL;

    struct sysfs_dir *dir;

    if (k_mem_slab_alloc(&dir_pool, (void **)&dir, K_NO_WAIT))
    {
        LOG_ERR("dir pool full , opendir '%s' fail", fs_path);
        return -ENOMEM;
    }

    dirp->dirp = dir;
    dir->node = node;
    dir->next = node->child;
    dir->attr = sys_dlist_peek_head(&node->attributes);
    dir->in_attr = false;

    return 0;
}

static int sysfs_readdir(struct fs_dir_t *dirp, struct fs_dirent *entry)
{
    struct sysfs_dir *dir = dirp->dirp;

    if (!dir->in_attr && dir->next)
    {
        strncpy(entry->name, dir->next->name, MAX_FILE_NAME);
        entry->type = FS_DIR_ENTRY_DIR;
        dir->next = dir->next->brother;
        return 0;
    }
    dir->in_attr = true;

    if (dir->attr)
    {
        struct sysfs_attribute *attr = CONTAINER_OF(dir->attr, struct sysfs_attribute, node);
        entry->type = FS_DIR_ENTRY_FILE;
        strncpy(entry->name, attr->name, MAX_FILE_NAME);
        dir->attr = sys_dlist_peek_next(&dir->node->attributes, dir->attr);
    }
    else
    {
        memset(entry, 0, sizeof(*entry));
    }

    return 0;
}

static int sysfs_closedir(struct fs_dir_t *dirp)
{
    k_mem_slab_free(&dir_pool, dirp->dirp);
    return 0;
}

static int sysfs_stat(struct fs_mount_t *mountp, const char *path, struct fs_dirent *entry)
{
    const char *attr_name = NULL;
    struct sysfs_node *node = sysfs_lookup(path);
    node = node ? node : sysfs_lookup_attribute(path, &attr_name);

    if (!node)
    {
        LOG_ERR("path not found '%s'", path); 
        return -EINVAL;
    }

    if (attr_name)
    {
        entry->type = FS_DIR_ENTRY_FILE;
        strncpy(entry->name, attr_name, MAX_FILE_NAME);
        entry->name[MAX_FILE_NAME] = '\0';
    }
    else
    {
        entry->type = FS_DIR_ENTRY_DIR;
        strncpy(entry->name, node->name, MAX_FILE_NAME);
        entry->name[MAX_FILE_NAME] = '\0';
    }

    return 0;
}

static int sysfs_statvfs(struct fs_mount_t *mountp, const char *path, struct fs_statvfs *stat)
{
    const char *attr_name = NULL;
    struct sysfs_node *node = sysfs_lookup(path);
    node = node ? node : sysfs_lookup_attribute(path, &attr_name);

    if (!node)
    {
        LOG_ERR("path not found '%s'", path); 
        return -EINVAL;
    }

    stat->f_bsize  = 1024;
    stat->f_frsize = 1024;
    stat->f_blocks = 1;
    stat->f_bfree  = 1;

    return 0;
}

static int sysfs_mount(struct fs_mount_t *mountp)
{
    memset(&sysfs, 0, sizeof(struct sysfs));
    strcpy(sysfs.root.name, "sys");

    return 0;
}

static int sysfs_unmount(struct fs_mount_t *mountp)
{
    return 0;
}

static const struct fs_file_system_t sysfs_ops = {
    .mount = sysfs_mount,
    .unmount = sysfs_unmount,
    .open = sysfs_open,
    .close = sysfs_close,
    .ioctl = sysfs_ioctl,
    .read = sysfs_read,
    .write = sysfs_write,
    .opendir = sysfs_opendir,
    .readdir = sysfs_readdir,
    .closedir = sysfs_closedir,
    .stat = sysfs_stat,
    .statvfs = sysfs_statvfs
};

static int sysfs_init(void)
{
    static struct fs_mount_t sysfs_mnt = {
        .type = FS_SYSFS,
        .mnt_point = "/sys",
        .fs_data = &sysfs
    };

    int ret = fs_register(FS_SYSFS, &sysfs_ops);
    if (!ret)
        ret = fs_mount(&sysfs_mnt);

    return ret;
}
SYS_INIT(sysfs_init, POST_KERNEL, CONFIG_FILE_SYSTEM_INIT_PRIORITY);



