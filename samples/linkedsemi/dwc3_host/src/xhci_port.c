#include <stddef.h>
#include <zephyr/kernel.h>
#include <zephyr/cache.h>

static volatile int event_sync = 0;

void xhci_enqueue_lock()
{

}

void xhci_enqueue_unlock()
{

}

void xhci_event_complete()
{
    event_sync = 1;
}

void xhci_event_wait()
{
    while (!event_sync);
    event_sync = 0;
}

void *xhci_mem_alloc(size_t align, size_t size)
{
    return k_aligned_alloc(align, size);;
}

void xhci_mem_free(void *ptr)
{
    k_free(ptr);
}

void xhci_cache_flush(void *buf, size_t size)
{
    sys_cache_data_flush_range(buf, size);
}

void xhci_cache_invalid(void *buf, size_t size)
{
    sys_cache_data_invd_range(buf, size);
}