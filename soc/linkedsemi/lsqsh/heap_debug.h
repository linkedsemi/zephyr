#ifndef __HEAP_DEBUG_H__
#define __HEAP_DEBUG_H__

#include <zephyr/kernel.h>

void heap_debug_callstack(const char *func, void *ptr, size_t size);
void heap_debug_ptr_push(const char *alloc_name, void *ptr, size_t size);
void heap_debug_ptr_pop(void *ptr, size_t size);
void heap_debug_ptr_replace(void *old_ptr, size_t old_size, const char *alloc_name, void *new_ptr, size_t new_size);
void heap_debug_ptr_dump(void);
bool heap_debug_cs_is_enable(void);
int heap_debug_rd_enable();

#endif //__HEAP_DEBUG_H__
