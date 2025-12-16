#ifndef __HEAP_DEBUG_H__
#define __HEAP_DEBUG_H__

void heap_debug_callstack(const char *func, size_t ptr, size_t size);
void heap_debug_ptr_push(void *ptr, size_t size);
void heap_debug_ptr_pop(void *ptr);
void heap_debug_ptr_dump(void);
bool heap_debug_cs_is_enable(void);

#endif //__HEAP_DEBUG_H__