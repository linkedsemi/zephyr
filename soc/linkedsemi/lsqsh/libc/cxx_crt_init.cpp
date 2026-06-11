#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <new>
#include <zephyr/kernel.h>

void *operator new(size_t size)
{
    void *ptr = malloc(size);
    if (!ptr)
    {
        printf("thread [%s] new [%u] bytes fail\n", k_current_get()->name, size);
        k_thread_abort(k_current_get());
    }

    return ptr;
}

void *operator new[](size_t size)
{
    void *ptr = malloc(size);
    if (!ptr)
    {
        printf("thread [%s] new[] [%u] bytes fail\n", k_current_get()->name, size);
        k_thread_abort(k_current_get());
    }

    return ptr;
}

void operator delete(void *ptr)
{
    free(ptr);
}

void operator delete[](void *ptr)
{
    free(ptr);
}

void* operator new(size_t size, const std::nothrow_t&) noexcept
{
    void *ptr = malloc(size);
    if (!ptr)
    {
        printf("thread [%s] new nothrow [%u] bytes fail\n", k_current_get()->name, size);
        k_thread_abort(k_current_get());
    }

    return ptr;
}

void* operator new[](size_t size, const std::nothrow_t&) noexcept
{
    void *ptr = malloc(size);
    if (!ptr)
    {
        printf("thread [%s] new[] nothrow [%u] bytes fail\n", k_current_get()->name, size);
        k_thread_abort(k_current_get());
    }

    return ptr;
}

void operator delete(void* ptr, const std::nothrow_t&) noexcept
{
    free(ptr);
}

void operator delete[](void* ptr, const std::nothrow_t&) noexcept
{
    free(ptr);
}

