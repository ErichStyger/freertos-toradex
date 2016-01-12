/*
 * Copyright (c) 2014, Mentor Graphics Corporation
 * All rights reserved.
 * Copyright (c) 2015 Xilinx, Inc. All rights reserved.
 * Copyright (c) 2015 Freescale, Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of Mentor Graphics Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**************************************************************************
 * FILE NAME
 *
 *       freertos_env.c
 *
 *
 * DESCRIPTION
 *
 *       This file is Free RTOS Implementation of env layer for OpenAMP.
 *
 *
 **************************************************************************/

#include "env.h"
#include "../config/config.h"

#include <stdlib.h>
#include <string.h>

#if (defined(__CC_ARM))
#define MEM_BARRIER()      __schedule_barrier()
#elif (defined(__GNUC__))
#define MEM_BARRIER()      asm volatile("" ::: "memory")
#else
#define MEM_BARRIER()
#endif

/*
 * function decalaration for platform provided facility
 */
extern void platform_interrupt_enable(void);
extern void platform_interrupt_disable(void);

/**
 * Structure to keep track of registered ISR's.
 */
struct isr_info isr_table[ISR_COUNT];
int Intr_Count = 0;

/**
 * env_init
 *
 * Initializes OS/BM environment.
 *
 */
int env_init() {
    return 0;
}

/**
 * env_deinit
 *
 * Uninitializes OS/BM environment.
 *
 * @returns - execution status
 */

int env_deinit() {
    return 0;
}
/**
 * env_allocate_memory - implementation
 *
 * @param size
 */
void *env_allocate_memory(unsigned int size)
{
    return (pvPortMalloc(size));
}

/**
 * env_free_memory - implementation
 *
 * @param ptr
 */
void env_free_memory(void *ptr)
{
    if (ptr != NULL)
    {
        vPortFree(ptr);
    }
}

/**
 *
 * env_memset - implementation
 *
 * @param ptr
 * @param value
 * @param size
 */
void env_memset(void *ptr, int value, unsigned long size)
{
    memset(ptr, value, size);
}

/**
 *
 * env_memcpy - implementation
 *
 * @param dst
 * @param src
 * @param len
 */
void env_memcpy(void *dst, void const * src, unsigned long len) {
    memcpy(dst,src,len);
}

/**
 *
 * env_strcmp - implementation
 *
 * @param dst
 * @param src
 */

int env_strcmp(const char *dst, const char *src){
    return (strcmp(dst, src));
}

/**
 *
 * env_strncpy - implementation
 *
 * @param dest
 * @param src
 * @param len
 */
void env_strncpy(char * dest, const char *src, unsigned long len)
{
    strncpy(dest, src, len);
}

/**
 *
 * env_strncmp - implementation
 *
 * @param dest
 * @param src
 * @param len
 */
int env_strncmp(char * dest, const char *src, unsigned long len)
{
    return (strncmp(dest, src, len));
}

/**
 *
 * env_mb - implementation
 *
 */
void env_mb()
{
    MEM_BARRIER();
}

/**
 * osalr_mb - implementation
 */
void env_rmb()
{
    MEM_BARRIER();
}

/**
 * env_wmb - implementation
 */
void env_wmb()
{
    MEM_BARRIER();
}

/**
 * env_map_vatopa - implementation
 *
 * @param address
 */
unsigned long env_map_vatopa(void *address)
{
    return (unsigned long)address;
}

/**
 * env_map_patova - implementation
 *
 * @param address
 */
void *env_map_patova(unsigned long address)
{
    return ((void *)address);
}

/**
 * env_create_mutex
 *
 * Creates a mutex with the given initial count.
 *
 */
int env_create_mutex(void **lock, int count)
{
    return 0;
}

/**
 * env_delete_mutex
 *
 * Deletes the given lock
 *
 */
void env_delete_mutex(void *lock)
{
}

/**
 * env_lock_mutex
 *
 * Tries to acquire the lock, if lock is not available then call to
 * this function will suspend.
 *
 * system level interrupt is disabled to avoid race condition, this has the same effect of a lock
 *
 */
void env_lock_mutex(void *lock)
{
    env_disable_interrupts();
}

/**
 * env_unlock_mutex
 *
 * Releases the given lock.
 *
 * system level interrupt is disabled to avoid race condition, this has the same effect of a lock
 *
 */

void env_unlock_mutex(void *lock)
{
    env_restore_interrupts();
}


/**
 * env_create_sync_lock
 *
 * Creates a synchronization lock primitive. It is used
 * when signal has to be sent from the interrupt context to main
 * thread context.
 */
int env_create_sync_lock(void **lock , int state)
{
    return 0;
}

/**
 * env_delete_sync_lock
 *
 * Deletes the given lock
 *
 */
void env_delete_sync_lock(void *lock)
{
}

/**
 * env_acquire_sync_lock
 *
 * Tries to acquire the lock, if lock is not available then call to
 * this function waits for lock to become available.
 */
void env_acquire_sync_lock(void *lock)
{
}

/**
 * env_release_sync_lock
 *
 * Releases the given lock.
 */

void env_release_sync_lock(void *lock)
{
}

/**
 * env_sleep_msec
 *
 * Suspends the calling thread for given time , in msecs.
 */

void env_sleep_msec(int num_msec)
{
    /* portSUPPRESS_TICKS_AND_SLEEP */
    vTaskDelay(num_msec * portTICK_PERIOD_MS);
}

/**
 * env_disable_interrupts
 *
 * Disables system interrupts
 *
 */
void env_disable_interrupts()
{
    taskDISABLE_INTERRUPTS();
}

/**
 * env_restore_interrupts
 *
 * Enables system interrupts
 *
 */
void env_restore_interrupts()
{
    taskENABLE_INTERRUPTS();
}

/**
 * env_register_isr
 *
 * Registers interrupt handler for the given interrupt vector.
 *
 * @param vector - vring index
 * @param isr    - interrupt handler
 */
void env_register_isr(int vector , void *data ,
                void (*isr)(int vector , void *data))
{
    env_disable_interrupts();

    if(Intr_Count < ISR_COUNT)
    {
        /* Save interrupt data */
        isr_table[Intr_Count].vector = vector;
        isr_table[Intr_Count].data = data;
        isr_table[Intr_Count++].isr = isr;
    }

    env_restore_interrupts();
}

/**
 * env_enable_interrupt
 *
 * Enables the given interrupt
 *
 * @param vector   - vring index
 * @param priority - interrupt priority
 * @param polarity - interrupt polarity
 */

void env_enable_interrupt(unsigned int vector , unsigned int priority ,
                unsigned int polarity)
{
    platform_interrupt_enable();
}

/**
 * env_disable_interrupt
 *
 * Disables the given interrupt
 *
 * @param vector   - interrupt vector number
 */

void env_disable_interrupt(unsigned int vector)
{
    platform_interrupt_disable();
}

/**
 * env_map_memory
 *
 * Enables memory mapping for given memory region.
 *
 * @param pa   - physical address of memory
 * @param va   - logical address of memory
 * @param size - memory size
 * param flags - flags for cache/uncached  and access type
 */

void env_map_memory(unsigned int pa, unsigned int va, unsigned int size,
                    unsigned int flags)
{
}

/**
 * env_disable_cache
 *
 * Disables system caches.
 *
 */

void env_disable_cache() 
{
}

/**
 * 
 * env_get_timestamp
 *
 * Returns a 64 bit time stamp.
 *
 *
 */
unsigned long long env_get_timestamp(void)
{
    return 0;
}

/*========================================================= */
/* Util data / functions for MQX */

void freertos_env_isr(int vector) {
    int idx;
    struct isr_info *info;

    env_disable_interrupt(vector);
    for(idx = 0; idx < ISR_COUNT; idx++)
    {
        info = &isr_table[idx];
        if(info->vector == vector)
        {
            info->isr(info->vector , info->data);   /*platform_isr*/
            env_enable_interrupt(info->vector , info->priority, info->type);
            break;
        }
    }
}
