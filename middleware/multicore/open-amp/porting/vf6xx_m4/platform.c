/*
 * Copyright (c) 2016, Toradex AG
 * All rights reserved.
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
 *       platform.c
 *
 * DESCRIPTION
 *
 *       This file is the Implementation of IPC hardware layer interface
 *       for Freescale Vybrid platform.
 *
 **************************************************************************/

#include "platform.h"
#include "plat_porting.h"
#include  "sema4.h"

extern void freertos_env_isr(int vector);

/*--------------------------- Globals ---------------------------------- */

struct hil_platform_ops proc_ops = {
    .enable_interrupt	= _enable_interrupt,
    .notify             = _notify,
    .boot_cpu           = _boot_cpu,
    .shutdown_cpu       = _shutdown_cpu,
};

void CPU2CPUInt0_Handler(void)
{
    MSCM_IRCP1IR = MSCM_IRCP1IR_INT0_MASK;
    SEMA4_Lock(SEMA4, 0);
    freertos_env_isr(0);
    SEMA4_Unlock(SEMA4, 0);
}

void CPU2CPUInt1_Handler(void)
{
    MSCM_IRCP1IR = MSCM_IRCP1IR_INT1_MASK;
    freertos_env_isr(1);
}

void rpmsg_handler(void)
{
    return;
}

int _enable_interrupt(struct proc_vring *vring_hw) {        /*enable_interrupt*/
    env_register_isr(vring_hw->intr_info.vect_id, vring_hw, platform_isr);

    return 0;
}

void _notify(int cpu_id, struct proc_intr *intr_info)
{
    uint32_t vector = intr_info->vect_id;

    MSCM_IRCPGIR = MSCM_IRCPGIR_TLF(1) | MSCM_IRCPGIR_INTID(vector);
}

int _boot_cpu(int cpu_id, unsigned int load_addr)
{
    return 0;
}

void _shutdown_cpu(int cpu_id)
{
}

void platform_isr(int vect_id, void *data)
{
    hil_isr(((struct proc_vring *) data));
}

void platform_interrupt_enable()
{
    MSCM_IRSPRC0 = 0x3;
    MSCM_IRSPRC1 = 0x3;
    NVIC_EnableIRQ(CPU2CPU_INT0_IRQ);
    NVIC_EnableIRQ(CPU2CPU_INT1_IRQ);
}

void platform_interrupt_disable()
{
    NVIC_DisableIRQ(CPU2CPU_INT0_IRQ);
    NVIC_DisableIRQ(CPU2CPU_INT1_IRQ);
}
