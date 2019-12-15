/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Simon Kuenzer <simon.kuenzer@neclab.eu>
 *
 *
 * Copyright (c) 2017, NEC Europe Ltd., NEC Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
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
 *
 * THIS HEADER MAY NOT BE EXTRACTED OR MODIFIED IN ANY WAY.
 */

#include <string.h>
#include <uk/plat/common/sections.h>

#include <common/gnttab.h>
#if (defined __X86_32__) || (defined __X86_64__)
#include <xen-x86/setup.h>
#include <xen-x86/mm_pv.h>
#include <xen-x86/mm.h>
#elif (defined __ARM_32__) || (defined __ARM_64__)
#include <xen-arm/setup.h>
#include <xen-arm/mm.h>
#endif

#include <xen/memory.h>
#include <common/hypervisor.h>

#include <uk/assert.h>

int ukplat_memregion_count(void)
{
	return (int) _libxenplat_mrd_num + 7;
}

int ukplat_memregion_get(int i, struct ukplat_memregion_desc *m)
{

	UK_ASSERT(m);

	switch (i) {
	case 0: /* text */
		m->base  = (void *) __TEXT;
		m->len   = (size_t) __ETEXT - (size_t) __TEXT;
		m->flags = (UKPLAT_MEMRF_RESERVED
			    | UKPLAT_MEMRF_READABLE);
#if CONFIG_UKPLAT_MEMRNAME
		m->name  = "text";
#endif
		break;
	case 1: /* eh_frame */
		m->base  = (void *) __EH_FRAME_START;
		m->len   = (size_t) __EH_FRAME_END
				- (size_t) __EH_FRAME_START;
		m->flags = (UKPLAT_MEMRF_RESERVED
			    | UKPLAT_MEMRF_READABLE);
#if CONFIG_UKPLAT_MEMRNAME
		m->name  = "eh_frame";
#endif
		break;
	case 2: /* eh_frame_hdr */
		m->base  = (void *) __EH_FRAME_HDR_START;
		m->len   = (size_t) __EH_FRAME_HDR_END
				- (size_t) __EH_FRAME_HDR_START;
		m->flags = (UKPLAT_MEMRF_RESERVED
			    | UKPLAT_MEMRF_READABLE);
#if CONFIG_UKPLAT_MEMRNAME
		m->name  = "eh_frame_hdr";
#endif
		break;
	case 3:	/* ro data */
		m->base  = (void *) __RODATA;
		m->len   = (size_t) __ERODATA - (size_t) __RODATA;
		m->flags = (UKPLAT_MEMRF_RESERVED
			       | UKPLAT_MEMRF_READABLE);
#if CONFIG_UKPLAT_MEMRNAME
		m->name  = "rodata";
#endif
		break;
	case 4: /* ctors */
		m->base  = (void *) __CTORS;
		m->len   = (size_t) __ECTORS - (size_t) __CTORS;
		m->flags = (UKPLAT_MEMRF_RESERVED
			    | UKPLAT_MEMRF_READABLE);
#if CONFIG_UKPLAT_MEMRNAME
		m->name  = "ctors";
#endif
		break;
	case 5: /* data */
		m->base  = (void *) __DATA;
		m->len   = (size_t) __EDATA - (size_t) __DATA;
		m->flags = (UKPLAT_MEMRF_RESERVED
			    | UKPLAT_MEMRF_READABLE
			    | UKPLAT_MEMRF_WRITABLE);
#if CONFIG_UKPLAT_MEMRNAME
		m->name  = "data";
#endif
		break;
	case 6: /* bss */
		m->base  = (void *) __BSS_START;
		m->len   = (size_t) __END - (size_t) __BSS_START;
		m->flags = (UKPLAT_MEMRF_RESERVED
			    | UKPLAT_MEMRF_READABLE
			    | UKPLAT_MEMRF_WRITABLE);
#if CONFIG_UKPLAT_MEMRNAME
		m->name  = "bss";
#endif
		break;
	default:
		if (i < 0 || i >= ukplat_memregion_count()) {
			m->base  = __NULL;
			m->len   = 0;
			m->flags = 0x0;
#if CONFIG_UKPLAT_MEMRNAME
			m->name  = __NULL;
#endif
			return -1;
		} else {
			memcpy(m, &_libxenplat_mrd[i - 7], sizeof(*m));
		}
		break;
	}

	return 0;
}

void mm_init(void)
{
	arch_mm_init(ukplat_memallocator_get());
}

int _ukplat_mem_mappings_init(void)
{
	mm_init();
#ifdef CONFIG_XEN_GNTTAB
	gnttab_init();
#endif
	return 0;
}

void ukplat_stack_set_current_thread(void *thread_addr)
{
	/* TODO revisit for HVM */
	extern char irqstack[];
	*((unsigned long *) irqstack) = (unsigned long) thread_addr;
}

/**
 * set up and call Xen hypercall to ask for memory back from Xen
*/
int xenmem_reservation_increase(int count, xen_pfn_t *frames, int order)
{
        struct xen_memory_reservation res = {
#if __XEN_INTERFACE_VERSION__ >= 0x00030209
                .memflags = 0;
#else
                .address_bits = 0,
#endif
                .extent_order = order,
                .domid        = DOMID_SELF
        };

        set_xen_guest_handle(res.extent_start, frames);
        res.nr_extents = count;

		/* Needs physical frame number */
        int r = HYPERVISOR_memory_op(XENMEM_populate_physmap, &res);
        return r;
}

/**
 * set up and call Xen hypercall to give memory to Xen
*/
int xenmem_reservation_decrease(int count, xen_pfn_t *frames, int order)
{
        struct xen_memory_reservation res = {
#if __XEN_INTERFACE_VERSION__ >= 0x00030209
                .mem_flags = 0,
#else
                .address_bits = 0,
#endif
                .extent_order = order,
                .domid        = DOMID_SELF
        };

        set_xen_guest_handle(res.extent_start, frames);
        res.nr_extents = count;

		/* Needs guest frame number */
        int r = HYPERVISOR_memory_op(XENMEM_decrease_reservation, &res);
        return r;
}

/**
 * When we inflate we will be decreasing the memory available to the VM
 * We will give the extent of extent order = order starting at va to the host.
*/
int ukplat_inflate(void* va, int order)
{

	xen_pfn_t pfn = virt_to_pfn(va);
	int r = xenmem_reservation_decrease(1, &pfn, order);

	return r;
}

/**
 * When we deflate we will be increasing the memory available to the VM
 * We will ask for 1 extent of extent order = order back from the host. It will map
 * The extent to the address va.
*/
int ukplat_deflate(void* va, int order)
{
	/* Make sure we are sending the correct frame number. Should be a GFN */
	//xen_pfn_t pfn = virt_to_pfn(va);
	int r = xenmem_reservation_increase(1, &va, order);

	return r;
}

