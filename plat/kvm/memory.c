/* SPDX-License-Identifier: ISC */
/* Copyright (c) 2015, IBM
 *           (c) 2017, NEC Europe Ltd.
 * Author(s): Dan Williams <djwillia@us.ibm.com>
 *            Simon Kuenzer <simon.kuenzer@neclab.eu>
 *
 * Permission to use, copy, modify, and/or distribute this software
 * for any purpose with or without fee is hereby granted, provided
 * that the above copyright notice and this permission notice appear
 * in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS
 * OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <uk/plat/common/sections.h>
#include <sys/types.h>
#include <uk/plat/memory.h>
#include <uk/assert.h>
#include <kvm/config.h>

int ukplat_memregion_count(void)
{
	return (9
		+ ((_libkvmplat_cfg.initrd.len > 0) ? 1 : 0)
		+ ((_libkvmplat_cfg.heap2.len  > 0) ? 1 : 0));
}

int ukplat_memregion_get(int i, struct ukplat_memregion_desc *m)
{
	int ret;

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
		ret = 0;
		break;
	case 1: /* eh_frame */
		m->base  = (void *) __EH_FRAME_START;
		m->len   = (size_t) __EH_FRAME_END
				- (size_t) __EH_FRAME_START;
		m->flags = (UKPLAT_MEMRF_RESERVED
			    | UKPLAT_MEMRF_READABLE);
		ret = 0;
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
		ret = 0;
#if CONFIG_UKPLAT_MEMRNAME
		m->name  = "eh_frame_hdr";
#endif
		break;
	case 3: /* rodata */
		m->base  = (void *) __RODATA;
		m->len   = (size_t) __ERODATA - (size_t) __RODATA;
		m->flags = (UKPLAT_MEMRF_RESERVED
			    | UKPLAT_MEMRF_READABLE);
#if CONFIG_UKPLAT_MEMRNAME
		m->name  = "rodata";
#endif
		ret = 0;
		break;
	case 4: /* ctors */
		m->base  = (void *) __CTORS;
		m->len   = (size_t) __ECTORS - (size_t) __CTORS;
		m->flags = (UKPLAT_MEMRF_RESERVED
			    | UKPLAT_MEMRF_READABLE);
#if CONFIG_UKPLAT_MEMRNAME
		m->name  = "ctors";
#endif
		ret = 0;
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
		ret = 0;
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
		ret = 0;
		break;
	case 7: /* heap */
		m->base  = (void *) _libkvmplat_cfg.heap.start;
		m->len   = _libkvmplat_cfg.heap.len;
		m->flags = UKPLAT_MEMRF_ALLOCATABLE;
#if CONFIG_UKPLAT_MEMRNAME
		m->name  = "heap";
#endif
		ret = 0;
		break;
	case 8: /* stack */
		m->base  = (void *) _libkvmplat_cfg.bstack.start;
		m->len   = _libkvmplat_cfg.bstack.len;
		m->flags = (UKPLAT_MEMRF_RESERVED
			    | UKPLAT_MEMRF_READABLE
			    | UKPLAT_MEMRF_WRITABLE);
		ret = 0;
#if CONFIG_UKPLAT_MEMRNAME
		m->name  = "bstack";
#endif
		break;
	case 9: /* initrd */
		if (_libkvmplat_cfg.initrd.len) {
			m->base  = (void *) _libkvmplat_cfg.initrd.start;
			m->len   = _libkvmplat_cfg.initrd.len;
			m->flags = (UKPLAT_MEMRF_INITRD |
				    UKPLAT_MEMRF_WRITABLE);
#if CONFIG_UKPLAT_MEMRNAME
			m->name  = "initrd";
#endif
			ret = 0;
			break;
		}
		/* fall-through */
	case 10: /* heap2
		 *  NOTE: heap2 could only exist if initrd was there,
		 *  otherwise we fall through */
		if (_libkvmplat_cfg.initrd.len && _libkvmplat_cfg.heap2.len) {
			m->base  = (void *) _libkvmplat_cfg.heap2.start;
			m->len   = _libkvmplat_cfg.heap2.len;
			m->flags = UKPLAT_MEMRF_ALLOCATABLE;
#if CONFIG_UKPLAT_MEMRNAME
			m->name  = "heap";
#endif
			ret = 0;
			break;
		}
		/* fall-through */
	default:
		m->base  = __NULL;
		m->len   = 0;
		m->flags = 0x0;
#if CONFIG_UKPLAT_MEMRNAME
		m->name  = __NULL;
#endif
		ret = -1;
		break;
	}

	return ret;
}

int _ukplat_mem_mappings_init(void)
{
	return 0;
}


/* 
 * front-end driver for kvm virtio balloon driver
 * putting relative includes here because should really
 * be in its own file
 */


#include <inttypes.h>
#include <uk/alloc.h>
#include <uk/sglist.h>
#include <uk/list.h>
#include <uk/assert.h>
//#include <uk/mutex.h>
#include <virtio/virtio_ids.h>
#include <virtio/virtio_bus.h>
#include <virtio/virtqueue.h> 

#define DRIVER_NAME "virtio-balloon"
#define VTBALLOON_PAGES_PER_REQUEST	256

static struct uk_alloc *a;

static struct virtio_balloon_device *global_vb;

/* pages given to hypervisor (in the balloon) */
struct balloon_pages { 

	uint32_t num_pages; /* # pages in balloon */

};

/* temporary storage for pages with which we are either
 * inflating or deflating the balloon
 */
struct transport_pages {

	uint32_t num_pages; /* # of pages in pages */
	uint32_t *pages;

};

/* wrapper for virtio device */
struct virtio_balloon_device {

	struct virtio_dev *vdev;

	struct virtqueue *inflate_vq, *deflate_vq;

	__u16 infvq_id;
	__u16 defvq_id;

	char* tag;

	struct balloon_pages *balloon; 

	struct transport_pages *transport;

	uint64_t features;
	uint32_t flags;

	//uk_mutex lock;

};

static void clear_transport(struct virtio_balloon_device* vb)
{
	int num = vb->transport->num_pages;
	for (int i = 0; i < num; i++) {
		(vb->transport->pages)[i] = 0;
		vb->transport->num_pages -= 1;
	}
}

/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2011, Bryan Venteicher <bryanv@FreeBSD.org>
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* The above copyright notice applies only to the below function, 
 * vtballoon_send_page_frames, which is based on FreeBSD's function
 * of the same name.
 */

static void vtballoon_send_page_frames(struct virtio_balloon_device *vb, 
			struct virtqueue *vq, int npages)
{
	
	struct uk_sglist sg;
	struct uk_sglist_seg segs[1];
	int c;

	uk_sglist_init(&sg, 1, segs);

	uk_sglist_append(&sg, vb->transport->pages, npages * sizeof(uint32_t));

	void* vq_cookie;
	virtqueue_buffer_enqueue(vq, &vq_cookie, &sg, 1, 0);
	
	virtqueue_host_notify(vq);
	
	__u32 len = 0;
	/* wait on KVM to respond. Need a safer method for this */
	while ((c = virtqueue_buffer_dequeue(vq, &vq_cookie, &len)) < 0);
	
}

/**
 * this is equivalent to leaking from the balloon and 
 * increasing memory reservation for guest
 */
static int deflate_balloon(uintptr_t *pages_to_guest, uint32_t num)
{
	/* get pages_to_guest from the balloon and tell host we are using them now */

	/* check if device is ready */
	if (!global_vb)
		return -ENXIO;

	struct virtio_balloon_device *vb = global_vb;

	//uk_mutex_lock(vb->lock);

	clear_transport(vb);
	
	if (vb->balloon->num_pages < num) num = vb->balloon->num_pages;
	
	for (uint32_t i = 0; i < num; i++) {
		uint32_t page = pages_to_guest[i];
		vb->transport->pages[i] = page; /* put page in temp array for host */
		vb->balloon->num_pages -= 1;
		vb->transport->num_pages += 1;
	}

	int num_pages_taken = vb->transport->num_pages;
	
	if (vb->transport->num_pages != 0) {
		vtballoon_send_page_frames(vb, vb->deflate_vq, vb->transport->num_pages);
	} 
	
	//uk_mutex_unlock(vb->lock);

	return num_pages_taken;
}

/**
 * this is equivalent to filling the balloon and 
 * decreasing memory reservation for guest
 */
static int inflate_balloon(uintptr_t *pages_to_host, uint32_t num)
{
	/* need to put pages_to_host in the balloon for the host to use */

	/* check if device is ready */
	if (!global_vb)
		return -ENXIO;

	struct virtio_balloon_device *vb = global_vb;

	//uk_mutex_lock(vb->lock);

	clear_transport(vb);
	
	for (uint32_t i = 0; i < num; i++) {
		uint32_t page = pages_to_host[i] / __PAGE_SIZE;
		vb->transport->pages[i] = page; /* put page in temp array for host */
		vb->balloon->num_pages += 1;
		vb->transport->num_pages += 1;
	}

	int num_pages_given = vb->transport->num_pages;

	if (vb->transport->num_pages != 0) {
		vtballoon_send_page_frames(vb, vb->inflate_vq, vb->transport->num_pages);
	} 
	
	//uk_mutex_unlock(vb->lock);

	return num_pages_given;
}


static inline void virtio_balloon_feature_set(struct virtio_balloon_device *vb)
{
	vb->features = 0;
	vb->flags = 0;
	vb->vdev->features = 0;
}

static int virtio_balloon_vq_alloc(struct virtio_balloon_device *vb)
{
	int vq_avail = 0;
	int rc = 0;
	__u16 qdesc_size[2];

	vq_avail = virtio_find_vqs(vb->vdev, 2, &(qdesc_size[0])); 
	if (unlikely(vq_avail != 2)) {
		uk_pr_err(DRIVER_NAME": Expected: %d queues, found %d\n",
			  2, vq_avail);
		rc = -ENOMEM;
		goto exit;
	}

	vb->infvq_id = 0;
	vb->defvq_id = 1;

	vb->inflate_vq = virtio_vqueue_setup(
						vb->vdev, vb->infvq_id, qdesc_size[0], NULL, a); /* no callback */
	vb->inflate_vq->priv = vb;

	if (unlikely(PTRISERR(vb->inflate_vq))) {
		uk_pr_err(DRIVER_NAME": Failed to set up virtqueue %"PRIu16"\n",
			vb->infvq_id);
		rc = PTR2ERR(vb->inflate_vq);
	}
	
	vb->deflate_vq = virtio_vqueue_setup(
						vb->vdev, vb->defvq_id, qdesc_size[1], NULL, a); /* no callback */
	vb->deflate_vq->priv = vb;

	if (unlikely(PTRISERR(vb->deflate_vq))) {
		uk_pr_err(DRIVER_NAME": Failed to set up virtqueue %"PRIu16"\n",
			vb->defvq_id);
		rc = PTR2ERR(vb->deflate_vq);
	}

exit:
	return rc;
}
	
static int virtio_balloon_start(struct virtio_balloon_device *vb)
{

	virtqueue_intr_enable(vb->inflate_vq);
	virtqueue_intr_enable(vb->deflate_vq);
	virtio_dev_drv_up(vb->vdev);
	uk_pr_info(DRIVER_NAME": %s started\n", vb->tag);

	return 0;
}

static int virtio_balloon_add_dev(struct virtio_dev *vdev)
{

	struct virtio_balloon_device *vbdev;
	int rc = 0;

	UK_ASSERT(vdev != NULL);
	
	vbdev = uk_calloc(a, 1, sizeof(*vbdev));

	if (!vbdev) {
		rc = -ENOMEM;
		goto err_out;
	}

	int tag_len = 30;
	vbdev->tag = uk_calloc(a, 1, sizeof(tag_len));
	vbdev->tag = "VIRTIO_BALLOON_DRV_DEV";

	//uk_mutex_init(&vbdev->lock);

	vbdev->vdev = vdev;
	virtio_balloon_feature_set(vbdev); 
	rc = virtio_balloon_vq_alloc(vbdev);
	if (rc) {
		goto err_out;
	}

	vbdev->transport = uk_calloc(a, 1, sizeof(struct transport_pages));
	if (!(vbdev->transport)) {
		rc = -ENOMEM;
		goto err_out;
	}
	vbdev->transport->pages = 
				uk_calloc(a, 1, VTBALLOON_PAGES_PER_REQUEST * sizeof(uint32_t));
	if (!(vbdev->transport->pages)) {
		rc = -ENOMEM;
		goto err_out;
	}
	vbdev->balloon = uk_calloc(a, 1, sizeof(struct balloon_pages));
	
	rc = virtio_balloon_start(vbdev);
	if (rc) {
		goto err_out;
	}

exit:
	global_vb = vbdev; /* initialize global vb */
	/* initial alloc and free to trigger ballon init */
	void* alc = uk_palloc(a, 0);
	uk_pfree(a, alc, 0);
	return rc;
err_out:
	uk_free(a, vbdev->transport->pages);
	uk_free(a, vbdev->transport);
	uk_free(a, vbdev->balloon);
	uk_free(a, vbdev);
	goto exit;

}

static int virtio_balloon_drv_init(struct uk_alloc *drv_allocator)
{
	/* driver initialization */
	if (!drv_allocator) {
		return -EINVAL;
	}

	a = drv_allocator;
	return 0;

}

static const struct virtio_dev_id vballoon_dev_id[] = {
	{VIRTIO_ID_BALLOON},
	{VIRTIO_ID_INVALID} /* List Terminator */
};

static struct virtio_driver virtio_balloon_driver = {
	.dev_ids = vballoon_dev_id,
	.init	 = virtio_balloon_drv_init,
	.add_dev = virtio_balloon_add_dev
};
VIRTIO_BUS_REGISTER_DRIVER(&virtio_balloon_driver);


/* memory.c inflation and deflation API function implementations: */

/**
 * number of pages is 2^order
 */
int get_num_pages(int order)
{
	int num_pages = 1;
	for (int i = 0; i < order; i++) num_pages *= 2;
	return num_pages;
}

/**
 * fill addresses for page range starting at first_page
 */
void fill_page_array(uintptr_t *pages_array, void* first_page, int num_pages)
{	
	uint64_t current_pg = (uint64_t) first_page;
	for (int i = 0; i < num_pages; i++) {
		pages_array[i] = current_pg;
		current_pg += __PAGE_SIZE;
	}
}

/**
 * call driver inflate_balloon
 * returns number of pages actually put into balloon or < 0 on error
 */
int ukplat_inflate(void* page, int order)
{
	int num_pages = get_num_pages(order);
	uintptr_t pages_to_host[num_pages];
	fill_page_array(pages_to_host, page, num_pages);
	
	return inflate_balloon(pages_to_host, num_pages);
}

/**
 * call driver deflate_balloon
 * returns number of pages actually taken from balloon or < 0 on error
 */
int ukplat_deflate(void* page, int order)
{
	int num_pages = get_num_pages(order);
	uintptr_t pages_to_guest[num_pages];
	fill_page_array(pages_to_guest, page, num_pages);

	return deflate_balloon(pages_to_guest, num_pages);
}
