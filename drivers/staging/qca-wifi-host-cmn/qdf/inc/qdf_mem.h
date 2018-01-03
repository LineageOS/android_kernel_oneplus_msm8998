/*
 * Copyright (c) 2014-2017 The Linux Foundation. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
 *
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * This file was originally distributed by Qualcomm Atheros, Inc.
 * under proprietary terms before Copyright ownership was assigned
 * to the Linux Foundation.
 */

/**
 * DOC: qdf_mem
 * QCA driver framework (QDF) memory management APIs
 */

#if !defined(__QDF_MEMORY_H)
#define __QDF_MEMORY_H

/* Include Files */
#include <qdf_types.h>
#include <i_qdf_mem.h>

#define QDF_CACHE_LINE_SZ __qdf_cache_line_sz

/**
 * struct qdf_mem_dma_page_t - Allocated dmaable page
 * @page_v_addr_start: Page start virtual address
 * @page_v_addr_end: Page end virtual address
 * @page_p_addr: Page start physical address
 */
struct qdf_mem_dma_page_t {
	char *page_v_addr_start;
	char *page_v_addr_end;
	qdf_dma_addr_t page_p_addr;
};

/**
 * struct qdf_mem_multi_page_t - multiple page allocation information storage
 * @num_element_per_page: Number of element in single page
 * @num_pages: Number of allocation needed pages
 * @dma_pages: page information storage in case of coherent memory
 * @cacheable_pages: page information storage in case of cacheable memory
 */
struct qdf_mem_multi_page_t {
	uint16_t num_element_per_page;
	uint16_t num_pages;
	struct qdf_mem_dma_page_t *dma_pages;
	void **cacheable_pages;
};


/* Preprocessor definitions and constants */

typedef __qdf_mempool_t qdf_mempool_t;

/**
 * qdf_mem_init() - Initialize QDF memory module
 *
 * Return: None
 *
 */
void qdf_mem_init(void);

/**
 * qdf_mem_exit() - Exit QDF memory module
 *
 * Return: None
 *
 */
void qdf_mem_exit(void);

#ifdef MEMORY_DEBUG
#define qdf_mem_malloc(size) \
	qdf_mem_malloc_debug(size, __FILE__, __LINE__)
void *qdf_mem_malloc_debug(size_t size, char *file_name, uint32_t line_num);
#else
void *
qdf_mem_malloc(qdf_size_t size);
#endif

void *qdf_mem_alloc_outline(qdf_device_t osdev, qdf_size_t size);

/**
 * qdf_mem_free() - free QDF memory
 * @ptr: Pointer to the starting address of the memory to be free'd.
 * This function will free the memory pointed to by 'ptr'.
 * Return:
 * None
 */
void qdf_mem_free(void *ptr);

void qdf_mem_set(void *ptr, uint32_t num_bytes, uint32_t value);

void qdf_mem_zero(void *ptr, uint32_t num_bytes);

void qdf_mem_copy(void *dst_addr, const void *src_addr, uint32_t num_bytes);

void qdf_mem_move(void *dst_addr, const void *src_addr, uint32_t num_bytes);

void qdf_mem_free_outline(void *buf);

void *qdf_mem_alloc_consistent(qdf_device_t osdev, void *dev, qdf_size_t size,
			       qdf_dma_addr_t *paddr);

void qdf_mem_free_consistent(qdf_device_t osdev, void *dev, qdf_size_t size,
			     void *vaddr, qdf_dma_addr_t paddr,
			     qdf_dma_context_t memctx);

void qdf_mem_zero_outline(void *buf, qdf_size_t size);

/**
 * qdf_mem_cmp() - memory compare
 * @memory1: pointer to one location in memory to compare.
 * @memory2: pointer to second location in memory to compare.
 * @num_bytes: the number of bytes to compare.
 *
 * Function to compare two pieces of memory, similar to memcmp function
 * in standard C.
 * Return:
 * int32_t - returns an int value that tells if the memory
 * locations are equal or not equal.
 * 0 -- equal
 * < 0 -- *memory1 is less than *memory2
 * > 0 -- *memory1 is bigger than *memory2
 */
static inline int32_t qdf_mem_cmp(const void *memory1, const void *memory2,
				  uint32_t num_bytes)
{
	return __qdf_mem_cmp(memory1, memory2, num_bytes);
}

/**
 * qdf_str_cmp - Compare two strings
 * @str1: First string
 * @str2: Second string
 * Return: =0 equal
 * >0    not equal, if  str1  sorts lexicographically after str2
 * <0    not equal, if  str1  sorts lexicographically before str2
 */
static inline int32_t qdf_str_cmp(const char *str1, const char *str2)
{
	return __qdf_str_cmp(str1, str2);
}

/**
 * qdf_str_lcopy - Copy from one string to another
 * @dest: destination string
 * @src: source string
 * @bytes: limit of num bytes to copy
 * Return: =0 returns the initial value of dest
 */
static inline uint32_t qdf_str_lcopy(char *dest, const char *src, uint32_t bytes)
{
	return __qdf_str_lcopy(dest, src, bytes);
}

/**
 * qdf_mem_map_nbytes_single - Map memory for DMA
 * @osdev: pomter OS device context
 * @buf: pointer to memory to be dma mapped
 * @dir: DMA map direction
 * @nbytes: number of bytes to be mapped.
 * @phy_addr: ponter to recive physical address.
 *
 * Return: success/failure
 */
static inline uint32_t qdf_mem_map_nbytes_single(qdf_device_t osdev, void *buf,
						 qdf_dma_dir_t dir, int nbytes,
						 uint32_t *phy_addr)
{
#if defined(HIF_PCI)
	return __qdf_mem_map_nbytes_single(osdev, buf, dir, nbytes, phy_addr);
#else
	return 0;
#endif
}

/**
 * qdf_mem_unmap_nbytes_single() - un_map memory for DMA
 * @osdev: pomter OS device context
 * @phy_addr: physical address of memory to be dma unmapped
 * @dir: DMA unmap direction
 * @nbytes: number of bytes to be unmapped.
 *
 * Return: none
 */
static inline void qdf_mem_unmap_nbytes_single(qdf_device_t osdev,
					       uint32_t phy_addr,
					       qdf_dma_dir_t dir,
					       int nbytes)
{
#if defined(HIF_PCI)
	__qdf_mem_unmap_nbytes_single(osdev, phy_addr, dir, nbytes);
#endif
}

/**
 * qdf_mempool_init - Create and initialize memory pool
 * @osdev: platform device object
 * @pool_addr: address of the pool created
 * @elem_cnt: no. of elements in pool
 * @elem_size: size of each pool element in bytes
 * @flags: flags
 * Return: Handle to memory pool or NULL if allocation failed
 */
static inline int qdf_mempool_init(qdf_device_t osdev,
				   qdf_mempool_t *pool_addr, int elem_cnt,
				   size_t elem_size, uint32_t flags)
{
	return __qdf_mempool_init(osdev, pool_addr, elem_cnt, elem_size,
				  flags);
}

/**
 * qdf_mempool_destroy - Destroy memory pool
 * @osdev: platform device object
 * @Handle: to memory pool
 * Return: none
 */
static inline void qdf_mempool_destroy(qdf_device_t osdev, qdf_mempool_t pool)
{
	__qdf_mempool_destroy(osdev, pool);
}

/**
 * qdf_mempool_alloc - Allocate an element memory pool
 * @osdev: platform device object
 * @Handle: to memory pool
 * Return: Pointer to the allocated element or NULL if the pool is empty
 */
static inline void *qdf_mempool_alloc(qdf_device_t osdev, qdf_mempool_t pool)
{
	return (void *)__qdf_mempool_alloc(osdev, pool);
}

/**
 * qdf_mempool_free - Free a memory pool element
 * @osdev: Platform device object
 * @pool: Handle to memory pool
 * @buf: Element to be freed
 * Return: none
 */
static inline void qdf_mempool_free(qdf_device_t osdev, qdf_mempool_t pool,
				    void *buf)
{
	__qdf_mempool_free(osdev, pool, buf);
}

void qdf_mem_dma_sync_single_for_device(qdf_device_t osdev,
					qdf_dma_addr_t bus_addr,
					qdf_size_t size,
					qdf_dma_dir_t direction);

void qdf_mem_dma_sync_single_for_cpu(qdf_device_t osdev,
					qdf_dma_addr_t bus_addr,
					qdf_size_t size,
					qdf_dma_dir_t direction);
/**
 * qdf_str_len() - returns the length of a string
 * @str: input string
 * Return:
 * length of string
 */
static inline int32_t qdf_str_len(const char *str)
{
	return __qdf_str_len(str);
}

void qdf_mem_multi_pages_alloc(qdf_device_t osdev,
			       struct qdf_mem_multi_page_t *pages,
			       size_t element_size, uint16_t element_num,
			       qdf_dma_context_t memctxt, bool cacheable);
void qdf_mem_multi_pages_free(qdf_device_t osdev,
			      struct qdf_mem_multi_page_t *pages,
			      qdf_dma_context_t memctxt, bool cacheable);

/**
 * qdf_mem_skb_inc() - increment total skb allocation size
 * @size: size to be added
 *
 * Return: none
 */
void qdf_mem_skb_inc(qdf_size_t size);

/**
 * qdf_mem_skb_dec() - decrement total skb allocation size
 * @size: size to be decremented
 *
 * Return: none
 */
void qdf_mem_skb_dec(qdf_size_t size);

/**
 * qdf_mem_map_table_alloc() - Allocate shared memory info structure
 * @num: number of required storage
 *
 * Allocate mapping table for DMA memory allocation. This is needed for
 * IPA-WLAN buffer sharing when SMMU Stage1 Translation is enabled.
 *
 * Return: shared memory info storage table pointer
 */
static inline qdf_mem_info_t *qdf_mem_map_table_alloc(uint32_t num)
{
	qdf_mem_info_t *mem_info_arr;

	mem_info_arr = qdf_mem_malloc(num * sizeof(qdf_mem_info_t));
	return mem_info_arr;
}

/**
 * qdf_update_mem_map_table() - Update DMA memory map info
 * @osdev: Parent device instance
 * @mem_info: Pointer to shared memory information
 * @dma_addr: dma address
 * @mem_size: memory size allocated
 *
 * Store DMA shared memory information
 *
 * Return: none
 */
static inline void qdf_update_mem_map_table(qdf_device_t osdev,
					    qdf_mem_info_t *mem_info,
					    qdf_dma_addr_t dma_addr,
					    uint32_t mem_size)
{
	if (!mem_info) {
		__qdf_print("%s: NULL mem_info\n", __func__);
		return;
	}

	mem_info->pa = __qdf_mem_paddr_from_dmaaddr(osdev, dma_addr);
	mem_info->iova = dma_addr;
	mem_info->size = mem_size;
}

/**
 * qdf_mem_smmu_s1_enabled() - Return SMMU stage 1 translation enable status
 * @osdev parent device instance
 *
 * @Return: true if smmu s1 enabled, false if smmu s1 is bypassed
 */
static inline bool qdf_mem_smmu_s1_enabled(qdf_device_t osdev)
{
	return __qdf_mem_smmu_s1_enabled(osdev);
}

/**
 * qdf_mem_paddr_from_dmaaddr() - get actual physical address from dma address
 * @osdev: Parent device instance
 * @dma_addr: DMA/IOVA address
 *
 * Get actual physical address from dma_addr based on SMMU enablement status.
 * IF SMMU Stage 1 tranlation is enabled, DMA APIs return IO virtual address
 * (IOVA) otherwise returns physical address. So get SMMU physical address
 * mapping from IOVA.
 *
 * Return: dmaable physical address
 */
static inline qdf_dma_addr_t qdf_mem_paddr_from_dmaaddr(qdf_device_t osdev,
							qdf_dma_addr_t dma_addr)
{
	return __qdf_mem_paddr_from_dmaaddr(osdev, dma_addr);
}

/**
 * qdf_mem_dma_get_sgtable() - Returns DMA memory scatter gather table
 * @dev: device instace
 * @sgt: scatter gather table pointer
 * @cpu_addr: HLOS virtual address
 * @dma_addr: dma address
 * @size: allocated memory size
 *
 * @Return: status
 */
static inline int
qdf_mem_dma_get_sgtable(struct device *dev, void *sgt, void *cpu_addr,
			qdf_dma_addr_t dma_addr, size_t size)
{
	return __qdf_os_mem_dma_get_sgtable(dev, sgt, cpu_addr, dma_addr, size);
}

/**
 * qdf_dma_get_sgtable_dma_addr() - Assigns DMA address to scatterlist elements
 * @sgt: scatter gather table pointer
 *
 * @Return: None
 */
static inline void
qdf_dma_get_sgtable_dma_addr(struct sg_table *sgt)
{
	return __qdf_dma_get_sgtable_dma_addr(sgt);
}

/**
 * qdf_mem_get_dma_addr() - Return dma address based on SMMU translation status.
 * @osdev: Parent device instance
 * @mem_info: Pointer to allocated memory information
 *
 * Get dma address based on SMMU enablement status. If SMMU Stage 1
 * tranlation is enabled, DMA APIs return IO virtual address otherwise
 * returns physical address.
 *
 * @Return: dma address
 */
static inline qdf_dma_addr_t qdf_mem_get_dma_addr(qdf_device_t osdev,
						  qdf_mem_info_t *mem_info)
{
	return __qdf_mem_get_dma_addr(osdev, mem_info);
}

/**
 * qdf_mem_get_dma_addr_ptr() - Return DMA address pointer from mem info struct
 * @osdev: Parent device instance
 * @mem_info: Pointer to allocated memory information
 *
 * Based on smmu stage 1 translation enablement, return corresponding dma
 * address storage pointer.
 *
 * @Return: dma address storage pointer
 */
static inline qdf_dma_addr_t *qdf_mem_get_dma_addr_ptr(qdf_device_t osdev,
						       qdf_mem_info_t *mem_info)
{
	return __qdf_mem_get_dma_addr_ptr(osdev, mem_info);
}

/**
 * qdf_mem_shared_mem_alloc() - Allocate DMA memory for shared resource
 * @osdev: parent device instance
 * @mem_info: Pointer to allocated memory information
 * @size: size to be allocated
 *
 * Allocate DMA memory which will be shared with external kernel module. This
 * information is needed for SMMU mapping.
 *
 * @Return: 0 suceess
 */
static inline qdf_shared_mem_t *qdf_mem_shared_mem_alloc(qdf_device_t osdev,
							 uint32_t size)
{
	qdf_shared_mem_t *shared_mem;

	shared_mem = qdf_mem_malloc(sizeof(qdf_shared_mem_t));
	if (!shared_mem) {
		__qdf_print("%s: Unable to allocate memory for shared resource struct\n",
			    __func__);
		return NULL;
	}

	shared_mem->vaddr = qdf_mem_alloc_consistent(osdev, osdev->dev, size,
				(qdf_dma_addr_t *)&shared_mem->mem_info.iova);
	if (!shared_mem->vaddr) {
		__qdf_print("%s; Unable to allocate DMA memory for shared resource\n",
			    __func__);
		qdf_mem_free(shared_mem);
		return NULL;
	}

	shared_mem->mem_info.size = size;
	qdf_mem_zero(shared_mem->vaddr, shared_mem->mem_info.size);
	shared_mem->mem_info.pa = __qdf_mem_paddr_from_dmaaddr(osdev,
				    (qdf_dma_addr_t)shared_mem->mem_info.iova);
	qdf_mem_dma_get_sgtable(osdev->dev,
				(void *)&shared_mem->sgtable,
				shared_mem->vaddr,
				qdf_mem_get_dma_addr(osdev,
						     &shared_mem->mem_info),
				shared_mem->mem_info.size);

	qdf_dma_get_sgtable_dma_addr(&shared_mem->sgtable);

	return shared_mem;
}

/**
 * qdf_mem_shared_mem_free() - Free shared memory
 * @osdev: parent device instance
 * @shared_mem: shared memory information storage
 *
 * Free DMA shared memory resource
 *
 * @Return: None
 */
static inline void qdf_mem_shared_mem_free(qdf_device_t osdev,
					   qdf_shared_mem_t *shared_mem)
{
	if (!shared_mem) {
		__qdf_print("%s: NULL shared mem struct passed\n",
			    __func__);
		return;
	}

	if (shared_mem->vaddr) {
		qdf_mem_free_consistent(osdev, osdev->dev,
					shared_mem->mem_info.size,
					shared_mem->vaddr,
					qdf_mem_get_dma_addr(osdev,
						&shared_mem->mem_info),
					qdf_get_dma_mem_context(shared_mem,
								memctx));
	}
	qdf_mem_free(shared_mem);
}

#endif /* __QDF_MEMORY_H */
