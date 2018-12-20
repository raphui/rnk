#include <kernel/printk.h>
#include <mm/memory.h>

#ifdef CONFIG_CUSTOM_MALLOC
static void free_mem(void *mem, struct memory_block *heap, void *base)
{
	struct alloc_header *header = (struct alloc_header *)((unsigned int *)mem - sizeof(struct alloc_header));
	unsigned int chunks = header->chunks;
	unsigned int index_block = addr_to_block((void *)header, base);
	unsigned int block_to_free;
	int i;

	if (header->magic != MAGIC) {
		debug_printk("Corrupted or invalid object to be free !\n");
		return;
	}

	if (chunks < CHUNK_PER_BLOCK) {
		heap[index_block].free_mask &= ~(MASK(chunks) << addr_to_chunks_offset((void *)header, base));
		heap[index_block].free_chunks += chunks;
	} else {
		block_to_free = chunks / CHUNK_PER_BLOCK;
		chunks = chunks % CHUNK_PER_BLOCK;

		for (i = 0; i < block_to_free; i++) {
			heap[index_block + i].free_mask = 0;
			heap[index_block + i].free_chunks = CHUNK_PER_BLOCK;
		}

		heap[index_block + block_to_free].free_mask &= ~MASK(chunks);
		heap[index_block + block_to_free].free_chunks += chunks;
	}
}
#endif /* CONFIG_CUSTOM_MALLOC */

void ufree(void *mem)
{
#ifdef CONFIG_CUSTOM_MALLOC
	free_mem(mem, kernel_heap, (void *)KERNEL_HEAP_START);
#elif defined(CONFIG_TLSF)

#ifdef CONFIG_USER
	tlsf_free(tlsf_mem_user_pool, mem);
#else
	tlsf_free(tlsf_mem_kernel_pool, mem);
#endif

#endif
}

void kfree(void *mem)
{
#ifdef CONFIG_CUSTOM_MALLOC
	free_mem(mem, kernel_heap, (void *)KERNEL_HEAP_START);
#elif defined(CONFIG_TLSF)
	tlsf_free(tlsf_mem_kernel_pool, mem);
#endif
}
