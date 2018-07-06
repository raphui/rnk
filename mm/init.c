#include <init.h>
#include <mm/memory.h>

int heap_size = MAX_KERNEL_HEAP_SIZE;

int heap_init(void)
{
	int ret = 0;
#ifdef CONFIG_CUSTOM_MALLOC
	int i;

	for (i = 0; i < KERNEL_NUM_BLOCKS; i++) {
		kernel_heap[i].free_chunks = CHUNK_PER_BLOCK;
		kernel_heap[i].free_mask = 0;
	}
#elif defined(CONFIG_TLSF)
	tlsf_mem_kernel_pool = tlsf_create_with_pool((void *)KERNEL_HEAP_START, MAX_KERNEL_HEAP_SIZE);
	tlsf_mem_user_pool = tlsf_create_with_pool((void *)USER_HEAP_START, MAX_USER_HEAP_SIZE);
#endif

	return ret;
}
pure_initcall(heap_init);
