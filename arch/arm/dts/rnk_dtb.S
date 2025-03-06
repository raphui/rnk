    .section .rodata
    .global _binary_rnk_dtb_start
    .global _binary_rnk_dtb_end
    .global _binary_rnk_dtb_size

_binary_rnk_dtb_start:
    .incbin "rnk.dtb"
_binary_rnk_dtb_end:

    .equ _binary_rnk_dtb_size, _binary_rnk_dtb_end - _binary_rnk_dtb_start

