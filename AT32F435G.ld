MEMORY {
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 384K
}
ENTRY(Reset_Handler)
_estack = ORIGIN(RAM) + LENGTH(RAM);
SECTIONS {
    .isr_vector : {
        KEEP(*(.isr_vector))
    } > FLASH
    .text : { *(.text .text*) *(.rodata*) } > FLASH
    .data : { _sdata = .; *(.data) *(.data*) . = ALIGN(4); _edata = .; } > RAM AT> FLASH
    _sidata = LOADADDR(.data);
    .bss : { _sbss = .; *(.bss) *(COMMON) . = ALIGN(4); _ebss = .; } > RAM
}