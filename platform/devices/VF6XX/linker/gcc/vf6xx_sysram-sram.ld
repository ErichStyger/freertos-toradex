/* Entry Point */
ENTRY(Reset_Handler)

STACK_SIZE = 0x400;
HEAP_SIZE = 0x200;

MEMORY
{
	/*
	 * Note: 0x1f000000 is an alias for code bus to the same memory
	 * located at 0x3f000000, hence start with the data section
	 * at an offset of 256KiB. One can define this section lenghts
	 * freely
	 */
	pc_ram (rwx) : ORIGIN = 0x1f000000, LENGTH = 256K
	ps_ram (rwx) : ORIGIN = 0x3f040000, LENGTH = 256K
}

SECTIONS
{
	.interrupts :
	{
		__VECTOR_TABLE = .;
		. = ALIGN(4);
		KEEP(*(.isr_vector))     /* Startup code */
		. = ALIGN(4);
	} > pc_ram

	.text :
	{
		. = ALIGN(4);
		*(.text)                 /* .text sections (code) */
		*(.text*)                /* .text* sections (code) */
		*(.rodata)               /* .rodata sections (constants, strings, etc.) */
		*(.rodata*)              /* .rodata* sections (constants, strings, etc.) */
		*(.glue_7)               /* glue arm to thumb code */
		*(.glue_7t)              /* glue thumb to arm code */
		*(.eh_frame)
		KEEP (*(.init))
		KEEP (*(.fini))
		. = ALIGN(4);
	} > pc_ram

	.ARM.extab :
	{
		*(.ARM.extab* .gnu.linkonce.armextab.*)
	} > pc_ram

	.ARM :
	{
		__exidx_start = .;
		*(.ARM.exidx*)
		__exidx_end = .;
	} > pc_ram

	.ctors :
	{
		__CTOR_LIST__ = .;
		/* gcc uses crtbegin.o to find the start of
		the constructors, so we make sure it is
		first.  Because this is a wildcard, it
		doesn't matter if the user does not
		actually link against crtbegin.o; the
		linker won't look for a file to match a
		wildcard.  The wildcard also means that it
		doesn't matter which directory crtbegin.o
		is in.  */
		KEEP (*crtbegin.o(.ctors))
		KEEP (*crtbegin?.o(.ctors))
		/* We don't want to include the .ctor section from
		from the crtend.o file until after the sorted ctors.
		The .ctor section from the crtend file contains the
		end of ctors marker and it must be last */
		KEEP (*(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors))
		KEEP (*(SORT(.ctors.*)))
		KEEP (*(.ctors))
		__CTOR_END__ = .;
	} > pc_ram

	.dtors :
	{
		__DTOR_LIST__ = .;
		KEEP (*crtbegin.o(.dtors))
		KEEP (*crtbegin?.o(.dtors))
		KEEP (*(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors))
		KEEP (*(SORT(.dtors.*)))
		KEEP (*(.dtors))
		__DTOR_END__ = .;
	} > pc_ram

	.preinit_array :
	{
		PROVIDE_HIDDEN (__preinit_array_start = .);
		KEEP (*(.preinit_array*))
		PROVIDE_HIDDEN (__preinit_array_end = .);
	} > pc_ram

	.init_array :
	{
		PROVIDE_HIDDEN (__init_array_start = .);
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array*))
		PROVIDE_HIDDEN (__init_array_end = .);
	} > pc_ram

	.fini_array :
	{
		PROVIDE_HIDDEN (__fini_array_start = .);
		KEEP (*(SORT(.fini_array.*)))
		KEEP (*(.fini_array*))
		PROVIDE_HIDDEN (__fini_array_end = .);
	} > pc_ram

	__etext = .;    /* define a global symbol at end of code */
	__DATA_ROM = .; /* Symbol is used by startup for data initialization */

	.data : AT(__DATA_ROM)
	{
		. = ALIGN(4);
		__DATA_RAM = .;
		__data_start__ = .;      /* create a global symbol at data start */
		*(.data)                 /* .data sections */
		*(.data*)                /* .data* sections */
		KEEP(*(.jcr*))
		. = ALIGN(4);
		__data_end__ = .;        /* define a global symbol at data end */
	} > ps_ram

	__DATA_END = __DATA_ROM + (__data_end__ - __data_start__);

	/* Uninitialized data section */
	.bss :
	{
		/* This is used by the startup in order to initialize the .bss section */
		. = ALIGN(4);
		__START_BSS = .;
		__bss_start__ = .;
		*(.bss)
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		__bss_end__ = .;
		__END_BSS = .;
	} > ps_ram

	.heap :
	{
		. = ALIGN(8);
		__end__ = .;
		PROVIDE(end = .);
		__HeapBase = .;
		. += HEAP_SIZE;
		__HeapLimit = .;
	} > ps_ram

	.stack :
	{
		. = ALIGN(8);
		. += STACK_SIZE;
	} > ps_ram

	/* Initializes stack on the end of block */
	__StackTop   = ORIGIN(ps_ram) + LENGTH(ps_ram);
	__StackLimit = __StackTop - STACK_SIZE;
	PROVIDE(__stack = __StackTop);
	.ARM.attributes 0 : { *(.ARM.attributes) }

	ASSERT(__StackLimit >= __HeapLimit, "region m_data overflowed with stack and heap")
}
