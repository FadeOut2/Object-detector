/* Minimal GNU startup for STM32F767 (Cortex-M7, no HAL)
 * - Vector table
 * - Copy .data, zero .bss
 * - Calls SystemInit, then C runtime, then main
 */

    .syntax unified
    .cpu cortex-m7
    .thumb

    .extern SystemInit
    .extern __libc_init_array
    .extern main

    .global g_pfnVectors
    .global Reset_Handler

    .extern _estack
    .extern _sidata
    .extern _sdata
    .extern _edata
    .extern _sbss
    .extern _ebss

/* ===================== Vector Table ===================== */
    .section .isr_vector,"a",%progbits
    .type g_pfnVectors, %object
g_pfnVectors:
    .word _estack               /* 0x00 Initial stack pointer */
    .word Reset_Handler         /* 0x04 Reset */
    .word NMI_Handler           /* 0x08 NMI */
    .word HardFault_Handler     /* 0x0C HardFault */
    .word MemManage_Handler     /* 0x10 MemManage */
    .word BusFault_Handler      /* 0x14 BusFault */
    .word UsageFault_Handler    /* 0x18 UsageFault */
    .word 0                     /* 0x1C Reserved */
    .word 0                     /* 0x20 Reserved */
    .word 0                     /* 0x24 Reserved */
    .word 0                     /* 0x28 Reserved */
    .word SVC_Handler           /* 0x2C SVCall */
    .word DebugMon_Handler      /* 0x30 DebugMonitor */
    .word 0                     /* 0x34 Reserved */
    .word PendSV_Handler        /* 0x38 PendSV */
    .word SysTick_Handler       /* 0x3C SysTick */

    /* Fill remaining external IRQs with Default_Handler */
    .rept 224   /* 240 vectors total - 16 core exceptions */
        .word Default_Handler
    .endr

    .size g_pfnVectors, .-g_pfnVectors

/* ===================== Reset Handler ===================== */
    .section .text.Reset_Handler,"ax",%progbits
    .thumb_func
Reset_Handler:
    /* Copy .data from flash to SRAM */
    ldr   r0, =_sidata
    ldr   r1, =_sdata
    ldr   r2, =_edata
1:
    cmp   r1, r2
    bcc   2f
    b     3f
2:
    ldr   r3, [r0], #4
    str   r3, [r1], #4
    b     1b

3:  /* Zero .bss */
    ldr   r0, =_sbss
    ldr   r1, =_ebss
    movs  r2, #0
4:
    cmp   r0, r1
    bcc   5f
    b     6f
5:
    str   r2, [r0], #4
    b     4b

6:  /* Low-level system init (clocks, etc.) */
    bl    SystemInit

    /* C/C++ static constructors (safe even if none) */
    bl    __libc_init_array

    /* Call main; loop forever on return */
    bl    main
7:
    b     7b

/* ===================== Default Handlers ===================== */
    .section .text.Default_Handler,"ax",%progbits
    .thumb_func
Default_Handler:
    b Default_Handler

    .weak NMI_Handler
    .thumb_set NMI_Handler, Default_Handler

    .weak HardFault_Handler
    .thumb_set HardFault_Handler, Default_Handler

    .weak MemManage_Handler
    .thumb_set MemManage_Handler, Default_Handler

    .weak BusFault_Handler
    .thumb_set BusFault_Handler, Default_Handler

    .weak UsageFault_Handler
    .thumb_set UsageFault_Handler, Default_Handler

    .weak SVC_Handler
    .thumb_set SVC_Handler, Default_Handler

    .weak DebugMon_Handler
    .thumb_set DebugMon_Handler, Default_Handler

    .weak PendSV_Handler
    .thumb_set PendSV_Handler, Default_Handler

    .weak SysTick_Handler
    .thumb_set SysTick_Handler, Default_Handler
