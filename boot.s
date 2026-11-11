/* boot.s - Arranque ARM64 (AArch64) con limpieza de BSS */
.section ".text.boot"
.global _start

_start:
    /* Leer el ID del núcleo de la CPU */
    mrs x1, mpidr_el1
    and x1, x1, #3
    cbz x1, core_zero

loop_inf:
    wfe
    b loop_inf

core_zero:
    /* Habilitar acceso a FP/SIMD (CPACR_EL1.FPEN = 0b11).
       El compilador (-O3) genera instrucciones NEON; sin esto cada una
       provoca una excepción (ESR EC=0x07) y el núcleo se queda en bucle. */
    mov x0, #(3 << 20)
    msr cpacr_el1, x0
    isb

    /* Configurar el puntero de pila (SP) antes de _start */
    ldr x1, =_start
    mov sp, x1

    /* Limpiar la sección BSS (necesario para los anillos VirtIO) */
    ldr x1, =__bss_start
    ldr x2, =__bss_end
bss_loop:
    cmp x1, x2
    b.ge bss_done
    str xzr, [x1], #8
    b bss_loop

bss_done:
    /* Saltar al núcleo en C */
    bl kernel_main
    b loop_inf
