#!/bin/bash
set -e

echo "=== Verificando entorno en macOS ==="
if ! command -v aarch64-elf-gcc &> /dev/null || ! command -v qemu-system-aarch64 &> /dev/null; then
    echo "Instalando aarch64-elf-gcc, binutils y qemu con Homebrew..."
    brew install aarch64-elf-gcc aarch64-elf-binutils qemu
fi

echo "=== Compilando Unikernel ARM64 + VirtIO-Net ==="
aarch64-elf-as boot.s -o boot.o
# -mstrict-align: sin MMU la memoria es Device y los accesos no alineados provocan Data Abort
aarch64-elf-gcc -c kernel.c -o kernel.o -ffreestanding -O3 -Wall -mstrict-align
aarch64-elf-ld -T linker.ld -o unikernel_http.elf boot.o kernel.o -nostdlib

echo "=== Arrancando Servidor Web Unikernel en QEMU ==="
echo "-> Abre tu navegador o ejecuta en otra terminal: curl -i http://localhost:18080"
echo "-> Para salir de QEMU: pulsa Ctrl+A y luego X"
echo ""

ACCEL_OPTS="-cpu cortex-a72"
if [[ "$(uname -m)" == "arm64" ]]; then
    ACCEL_OPTS="-cpu host -accel hvf"
fi

# Configuramos VirtIO-Net MMIO (force-legacy=on) y redirigimos el puerto 18080 de macOS al 18080 del Unikernel
qemu-system-aarch64 \
    -M virt \
    $ACCEL_OPTS \
    -m 128M \
    -global virtio-mmio.force-legacy=on \
    -netdev user,id=net0,hostfwd=tcp::18080-:18080 \
    -device virtio-net-device,netdev=net0 \
    -kernel unikernel_http.elf \
    -nographic
