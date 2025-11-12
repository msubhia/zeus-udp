
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <iostream>
#include <cstdint>

#define DMA_BASE   0xA0000000
#define DMA_MAP    0x10000
#define BUF_PHYS   (0x77f12000)
#define BUF_SIZE   (128)          // 32 * 4 bytes

using namespace std;

int main() {
    // Map DMA registers
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    void* dma_v = mmap(nullptr, DMA_MAP, PROT_READ|PROT_WRITE, MAP_SHARED, fd, DMA_BASE);
    volatile uint32_t* regs = (volatile uint32_t*)dma_v;

    uint64_t phys = BUF_PHYS;
    void* buf_v = mmap(nullptr, BUF_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, phys);
    volatile uint32_t* buf = (volatile uint32_t*)((char*)buf_v);


    std::cout << "Buffer contents:" << std::endl;
    for (int i = 0; i < BUF_SIZE / 4; i++) {
        std::cout << "  [" << i << "] = " << std::hex << buf[i] << std::dec << std::endl;
    }

    volatile uint32_t* S2MM_DMACR   = regs + 0x30/4;
    volatile uint32_t* S2MM_DMASR   = regs + 0x34/4;
    volatile uint32_t* S2MM_DA      = regs + 0x48/4;
    volatile uint32_t* S2MM_LENGTH  = regs + 0x58/4;

    cout << "control register = " << *S2MM_DMACR << endl;
    cout << "status register = " << *S2MM_DMASR << endl;

    *S2MM_DMACR = 0x0;

    usleep(10000);

    *S2MM_DMACR = 0x1;

    *S2MM_DA     = BUF_PHYS;
    *S2MM_LENGTH = BUF_SIZE;



    // Configure destination and start
    
    cout << "my_physical_addr buf = " << *S2MM_DA << endl;
    

    // Wait until Idle
    while (!(*S2MM_DMASR & (1 << 1))) usleep(100);


    // Print the contents
    std::cout << "Buffer contents:" << std::endl;
    for (int i = 0; i < BUF_SIZE / 4; i++) {
        std::cout << "  [" << i << "] = " <<  buf[i] << std::endl;
    }

    cout << "control register = " << *S2MM_DMACR << endl;
    cout << "status register = " << *S2MM_DMASR << endl;

    munmap(buf_v, BUF_SIZE);
    munmap(dma_v, DMA_MAP);
    close(fd);
}

