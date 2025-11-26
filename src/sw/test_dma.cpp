#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <iostream>
#include <cstdint>

#define DEVICE_MEMORY_PATH  "/dev/mem"


class DMA_Driver{
public:
    const uint32_t AXI_DMA_BASE_ADDRESS                 = 0xA0000000;
    const uint32_t AXI_DMA_HIGH_ADDRESS                 = 0xA000FFFF;

    // Memory Map to Stream Register Detail
    const uint32_t AXI_DMA_MM2S_CONTROL_REG             = 0x00;
    const uint32_t AXI_DMA_MM2S_STATUS_REG              = 0x04;
    const uint32_t AXI_DMA_MM2S_SRC_ADDR_LOWER32        = 0x18;
    const uint32_t AXI_DMA_MM2S_SRC_ADDR_UPPER32        = 0x1C;
    const uint32_t AXI_DMA_MM2S_TRANSFER_LENGTH_BYTES   = 0x28;

    // Stream to Memory Map Register Detail
    const uint32_t AXI_DMA_S2MM_CONTROL_REG             = 0x30;
    const uint32_t AXI_DMA_S2MM_STATUS_REG              = 0x34;
    const uint32_t AXI_DMA_MM2S_DST_ADDR_LOWER32        = 0x48;
    const uint32_t AXI_DMA_MM2S_DST_ADDR_UPPER32        = 0x4C;
    const uint32_t AXI_DMA_MM2S_TRANSFER_LENGTH_BYTES   = 0x58;

    DMA_Driver(){
        fd          = open( DEVICE_MEMORY_PATH, O_RDWR | O_SYNC);
        dma_virt    = mmap( nullptr,
                            AXI_DMA_HIGH_ADDRESS - AXI_DMA_BASE_ADDRESS, 
                            PROT_READ|PROT_WRITE, 
                            MAP_SHARED, 
                            fd, 
                            AXI_DMA_BASE_ADDRESS);
        dma_regs = (volatile uint32_t*) dma_virt;
        
    };

    ~DMA_Driver(){
        close(fd);
        munmap(dma_virt, AXI_DMA_HIGH_ADDRESS - AXI_DMA_BASE_ADDRESS);
    };

    void set_reg(uint32_t offset, uint32_t val){
        volatile uint32_t* my_reg = this->dma_regs + (offset>>2);
        *my_reg = val;
    }

    uint32_t get_reg(uint32_t offset){
        volatile uint32_t* my_reg = this->dma_regs + (offset>>2);
        return *my_reg;
    }

    void reset(){
        // set_reg(AXI_DMA_MM2S_CONTROL_REG)
    }


private:
    int fd;
    void* dma_virt;
    volatile uint32_t* dma_regs;
};







int main() {
    // Map DMA registers
    int fd      = open("/dev/mem", O_RDWR | O_SYNC);
    void* dma_v = mmap(nullptr, DMA_MAP, PROT_READ|PROT_WRITE, MAP_SHARED, fd, DMA_BASE);
    volatile uint32_t* regs = (volatile uint32_t*)dma_v;

    uint64_t phys = BUF_PHYS;
    uint64_t page = phys & ~0xFFFULL;
    size_t off = phys & 0xFFFULL;
    // Map the destination buffer itself
    void* buf_v = mmap(nullptr, BUF_SIZE + off, PROT_READ|PROT_WRITE, MAP_SHARED, fd, page);
    volatile uint32_t* buf = (volatile uint32_t*)((char*)buf_v + off);


    std::cout << "Buffer contents:" << std::endl;
    for (int i = 0; i < BUF_SIZE / 4; i++) {
    std::cout << " [" << i << "] = 0x" << std::hex << buf[i] << std::dec << std::endl;
    }

    volatile uint32_t* S2MM_DMACR = regs + 0x30/4;
    volatile uint32_t* S2MM_DMASR = regs + 0x34/4;
    volatile uint32_t* S2MM_DA = regs + 0x48/4;
    volatile uint32_t* S2MM_LENGTH = regs + 0x58/4;

    // Reset + start DMA
    *S2MM_DMACR = 0x4;
    usleep(1000);
    *S2MM_DMACR = 0x1;

    // Configure destination and start
    *S2MM_DA = BUF_PHYS;
    *S2MM_LENGTH = BUF_SIZE - 4;

    // Wait until Idle
    while (!(*S2MM_DMASR & (1 << 1))) usleep(100);


    // Print the contents
    std::cout << "Buffer contents:" << std::endl;
    for (int i = 0; i < BUF_SIZE / 4; i++) {
    std::cout << " [" << i << "] = 0x" << buf[i] << std::endl;
    }

    munmap(buf_v, BUF_SIZE);
    munmap(dma_v, DMA_MAP);
    close(fd);
}