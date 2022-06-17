#include "common.hh"
#include "mem.hh"

#include <syscall.h>
#include <arpa/inet.h>

#include <libsyscall_intercept_hook_point.h>

// Temp.
#include <cassert>
#include <cstdio>

#include <fcntl.h>
#include <sys/mman.h>

#include <poll.h>
#include <sys/epoll.h>

#include <x86intrin.h>
// Temp. END

#define NUM_SYSCALLS 449


#pragma once

#if EAGAIN != EWOULDBLOCK
#error EAGAIN != EWOULDBLOCK
#endif

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/epoll.h>

#define ENVSTR_GLOBAL_IVSHMEM_INTR_DEV_PATH "GLOBAL_IVSHMEM_INTR_DEV_PATH"

// Use this type for pointers only!
typedef struct __attribute__((aligned(256), packed)) ivshmem_intr_mmio
{
    uint32_t IntrMask;   // useless
    uint32_t IntrStatus; // useless
    uint32_t IVPosition; // for reading dest. intr. ID of this VM
    uint32_t Doorbell;   // for writing intr. msg.
    uint32_t IVLiveList; // useless
} IVSHMEM_INTR_MMIO;     // sizeof(IVSHMEM_INTR_MMIO_REG) == 256 for IVSHMEM-Doorbell virtual PCIe BAR0 register region

uint32_t global_intr_dest_id;
typedef uint32_t ivshmem_intr_req_t;
typedef uint32_t ivshmem_intr_resp_t;
#define IVSHMEM_INTR_UIO_IRQ 0
#define IVSHMEM_INTR_MSG(dest, irq) (ivshmem_intr_req_t)(((uint32_t)dest << 16)) // IVSHMEM-Doorbell UIO driver only supports single IRQ (which is 0).

// Use this type for global variables only!
typedef struct __attribute__((aligned(16), packed)) ivshmem_intr_dev
{
    struct epoll_event dump; // 16-byte; dump anything here (useless)
    IVSHMEM_INTR_MMIO *mmio; // 8-byte; for sending interrupt through MMIO
    int fd;                  // 4-byte; for device file
    int epollfd;             // 4-byte; for edge-triggered epoll_wait() if you don't need the interrupt value
} IVSHMEM_INTR_DEV;          // sizeof(IVSHMEM_INTR_FD) == 32; This should be aligned to 256-bit (32-byte) for better performance.

// Currently reading file desc. of IVSHMEM-Doorbell dev. returns only global interrupt receive count, as QEMU/KVM "doorbell" model fashion, which is useless.
// Also, the device destination IDs (specified in "iv_position" on each VM) are the only possible target for sending interrupt, just use global intr. file desc. for now...
extern IVSHMEM_INTR_DEV global_ivshmem_intr_dev;

static inline __attribute__((always_inline, regparm(1))) void global_ivshmem_intr_mmio_write(const uint32_t dest_addr)
{
    global_ivshmem_intr_dev.mmio->Doorbell = IVSHMEM_INTR_MSG(dest_addr, IVSHMEM_INTR_UIO_IRQ);
}

// Edge-trigger (faster and support "timeout", but does not clear input event level for poll()/select())
static inline __attribute__((always_inline, regparm(1))) int global_ivshmem_intr_wait_trigger(const int timeout_ms)
{
    register int ret = epoll_wait(global_ivshmem_intr_dev.epollfd, &global_ivshmem_intr_dev.dump, 1, timeout_ms);
    if (unlikely(ret < 0))
    {
        perror("epoll_wait(global_ivshmem_intr_dev.epollfd, &global_ivshmem_intr_dev.dump, 1, timeout_ms)");
        exit(EXIT_FAILURE);
    }
}
// Clear input event level for poll()/select() (slower and "timeout" is not possible)
static inline __attribute__((always_inline)) int global_ivshmem_intr_consume()
{
    ivshmem_intr_resp_t intr_resp = -1;
    register ssize_t len = read(global_ivshmem_intr_dev.fd, &intr_resp, sizeof(intr_resp));
    if (unlikely((len <= 0) && ((errno != EAGAIN))))
    {
        perror("read(global_ivshmem_intr_dev.fd, &intr_resp, sizeof(intr_resp))");
        exit(EXIT_FAILURE);
    }
    return intr_resp;
}

#define _GNU_SOURCE

#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>

// Global interrupt device descriptor definition
IVSHMEM_INTR_DEV global_ivshmem_intr_dev;

// Do not call multiple times!
void global_ivshmem_intr_init()
{
    // Local register variable for faster initialization
    register IVSHMEM_INTR_DEV ivshmem_intr_dev;
    register const char *global_ivshmem_intr_dev_path = getenv(ENVSTR_GLOBAL_IVSHMEM_INTR_DEV_PATH); // Get the global device file path through getenv()
    if (unlikely(!global_ivshmem_intr_dev_path))
    {
        fprintf(stderr, "Please specifiy the global IVSHMEM interrupt device path through %s=... first!", ENVSTR_GLOBAL_IVSHMEM_INTR_DEV_PATH);
        exit(EXIT_FAILURE);
    }

    // Open device file
    ivshmem_intr_dev.fd = open(global_ivshmem_intr_dev_path, O_RDWR | O_ASYNC | O_NONBLOCK);
    if (unlikely(ivshmem_intr_dev.fd < 0))
    {
        perror("open(global_ivshmem_intr_dev_path, O_RDWR | O_ASYNC | O_NONBLOCK)");
        exit(EXIT_FAILURE);
    }

    // mmap() on BAR0 region (which is for interrupt MMIO)
    ivshmem_intr_dev.mmio = (IVSHMEM_INTR_MMIO *)mmap(
        NULL, sizeof(ivshmem_intr_dev.mmio), PROT_READ | PROT_WRITE, MAP_SHARED, ivshmem_intr_dev.fd, 0);
    if (unlikely(ivshmem_intr_dev.mmio == MAP_FAILED))
    {
        perror("mmap(NULL, sizeof(ivshmem_intr_dev.mmio), PROT_READ | PROT_WRITE, MAP_SHARED, ivshmem_intr_dev.fd, 0)");
        exit(EXIT_FAILURE);
    }

    // Setup additional epollfd
    ivshmem_intr_dev.epollfd = epoll_create1(0);
    if (unlikely(ivshmem_intr_dev.epollfd < 0))
    {
        perror("epoll_create1(0)");
        exit(EXIT_FAILURE);
    }
    ivshmem_intr_dev.dump.events = EPOLLIN | EPOLLET;
    ivshmem_intr_dev.dump.data.fd = ivshmem_intr_dev.fd;
    if (unlikely(epoll_ctl(ivshmem_intr_dev.epollfd, EPOLL_CTL_ADD, ivshmem_intr_dev.fd, &ivshmem_intr_dev.dump)))
    {
        perror("epoll_ctl(ivshmem_intr_dev.epollfd, EPOLL_CTL_ADD, ivshmem_intr_dev.fd, &ivshmem_intr_dev.dump)");
        exit(EXIT_FAILURE);
    }

    // Set global device
    global_ivshmem_intr_dev = ivshmem_intr_dev;

    errno = 0;
    perror("global_ivshmem_intr_init()");
}



#define MAX_UMWAIT_WAIT_TSC_CNT 100000
static inline __attribute__((__always_inline__)) void umwait_sync(void *cache_line)
{ // UMWAIT (Intel 12th Gen.~)
    register unsigned long long tsc_cnt;

    _umonitor(cache_line);
    tsc_cnt = __rdtsc();
    _umwait(0, tsc_cnt + 100000);
}

extern "C"
{
    class SYSCALL_GOTO_OFFSETS
    {
    public:
        void *goto_base;
        long goto_offsets[NUM_SYSCALLS] = {0};

        SYSCALL_GOTO_OFFSETS(void *const goto_base, const size_t npairs, const GOTO_PAIR *__restrict__ goto_pairs)
        {
            DEBUG_PRINT("SYSCALL_GOTO_OFFSETS()\n");

            this->goto_base = goto_base;
            for (size_t i = 0; i < npairs; ++i)
            {
                this->goto_offsets[goto_pairs[i].idx] = goto_pairs[i].goto_offset;
            }
        }
    };
    
    typedef struct _epoll_allocated_internal {
        int fd;
        struct epoll_event event;
        struct _epoll_allocated_internal *next;
    } epoll_allocated_internal;

    typedef struct _epoll_allocated {
        int cnt;
        // unsigned char fds[128];
        unsigned int fds[1024];
        struct epoll_event *epoll_event[1024];
    } epoll_allocated;

    static __attribute__((__noinline__, __noclone__)) int stub_hook(long syscall_number, long arg0, long arg1, long arg2, long arg3, long arg4, long arg5, long *result)
    { // performance-critical
        static long syscall_ret;
        static const GOTO_PAIR goto_pairs[] = {
            {__NR_read, GOTO_OFFSET(READ, BASE)},
            {__NR_write, GOTO_OFFSET(WRITE, BASE)},
            {__NR_close, GOTO_OFFSET(CLOSE, BASE)},
            {__NR_select, GOTO_OFFSET(SELECT, BASE)},
            {__NR_poll, GOTO_OFFSET(POLL, BASE)},
            {__NR_pselect6, GOTO_OFFSET(PSELECT6, BASE)},
            {__NR_connect, GOTO_OFFSET(CONNECT, BASE)},
            {__NR_accept, GOTO_OFFSET(ACCEPT, BASE)},
            {__NR_sendto, GOTO_OFFSET(WRITE, BASE)},
            {__NR_recvfrom, GOTO_OFFSET(READ, BASE)},
            {__NR_epoll_wait, GOTO_OFFSET(EPOLL_WAIT, BASE)},
            {__NR_epoll_ctl, GOTO_OFFSET(EPOLL_CTL, BASE)},
            };
        static const SYSCALL_GOTO_OFFSETS syscall_goto_offsets(GOTO_ADDR(BASE), sizeof(goto_pairs) / sizeof(*goto_pairs), goto_pairs);
        goto *ADDR(syscall_goto_offsets.goto_base, syscall_goto_offsets.goto_offsets[syscall_number]);

        // Do NOT declare a stack variable in this function!
        // (since stub_hook() can recursively call itself)
        // Temp.
        static unsigned char fdmap[__FD_SETSIZE] = {0};
        static IVSHMEM_FIFO *__restrict__ fd_to_ivshmem_r_fifo[__FD_SETSIZE];
        static IVSHMEM_FIFO *__restrict__ fd_to_ivshmem_w_fifo[__FD_SETSIZE];

        static int memfd;
        static uint8_t cmd;
        static struct sockaddr_in client_addr;
        static socklen_t client_addr_len = sizeof(client_addr);

        static epoll_allocated *epoll_data[__FD_SETSIZE];
        static unsigned long long last_origin_select_called_tsc = 0;
        static unsigned long long last_origin_poll_called_tsc = 0;
        static unsigned long long last_origin_epoll_wait_called_tsc = 0;
        // Temp. END

    BASE:
        // Return; Call original syscall() without interception
        DEBUG_PRINT("stub_hook(%ld, ...)\n", syscall_number);

        return 1;

    READ: // performace-critical
        // Temp.
        if (TEST_BIT(fdmap, arg0))
        {
            DEBUG_PRINT("READ(%ld, %p, %ld, ...)\n", arg0, arg1, arg2);

            register IVSHMEM_FIFO *__restrict__ const src = fd_to_ivshmem_r_fifo[arg0];
            register long dest = arg1;
            register long remain_cnt = arg2;
            register const long pos_end = src->data.pos_end; // read() 중에 pos_end가 바뀌지 않을거라고 가정

            while (remain_cnt)
            {
                register const long pos_w = src->data.pos_w; // From here, src->data.pos_w should never be read.
                register const long pos_r = src->data.pos_r; // From here, src->data.pos_r should never be read.
                register const long actual_size = READ_FEASIBLE_BYTES(pos_r, pos_w, pos_end, remain_cnt);

                DEBUG_PRINT("READ: [prev] pos_end = %ld\n", pos_end);
                DEBUG_PRINT("READ: [prev] pos_r = %ld\n", pos_r);
                DEBUG_PRINT("READ: [prev] pos_w = %ld\n", pos_w);
                DEBUG_PRINT("READ: actual_size = %ld\n", actual_size);
                if (!actual_size)
                {
                    // Reset pos_r
                    if ((pos_r == pos_end) && (pos_r != pos_w))
                    {
                        src->data.pos_r = IVSHMEM_FIFO_POS_START;
                        global_ivshmem_intr_mmio_write(global_intr_dest_id);
                    }
                    // Reset pos_r END

                    else
                    {
                        // close() check --> 더 최적화 불가능 한가?
                        if (unlikely(src->data.term))
                        {
                            DEBUG_PRINT("READ: src->data.term = %ld\n", src->data.term);
                            DEBUG_PRINT("READ: arg2 - remain_cnt = %ld\n", arg2 - remain_cnt);
                            if (arg2 - remain_cnt)
                            {
                                *result = arg2 - remain_cnt;
                                return 0;
                            }
                            else
                            {
                                DEBUG_PRINT("stub_hook(SYS_read, %ld, ...)\n", arg0);

                                return 1;
                            }
                        }
                        // close() check END

                        // Timeout implementation 필요
                        // 여기에서 원래 소켓(또는 다른 fd 구현체)에서 오는 close() 메세지를 잡아내야 함!

                        // Sync
                        // _mm_pause(); // Great on Intel 12th Gen for both Gbps and trans./s; Great for trans./s on most CPUs
                        // umwait_sync((void *)&src->data.pos_w); // Similar with _mm_pause() or little bit worse
                        // usleep(100); // Great on old Intel CPUs & AMD CPUs for Gbps
                        global_ivshmem_intr_wait_trigger(-1);
                        
                        // Sync END
                    }
                }
                else
                {
                    OUT_MEMCPY((void *)dest, (void *)ADDR(src, pos_r), actual_size);
                    // Do we need memory fence here?
                    src->data.pos_r += actual_size;
                    global_ivshmem_intr_mmio_write(global_intr_dest_id);
                    dest += actual_size;
                    remain_cnt -= actual_size;
                }
                DEBUG_PRINT("READ: [new] pos_end = %ld\n", pos_end);
                DEBUG_PRINT("READ: [new] pos_r = %ld\n", pos_r);
                DEBUG_PRINT("READ: [new] pos_w = %ld\n", pos_w);
            }

            // Reception is done.
            *result = arg2 - remain_cnt;
            DEBUG_PRINT("READ: *result = %ld\n", *result);
            return 0;
        }
        // Temp. END

        // Return; Call original syscall() without interception
        DEBUG_PRINT("stub_hook(SYS_read, %ld, ...)\n", arg0);

        return 1;
        // Temp. END

    WRITE: // performace-critical
        // Temp.
        if (TEST_BIT(fdmap, arg0))
        {
            DEBUG_PRINT("WRITE(%ld, %p, %ld, ...)\n", arg0, arg1, arg2);

            register long src = arg1;
            register IVSHMEM_FIFO *__restrict__ const dest = fd_to_ivshmem_w_fifo[arg0];
            register long remain_cnt = arg2;
            register const long pos_end = dest->data.pos_end; // write() 중에 pos_end가 바뀌지 않을거라고 가정

            while (remain_cnt)
            {
                register const long pos_r = dest->data.pos_r; // From here, dest->data.pos_r should never be read.
                register const long pos_w = dest->data.pos_w; // From here, dest->data.pos_w should never be read.
                register const long actual_size = WRITE_FEASIBLE_BYTES(pos_w, pos_r, pos_end, remain_cnt);

                DEBUG_PRINT("WRITE: [prev] pos_end = %ld\n", pos_end);
                DEBUG_PRINT("WRITE: [prev] pos_r = %ld\n", pos_r);
                DEBUG_PRINT("WRITE: [prev] pos_w = %ld\n", pos_w);
                DEBUG_PRINT("WRITE: actual_size = %ld\n", actual_size);
                if (!actual_size)
                {
                    // Reset pos_w
                    if ((pos_w == pos_end) && (pos_r != IVSHMEM_FIFO_POS_START))
                    {
                        dest->data.pos_w = IVSHMEM_FIFO_POS_START;
                        global_ivshmem_intr_mmio_write(global_intr_dest_id);
                    }
                    // Reset pos_w END

                    else
                    {
                        // close() check --> 더 최적화 불가능 한가?
                        if (unlikely(dest->data.term))
                        {
                            DEBUG_PRINT("READ: dest->data.term = %ld\n", dest->data.term);
                            DEBUG_PRINT("READ: arg2 - remain_cnt = %ld\n", arg2 - remain_cnt);
                            if (arg2 - remain_cnt)
                            {
                                *result = arg2 - remain_cnt;
                                return 0;
                            }
                            else
                            {
                                DEBUG_PRINT("stub_hook(SYS_write, %ld, ...)\n", arg0);

                                return 1;
                            }
                        }
                        // close() check END

                        // Timeout implementation 필요

                        // Sync
                        // _mm_pause(); // Great on Intel 12th Gen for both Gbps and trans./s; Great for trans./s on most CPUs
                        // umwait_sync((void *)&src->data.pos_w); // Similar with _mm_pause() or little bit worse
                        global_ivshmem_intr_wait_trigger(-1);
                        // Sync END
                    }
                }
                else
                {
                    OUT_MEMCPY((void *)ADDR(dest, pos_w), (void *)src, actual_size);
                    // Do we need memory fence here?
                    dest->data.pos_w += actual_size;
                    global_ivshmem_intr_mmio_write(global_intr_dest_id);
                    src += actual_size;
                    remain_cnt -= actual_size;
                }
                DEBUG_PRINT("WRITE: [new] pos_end = %ld\n", pos_end);
                DEBUG_PRINT("WRITE: [new] pos_r = %ld\n", pos_r);
                DEBUG_PRINT("WRITE: [new] pos_w = %ld\n", pos_w);
            }

            // Reception is done.
            *result = arg2 - remain_cnt;
            DEBUG_PRINT("WRITE: %result = %ld\n", *result);
            return 0;
        }
        // Temp. END

        // Return; Call original syscall() without interception
        DEBUG_PRINT("stub_hook(SYS_write, %ld, ...)\n", arg0);

        return 1;

    CONNECT:
        // Temp.
        DEBUG_PRINT("stub_hook(SYS_connect, ...)\n");

        // 원래 connect() 수행
        // client_sockfd == arg0
        syscall_ret = syscall_no_intercept(syscall_number, arg0, arg1, arg2, arg3, arg4, arg5, result);
        *result = syscall_ret;
        if (syscall_ret)
        { // Original connect() has failed.
            DEBUG_PRINT("syscall_no_intercept(SYS_connect, ...): return %ld\n", syscall_ret);
            return 0;
        }
        // 원래 connect() 수행 끝

        DEBUG_PRINT("CONNECT(%ld, ...)\n", arg0);

        // 반대편 주소 가져오기
        assert(getpeername(arg0, (struct sockaddr *)&client_addr, &client_addr_len) == 0);
        if ((client_addr.sin_addr.s_addr & static_vm_target_netmask) != (static_vm_target_subnet & static_vm_target_netmask))
        { // The other side is outside of VM network.
            DEBUG_PRINT("The other side is outside of VM network.\n", syscall_ret);
            return 0;
        }
        // 반대편 주소 가져오기 끝

        // Sync
        cmd = REQ_CONNECT_TO_IVSHMEM;
        syscall_no_intercept(SYS_write, arg0, &cmd, sizeof(cmd)); // read(client_sockfd, &cmd, sizeof(cmd))

        syscall_no_intercept(SYS_read, syscall_ret, global_intr_dest_id, sizeof(uint32_t));
        syscall_no_intercept(SYS_write, syscall_ret, &global_ivshmem_intr_dev.mmio->IVPosition, sizeof(uint32_t));

        syscall_no_intercept(SYS_read, arg0, &cmd, sizeof(cmd));  // write(client_sockfd, &cmd, sizeof(cmd))
        assert(cmd == RESP_CONNECT_TO_IVSHMEM);
        // Sync END

        // 할당
        memfd = open(IVSHMEM_FILE, O_RDWR | O_ASYNC); // O_ASYNC must be used.
        if (memfd < 0)
        {
            // open() has failed.
            DEBUG_PERROR("open(memfd, ...)");
            return 0;
        }
        fd_to_ivshmem_w_fifo[arg0] = (IVSHMEM_FIFO *)mmap(NULL, IVSHMEM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, memfd, IVSHMEM_PHYS_ADDR);
        if (fd_to_ivshmem_w_fifo[arg0] == MAP_FAILED)
        {
            // mmap() has failed.
            DEBUG_PERROR("mmap(..., memfd, ...)");
            return 0;
        }
        fd_to_ivshmem_r_fifo[arg0] = ADDR(fd_to_ivshmem_w_fifo[arg0], IVSHMEM_STC_START);
        if (close(memfd) < 0)
        {
            // close() has failed.
            DEBUG_PERROR("close(memfd)");
            if (munmap(fd_to_ivshmem_w_fifo[arg0], IVSHMEM_SIZE / 2) < 0)
            {
                // munmap() has failed.
                DEBUG_PERROR("munmap(fd_to_ivshmem_w_fifo[arg0], ...)");
            }
            if (munmap(fd_to_ivshmem_r_fifo[arg0], IVSHMEM_SIZE / 2) < 0)
            {
                // munmap() has failed.
                DEBUG_PERROR("munmap(fd_to_ivshmem_r_fifo[arg0], ...)");
            }
            return 0;
        }
        // 할당 끝

        // Set fdmap
        SET_BIT(fdmap, arg0);
        // Set fdmap END

        // Connection is done.
        return 0;
        // Temp. END

    ACCEPT:
        // Temp.
        DEBUG_PRINT("stub_hook(SYS_accept, ...)\n");

        // 원래 accept() 수행
        // client_sockfd == syscall_ret
        syscall_ret = syscall_no_intercept(syscall_number, arg0, arg1, arg2, arg3, arg4, arg5, result);
        *result = syscall_ret;
        if (syscall_ret < 0)
        { // accept() has failed.
            DEBUG_PRINT("syscall_no_intercept(SYS_accept, ...): return %ld\n", syscall_ret);
            return 0;
        }
        // 원래 accept() 수행 끝

        DEBUG_PRINT("ACCEPT(%ld, ...)\n", syscall_ret);

        // 반대편 주소 가져오기
        assert(getpeername(syscall_ret, (struct sockaddr *)&client_addr, &client_addr_len) == 0);
        if ((client_addr.sin_addr.s_addr & static_vm_target_netmask) != (static_vm_target_subnet & static_vm_target_netmask))
        { // The other side is outside of VM network.
            DEBUG_PRINT("The other side is outside of VM network.\n", syscall_ret);
            return 0;
        }
        // 반대편 주소 가져오기 끝

        // Sync
        syscall_no_intercept(SYS_read, syscall_ret, &cmd, sizeof(cmd)); // read(client_sockfd, &cmd, sizeof(cmd))
        assert(cmd == REQ_CONNECT_TO_IVSHMEM);

        // 할당
        memfd = open(IVSHMEM_FILE, O_RDWR | O_ASYNC); // O_ASYNC must be used.
        if (memfd < 0)
        {
            // open() has failed.
            DEBUG_PERROR("open(memfd, ...)");
            return 0;
        }
        fd_to_ivshmem_r_fifo[syscall_ret] = (IVSHMEM_FIFO *)mmap(NULL, IVSHMEM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, memfd, IVSHMEM_PHYS_ADDR);
        if (fd_to_ivshmem_r_fifo[syscall_ret] == MAP_FAILED)
        {
            // mmap() has failed.
            DEBUG_PERROR("mmap(..., memfd, ...)");
            return 0;
        }
        fd_to_ivshmem_w_fifo[syscall_ret] = ADDR(fd_to_ivshmem_r_fifo[syscall_ret], IVSHMEM_STC_START);
        if (close(memfd) < 0)
        {
            // close() has failed.
            DEBUG_PERROR("close(memfd)");
            if (munmap(fd_to_ivshmem_r_fifo[syscall_ret], IVSHMEM_SIZE / 2) < 0)
            {
                // munmap() has failed.
                DEBUG_PERROR("munmap(fd_to_ivshmem_r_fifo[syscall_ret], ...)");
            }
            if (munmap(fd_to_ivshmem_w_fifo[syscall_ret], IVSHMEM_SIZE / 2) < 0)
            {
                // munmap() has failed.
                DEBUG_PERROR("munmap(fd_to_ivshmem_w_fifo[syscall_ret], ...)");
            }
            return 0;
        }
        // 할당 끝

        // 전체 영역 초기화 (for security - Previous buffer data can be left)
        memset(fd_to_ivshmem_r_fifo[syscall_ret], 0, IVSHMEM_SIZE);
        // 전체 영역 초기화 끝

        // CTS 초기화
        fd_to_ivshmem_r_fifo[syscall_ret]->data.pos_end = IVSHMEM_FIFO_SIZE;
        fd_to_ivshmem_r_fifo[syscall_ret]->data.pos_r = IVSHMEM_FIFO_POS_START; // pos_r = IVSHMEM_FIFO_POS_START
        fd_to_ivshmem_r_fifo[syscall_ret]->data.pos_w = IVSHMEM_FIFO_POS_START; // pos_w = IVSHMEM_FIFO_POS_START
        fd_to_ivshmem_r_fifo[syscall_ret]->data.term = 0;
        // CTS 초기화 END

        // STC 초기화
        fd_to_ivshmem_w_fifo[syscall_ret]->data.pos_end = IVSHMEM_FIFO_SIZE;
        fd_to_ivshmem_w_fifo[syscall_ret]->data.pos_r = IVSHMEM_FIFO_POS_START; // pos_r = IVSHMEM_FIFO_POS_START
        fd_to_ivshmem_w_fifo[syscall_ret]->data.pos_w = IVSHMEM_FIFO_POS_START; // pos_w = IVSHMEM_FIFO_POS_START
        fd_to_ivshmem_w_fifo[syscall_ret]->data.term = 0;
        // STC 초기화 END

        // Set fdmap
        SET_BIT(fdmap, syscall_ret);
        // Set fdmap END

        syscall_no_intercept(SYS_write, syscall_ret, &global_ivshmem_intr_dev.mmio->IVPosition, sizeof(uint32_t));
        syscall_no_intercept(SYS_read, syscall_ret, global_intr_dest_id, sizeof(uint32_t));

        cmd = RESP_CONNECT_TO_IVSHMEM;
        syscall_no_intercept(SYS_write, syscall_ret, &cmd, sizeof(cmd)); // write(client_sockfd, &cmd, sizeof(cmd))
        // Sync END

        // Acception is done.
        return 0;
        // Temp. END

    
    SELECT:
    PSELECT6:
        DEBUG_PRINT("stub_hook(SYS_select, %ld)\n", arg0);

        // readfds, writefds, errorfds
        {
            /*
                1. 루프합치기
                    - readfds와 writefds를 하나의 루프로 구현
                    - readfds, writefds가 NULL인지 확인 해야함
                    - 원래 select를 한 번만 호출해도됨
                2. 각 fds마다 다른 루프로 구현

                3. timeout은 무시한다. 단순히 읽을거 있는지 써진거 있는지만 확인함
            */

#if 1
            long *fdmap_ptr = (long *)(void *)fdmap;
            int nfds = arg0;

            int setted_bit = 0;
            bool must_call_origin = false;

            for (int i = 0; i < FD_SETSIZE / NFDBITS; i++) {
                if (i * sizeof(long) > nfds)
                    break;

                //
                //  readfds
                //
                if (arg1 != NULL) {
                    fd_set *fds = (fd_set *)arg1;
                    long *fds_bits = (long *)(fds->fds_bits);
                    long x = fds_bits[i] & fdmap_ptr[i];

                    if (x != 0) {
                        if (!must_call_origin && ((x | fdmap_ptr[i]) != fdmap_ptr[i])) {
                            must_call_origin = true;
                        }

                        for ( ; x; ) {
                            long now_fd = i * sizeof(long) + __builtin_ctz(x);
                            // DEBUG_PRINT("value: %d, %d, %d, %d\n", i, x, __builtin_ctz(x), now_fd);

                            if (!TEST_BIT(fdmap, now_fd)) {
                                x = x & (x - 1);
                                continue;
                            }

                            register IVSHMEM_FIFO *__restrict__ const src = fd_to_ivshmem_r_fifo[now_fd];
                            register long remain_cnt = 1;

                            register const long pos_end = src->data.pos_end;
                            register const long pos_w = src->data.pos_w; // From here, src->data.pos_w should never be read.
                            register const long pos_r = src->data.pos_r; // From here, src->data.pos_r should never be read.
                            register const long actual_size = READ_FEASIBLE_BYTES(pos_r, pos_w, pos_end, remain_cnt);

                            DEBUG_PRINT("spy %ld %ld %ld\n", pos_end, pos_w, pos_r);

                            if (!actual_size) {
                                UNSET_BIT(fds_bits, now_fd);
                            } else {
                                setted_bit++;
                            }

                            // 가장 오른쪽 1비트를 0으로 바꿈
                            x = x & (x - 1);
                        }
                    } else if (fds_bits[i] > 0) {
                        must_call_origin = false;
                    }
                }

                //
                //  writefds
                //
                if (arg2 != NULL) {
                    fd_set *fds = (fd_set *)arg1;
                    long *fds_bits = (long *)(fds->fds_bits);
                    long x = fds_bits[i] & fdmap_ptr[i];

                    if (x != 0) {
                        if (!must_call_origin && ((x | fdmap_ptr[i]) != fdmap_ptr[i])) {
                            must_call_origin = true;
                        }

                        for ( ; x; ) {
                            long now_fd = i * sizeof(long) + __builtin_ctz(x);

                            if (!TEST_BIT(fdmap, now_fd)) {
                                x = x & (x - 1);
                                continue;
                            }

                            register IVSHMEM_FIFO *__restrict__ const dest = fd_to_ivshmem_w_fifo[now_fd];
                            register long remain_cnt = 1;
                            register const long pos_end = dest->data.pos_end; // write() 중에 pos_end가 바뀌지 않을거라고 가정

                            register const long pos_r = dest->data.pos_r; // From here, dest->data.pos_r should never be read.
                            register const long pos_w = dest->data.pos_w; // From here, dest->data.pos_w should never be read.
                            register const long actual_size = WRITE_FEASIBLE_BYTES(pos_w, pos_r, pos_end, remain_cnt);

                            if (!actual_size) {
                                UNSET_BIT(fds_bits, now_fd);
                            } else {
                                setted_bit++;
                            }

                            // 가장 오른쪽 1비트를 0으로 바꿈
                            x = x & (x - 1);
                        }
                    } else if (fds_bits[i] > 0) {
                        must_call_origin = false;
                    }
                }
            }
       
            if (must_call_origin || arg3 != NULL) {
                fd_set readfds_tmp;
                fd_set writefds_tmp;
                fd_set *readfds_tmp_ptr = NULL;
                fd_set *writefds_tmp_ptr = NULL;

                if (arg1 != NULL) {
                    fd_set *fds = (fd_set *)arg1;
                    readfds_tmp = *fds;
                    for (int i = 0; i < FD_SETSIZE / NFDBITS; i++) {
                        readfds_tmp.fds_bits[i] ^= readfds_tmp.fds_bits[i] & fdmap_ptr[i];
                    }
                    readfds_tmp_ptr = &readfds_tmp;
                }
                
                if (arg2 != NULL) {
                    fd_set *fds = (fd_set *)arg1;
                    writefds_tmp = *fds;
                    for (int i = 0; i < FD_SETSIZE / NFDBITS; i++) {
                        writefds_tmp.fds_bits[i] ^= writefds_tmp.fds_bits[i] & fdmap_ptr[i];
                    }
                    writefds_tmp_ptr = &writefds_tmp;
                }

                int ret = syscall_no_intercept(SYS_select, arg0, readfds_tmp_ptr, writefds_tmp_ptr, arg3, arg4);

                if (ret >= 0) {
                    setted_bit += ret;
                } else {
                    *result = ret;
                    return 0;
                }

                for (int i = 0; i < FD_SETSIZE / NFDBITS; i++) {
                    if (i * sizeof(long) > nfds)
                        break;
                    if (arg1 != NULL) {
                        ((fd_set *)arg1)->fds_bits[i] |= readfds_tmp.fds_bits[i];
                    }
                    if (arg2 != NULL) {
                        ((fd_set *)arg1)->fds_bits[i] |= writefds_tmp.fds_bits[i];
                    }
                }
            }

            *result = setted_bit;
            return 0;
#else
            long *fdmap_ptr = (long *)(void *)fdmap;
            int nfds = arg0;
            int setted_bit = 0;
            // 일단 readfds부터 해보자
            if (arg1 != NULL) {
                fd_set *readfds = (fd_set *)arg1;
                long *readfds_bits = (long *)(readfds->fds_bits);
                // 우리가 관리하고 있는 fd가 목록에 있는지?
                bool has_managed_fd = false;
                // 우리가 관리하고 있는 fd만 목록에 있는지?
                bool has_only_managed_fd = false;
                // fd 테스팅
                for (int i = 0; i < FD_SETSIZE / NFDBITS; i++) {
                    if (i * sizeof(long) > nfds)
                        break;
                    // and 해서 0이 아니라면 겹치는 부분 존재
                    long x = readfds_bits[i] & fdmap_ptr[i];
                    if (x != 0) {
                        has_managed_fd = true;
                        if (x | fdmap_ptr[i] == fdmap_ptr[i]) {
                            has_only_managed_fd = true;
                            break;
                        }
                    }
                }
                // 1. 우리가 관리하고 있는 fd가 없다면
                // 1.1. 원래 syscall을 호출시킨다.
                if (!has_managed_fd) {
                    return 1;
                }
                // 2. 우리가 관리하고 있는 fd를 확인해준다.
                for (int i = 0; i < FD_SETSIZE / NFDBITS; i++) {
                    if (i * sizeof(long) > nfds)
                        break;
                    long x = readfds_bits[i] & fdmap_ptr[i];
                    for ( ; x; ) {
                        long now_fd = i * sizeof(long) + __builtin_ctz(x);
                        register IVSHMEM_FIFO *__restrict__ const src = fd_to_ivshmem_r_fifo[now_fd];
                        register long remain_cnt = 1;
                        register const long pos_end = src->data_r.pos_end;
                        register const long pos_w = src->data_w.pos_w; // From here, src->data.pos_w should never be read.
                        register const long pos_r = src->data_r.pos_r; // From here, src->data.pos_r should never be read.
                        register const long actual_size = READ_FEASIBLE_BYTES(pos_r, pos_w, pos_end, remain_cnt);
                        if (!actual_size) {
                            UNSET_BIT(readfds_bits, now_fd);
                        } else {
                            setted_bit++;
                        }
                        // 가장 오른쪽 1비트를 0으로 바꿈
                        x = ~x & (x+1);
                    }
                }
                if (!has_only_managed_fd) {
                    // 3. 우리가 관리하고 있는 fd와 일반 fd가 같이 select된 것이라면
                    // 3.1. 일단 우리 것 부터 빠르게 처리 <= 2에서 처리
                    // 3.2. 우리꺼 처리해놓은걸 복사하고, 복사본의 fd mark를 0으로 세팅
                    fd_set readfds_tmp = *readfds;
                    for (int i = 0; i < FD_SETSIZE / NFDBITS; i++) {
                        readfds_tmp.fds_bits[i] ^= readfds_tmp.fds_bits[i] & fdmap_ptr[i];
                    }
                    // 3.3. 원래 select 호출
                    int ret = syscall_no_intercept(SYS_select, arg0, &readfds_tmp, NULL, NULL, arg4);
                    // 3.4. 원래 select 성공, 실패 확인
                    if (ret >= 0) {
                        setted_bit += ret;
                    } else {
                        *result = ret;
                        return 0;
                    }
                    // 3.5. result 합치기
                    for (int i = 0; i < FD_SETSIZE / NFDBITS; i++) {
                        if (i * sizeof(long) > nfds)
                            break;
                        readfds->fds_bits[i] |= readfds_tmp.fds_bits[i];
                    }
                }
            }
            
            // 그 다음 writefds
            if (arg2 != NULL) {
                fd_set *writefds = (fd_set *)arg2;
                long *writefds_bits = (long *)(writefds->fds_bits);
                // 우리가 관리하고 있는 fd가 목록에 있는지?
                bool has_managed_fd = false;
                // 우리가 관리하고 있는 fd만 목록에 있는지?
                bool has_only_managed_fd = false;
                // fd 테스팅
                for (int i = 0; i < FD_SETSIZE / NFDBITS; i++) {
                    if (i * sizeof(long) > nfds)
                        break;
                    // and 해서 0이 아니라면 겹치는 부분 존재
                    long x = writefds_bits[i] & fdmap_ptr[i];
                    if (x != 0) {
                        has_managed_fd = true;
                        if (x == fdmap_ptr[i]) {
                            has_only_managed_fd = true;
                            break;
                        }
                    }
                }
                // 1. 우리가 관리하고 있는 fd가 없다면
                // 1.1. 원래 syscall을 호출시킨다.
                if (!has_managed_fd) {
                    return 1;
                }
                // 2. 우리가 관리하고 있는 fd를 확인해준다.
                for (int i = 0; i < FD_SETSIZE / NFDBITS; i++) {
                    if (i * sizeof(long) > nfds)
                        break;
                    long x = writefds_bits[i] & fdmap_ptr[i];
                    for ( ; x; ) {
                        long now_fd = i * sizeof(long) + __builtin_ctz(x);
                        register IVSHMEM_FIFO *__restrict__ const src = fd_to_ivshmem_r_fifo[now_fd];
                        register long remain_cnt = 1;
                        register const long pos_end = src->data_r.pos_end;
                        register const long pos_w = src->data_w.pos_w; // From here, src->data.pos_w should never be read.
                        register const long pos_r = src->data_r.pos_r; // From here, src->data.pos_r should never be read.
                        register const long actual_size = READ_FEASIBLE_BYTES(pos_r, pos_w, pos_end, remain_cnt);
                        if (!actual_size) {
                            UNSET_BIT(writefds_bits, now_fd);
                        } else {
                            setted_bit++;
                        }
                        // 가장 오른쪽 1비트를 0으로 바꿈
                        x = ~x & (x+1);
                    }
                }
                if (!has_only_managed_fd) {
                    // 3. 우리가 관리하고 있는 fd와 일반 fd가 같이 select된 것이라면
                    // 3.1. 일단 우리 것 부터 빠르게 처리 <= 2에서 처리
                    // 3.2. 우리꺼 처리해놓은걸 복사하고, 복사본의 fd mark를 0으로 세팅
                    fd_set writefds_tmp = *writefds;
                    for (int i = 0; i < FD_SETSIZE / NFDBITS; i++) {
                        if (i * sizeof(long) > nfds)
                            break;
                        writefds_tmp.fds_bits[i] ^= writefds_tmp.fds_bits[i] & fdmap_ptr[i];
                    }
                    // 3.3. 원래 select 호출
                    int ret = syscall_no_intercept(SYS_select, arg0, NULL, &writefds_tmp, NULL, arg4);
                    // 3.4. 원래 select 성공, 실패 확인
                    if (ret >= 0) {
                        setted_bit += ret;
                    } else {
                        *result = ret;
                        return 0;
                    }
                    // 3.5. result 합치기
                    for (int i = 0; i < FD_SETSIZE / NFDBITS; i++) {
                        if (i * sizeof(long) > nfds)
                            break;
                        writefds->fds_bits[i] |= writefds_tmp.fds_bits[i];
                    }
                }
            }
            // syscall_ret = syscall_no_intercept(syscall_number, arg0, arg1, arg2, arg3, arg4, arg5, result);
            // *result = syscall_ret;
            // // if timeout
            // *result = 0;
            // // if error
            // *result = -1;
            
            *result = setted_bit;
            return 0;
#endif
        }

        return 1;

    POLL:
        DEBUG_PRINT("stub_hook(SYS_poll, %ld)\n", arg0);

        {
            struct pollfd *fds = (struct pollfd *)arg0;
            nfds_t nfds = (nfds_t)arg1;
            int timeout = (int)arg2;

            register unsigned long long now_tsc = __rdtsc();
            register unsigned long long tsc_diff = last_origin_poll_called_tsc - now_tsc;

            if (tsc_diff > (1LL << (sizeof(long long) - 1)))
                tsc_diff = now_tsc - last_origin_poll_called_tsc;

            DEBUG_PRINT("diff => %lld", tsc_diff);

            if (tsc_diff < 150000) {
                bool must_call_origin = false;

                int trig_cnt = 0;
                

                for (int i = 0; i < nfds; i++) {
                    struct pollfd *fd = &fds[i];

                    if (!TEST_BIT(fdmap, fd->fd)) {
                        must_call_origin = true;
                        continue;
                    }

                    if (fd->events == POLLIN) {
                        register IVSHMEM_FIFO *__restrict__ const src = fd_to_ivshmem_r_fifo[fd->fd];
                        register long remain_cnt = 1;

                        register const long pos_end = src->data.pos_end;
                        register const long pos_w = src->data.pos_w; // From here, src->data.pos_w should never be read.
                        register const long pos_r = src->data.pos_r; // From here, src->data.pos_r should never be read.
                        register const long actual_size = READ_FEASIBLE_BYTES(pos_r, pos_w, pos_end, remain_cnt);

                        if (actual_size) {
                            fd->revents |= POLLIN;
                            trig_cnt++;
                        }
                    }

                    if (fd->events == POLLOUT) {
                        register IVSHMEM_FIFO *__restrict__ const dest = fd_to_ivshmem_w_fifo[fd->fd];
                        register long remain_cnt = 1;

                        register const long pos_end = dest->data.pos_end; // write() 중에 pos_end가 바뀌지 않을거라고 가정
                        register const long pos_r = dest->data.pos_r; // From here, dest->data.pos_r should never be read.
                        register const long pos_w = dest->data.pos_w; // From here, dest->data.pos_w should never be read.
                        register const long actual_size = WRITE_FEASIBLE_BYTES(pos_w, pos_r, pos_end, remain_cnt);

                        if (actual_size) {
                            fd->revents |= POLLIN;
                            trig_cnt++;
                        }
                    }
                }

                if (must_call_origin) {
                    // TODO: 우리꺼가 같이 넘어가니깐 우리꺼의 fd->event를 0으로 바꿔야하는가?
                    int ret = syscall_no_intercept(SYS_poll, arg0, arg1, arg2);
                    trig_cnt += ret;
                }

                *result = trig_cnt;
                return 0;

            }
        }

        last_origin_poll_called_tsc = __rdtsc();

        return 1;

    EPOLL_CTL:
        DEBUG_PRINT("stub_hook(SYS_epoll_ctl, %ld)\n", arg0);

        {
            int epfd = (int)arg0;
            int op = (int)arg1;
            int fd = (int)arg2;
            struct epoll_event *event = (struct epoll_event *)arg3;

            if (TEST_BIT(fdmap, fd)) {
                if (epfd < 1024) {
                    // epoll data[epfd]가 null이라면 새로 만들어 주자
                    if (epoll_data[epfd] == NULL) {
                        epoll_data[epfd] = (epoll_allocated *)malloc(sizeof(epoll_allocated));
                        memset(epoll_data[epfd], 0, sizeof(epoll_allocated));
                    }

                    // 비트셋 구현
                    if (op ==  EPOLL_CTL_ADD) {
                        SET_BIT(epoll_data[epfd]->fds, fd);
                    } else if (op == EPOLL_CTL_DEL) {
                        UNSET_BIT(epoll_data[epfd]->fds, fd);
                    }

                    epoll_data[epfd]->cnt++;

                    epoll_event *eve = (epoll_event *)malloc(sizeof(epoll_event));
                    memcpy(eve, event, sizeof(epoll_event));
                    epoll_data[epfd]->epoll_event[fd] = event;


                    *result = 0;
                } else {
                    *result = -1;
                }

                return 0;
            } else {

                // 우리가 관심있는 fd가 아니라면 그냥 bitset에 등록만 해준다.
                if (epfd < 1024 && epoll_data[epfd] != NULL) {
                    if (op ==  EPOLL_CTL_ADD) {
                        SET_BIT(epoll_data[epfd]->fds, fd);
                    } else if (op == EPOLL_CTL_DEL) {
                        UNSET_BIT(epoll_data[epfd]->fds, fd);
                    }
                }
            }
        }

        return 1;

    EPOLL_WAIT:
        DEBUG_PRINT("stub_hook(SYS_epoll_wait, %ld)\n", arg0);

        {
            int epfd = (int)arg0;
            struct epoll_event *events = (struct epoll_event *)arg1;
            int maxevents = (int)arg2;
            int timeout = (int)arg3;

            register unsigned long long now_tsc = __rdtsc();
            register unsigned long long tsc_diff = last_origin_epoll_wait_called_tsc - now_tsc;

            if (tsc_diff > (1LL << (sizeof(long long) - 1)))
                tsc_diff = now_tsc - last_origin_epoll_wait_called_tsc;

            if (epoll_data[epfd] != NULL && tsc_diff < 150000) {
                int count_event = 0;
                int count_not_our_fd = 0;

                // for (int i = 0; i < epoll_data[epfd]->cnt && count_event < maxevents; i++) {
                for (int i = 0; i < 1024 && count_event < maxevents; i++) {
                    int fd = i;

                    if (!TEST_BIT(epoll_data[epfd]->fds, fd))
                        continue;

                    if (!TEST_BIT(fdmap, fd)) {
                        // 우리가 관심있는 fd가 아닌경우
                        count_not_our_fd++;
                        continue;
                    }

                    uint32_t event = epoll_data[epfd]->epoll_event[fd]->events;

                    if (event & EPOLLIN) {
                        register IVSHMEM_FIFO *__restrict__ const src = fd_to_ivshmem_r_fifo[fd];
                        register long remain_cnt = 1;

                        register const long pos_end = src->data.pos_end;
                        register const long pos_w = src->data.pos_w; // From here, src->data.pos_w should never be read.
                        register const long pos_r = src->data.pos_r; // From here, src->data.pos_r should never be read.
                        register const long actual_size = READ_FEASIBLE_BYTES(pos_r, pos_w, pos_end, remain_cnt);

                        if (actual_size) {
                            events[count_event].events |= EPOLLIN;
                            events[count_event].data.fd = fd;
                        }
                    }

                    if (event & EPOLLOUT) {
                        register IVSHMEM_FIFO *__restrict__ const dest = fd_to_ivshmem_w_fifo[fd];
                        register long remain_cnt = 1;

                        register const long pos_end = dest->data.pos_end; // write() 중에 pos_end가 바뀌지 않을거라고 가정
                        register const long pos_r = dest->data.pos_r; // From here, dest->data.pos_r should never be read.
                        register const long pos_w = dest->data.pos_w; // From here, dest->data.pos_w should never be read.
                        register const long actual_size = WRITE_FEASIBLE_BYTES(pos_w, pos_r, pos_end, remain_cnt);

                        if (actual_size) {
                            events[count_event].events |= EPOLLOUT;
                            events[count_event].data.fd = fd;
                        }
                    }

                    if (events[count_event].events > 0) {
                        count_event++;
                    }
                }

                *result = count_event;

                // 우리가 관심있는 fd 이벤트 정보는 하나도 업데이트되지 않았고,
                // 우리가 관심없는 fd들이 epfd에 포함이 되어있는 경우
                if (count_event == 0 && count_not_our_fd > 0) {
                    // timeout를 1로 설정하고 원래 epoll_wait을 호출
                    *result = syscall_no_intercept(SYS_epoll_wait, arg0, arg1, arg2, 1);
                }

                return 0;
            }
        }

        last_origin_epoll_wait_called_tsc = __rdtsc();

        return 1;

    CLOSE:
        if (TEST_BIT(fdmap, arg0))
        {
            DEBUG_PRINT("CLOSE(%ld)\n", arg0);

            // Temp.
            // Do we need atomic here?
            fd_to_ivshmem_r_fifo[arg0]->data.term = 1;
            fd_to_ivshmem_w_fifo[arg0]->data.term = 1;

            if (munmap(fd_to_ivshmem_r_fifo[arg0], IVSHMEM_SIZE / 2) < 0)
            {
                // munmap() has failed.
                DEBUG_PERROR("munmap(fd_to_ivshmem_r_fifo[arg0], ...)");
            }
            if (munmap(fd_to_ivshmem_w_fifo[arg0], IVSHMEM_SIZE / 2) < 0)
            {
                // munmap() has failed.
                DEBUG_PERROR("munmap(fd_to_ivshmem_w_fifo[arg0], ...)");
            }
            // Temp. END
        }

        // Return; Call original syscall() without interception
        DEBUG_PRINT("stub_hook(SYS_close, %ld)\n", arg0);

        return 1;
    }
    static __attribute__((constructor)) void stub_init(void)
    {
        eprintf_syscall = syscall_no_intercept;

        //  = {.dump = {0}, .epollfd = -1, .fd = -1, .mmio = NULL}
        global_ivshmem_intr_dev.dump = {0};
        global_ivshmem_intr_dev.epollfd = -1;
        global_ivshmem_intr_dev.fd = -1;
        global_ivshmem_intr_dev.mmio = NULL;
        global_ivshmem_intr_init();

        // [Temp. ] Set static target subnet environment variable
        const char *envstr_usernet_static_domain_subnet = getenv("USERNET_STATIC_DOMAIN_SUBNET");
        const char *envstr_usernet_static_domain_netmask = getenv("USERNET_STATIC_DOMAIN_NETMASK");

        if (!envstr_usernet_static_domain_subnet)
        {
            fprintf(stderr, "Please define USERNET_STATIC_DOMAIN_SUBNET=<SUBNET_IPV4> environment variable first!\n");
            exit(EXIT_FAILURE);
        }
        else
        {
            static_vm_target_subnet = inet_addr(envstr_usernet_static_domain_subnet);
        }
        if (!envstr_usernet_static_domain_netmask)
        {
            fprintf(stderr, "Please define USERNET_STATIC_DOMAIN_NETMASK=<NETMASK_IPV4> environment variable first!\n");
            exit(EXIT_FAILURE);
        }
        else
        {
            static_vm_target_netmask = inet_addr(envstr_usernet_static_domain_netmask);
        }

        DEBUG_PRINT("stub_init()\n");
        intercept_hook_point = stub_hook;
    }
}
