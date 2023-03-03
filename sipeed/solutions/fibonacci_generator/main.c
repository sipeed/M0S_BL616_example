#include "bflb_mtimer.h"
#include "board.h"

#define DBG_TAG "MAIN"
#include "log.h"

#include "sipeed/coroutine.h"

uint64_t fibonacci_generator(void)
{
    static uint64_t fib_prev = 0;
    static uint64_t fib_curr = 1;

    scrBegin;
    for (;;) {
        uint64_t fib_next;
        fib_next = fib_prev + fib_curr;
        fib_prev = fib_curr;
        fib_curr = fib_next;
        if (fib_curr < fib_prev)
            scrReturn(-1);
        scrReturn(fib_curr);
    }
    scrFinish(-1);
}

int main(void)
{
    board_init();

    while (1) {
        uint64_t fib_curr = fibonacci_generator();
        _ASSERT_PARAM(fib_curr != -1);
        LOG_I("%llu\r\n", fib_curr);
        bflb_mtimer_delay_ms(1000);
    }
}
