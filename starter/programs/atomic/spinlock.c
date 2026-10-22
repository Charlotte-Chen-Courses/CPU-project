// spinlock.c: a test-and-set spinlock (amoswap.w.aq / amoswap.w.rl) around a critical
// section that mixes loads and stores with the lock word nearby.
#define N 40
volatile int lock;
int shared[N], log_[N];

static void acquire(void) { while (__atomic_exchange_n(&lock, 1, __ATOMIC_ACQUIRE)) ; }
static void release(void) { __atomic_store_n(&lock, 0, __ATOMIC_RELEASE); }

int main(void) {
    for (int r = 0; r < 5; r++)
        for (int i = 0; i < N; i++) {
            acquire();
            shared[i] = shared[i] * 3 + r + i;
            log_[i] += shared[(i + 1) % N];
            release();
        }
    return 0;
}
