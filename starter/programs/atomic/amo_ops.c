// amo_ops.c: every AMO on an array (fetch_add/and/or/xor/exchange through __atomic builtins,
// signed/unsigned min/max through inline asm), results kept in memory.
#define N 32
int      a[N];
unsigned u[N];
int      old_vals[6 * N];

static int amomin(int *p, int v)  { int r; __asm__ volatile ("amomin.w %0, %2, (%1)"  : "=r"(r) : "r"(p), "r"(v) : "memory"); return r; }
static int amomax(int *p, int v)  { int r; __asm__ volatile ("amomax.w %0, %2, (%1)"  : "=r"(r) : "r"(p), "r"(v) : "memory"); return r; }
static unsigned amominu(unsigned *p, unsigned v) { unsigned r; __asm__ volatile ("amominu.w %0, %2, (%1)" : "=r"(r) : "r"(p), "r"(v) : "memory"); return r; }
static unsigned amomaxu(unsigned *p, unsigned v) { unsigned r; __asm__ volatile ("amomaxu.w %0, %2, (%1)" : "=r"(r) : "r"(p), "r"(v) : "memory"); return r; }

int main(void) {
    unsigned s = 7;
    int k = 0;
    for (int i = 0; i < N; i++) { s = s * 1664525u + 1013904223u; a[i] = (int)s; u[i] = s ^ 0x80000000u; }
    for (int i = 0; i < N; i++) {
        s = s * 1664525u + 1013904223u;
        old_vals[k++] = __atomic_fetch_add(&a[i], (int)(s >> 3), __ATOMIC_SEQ_CST);
        old_vals[k++] = __atomic_fetch_xor(&a[i], (int)s, __ATOMIC_ACQUIRE);
        old_vals[k++] = __atomic_fetch_and(&a[(i + 5) % N], (int)(s | 0x0f0f0000), __ATOMIC_RELEASE);
        old_vals[k++] = __atomic_fetch_or(&a[(i + 9) % N], (int)(s & 0x00ff00ff), __ATOMIC_RELAXED);
        old_vals[k++] = __atomic_exchange_n(&a[(i + 3) % N], (int)(s >> 1), __ATOMIC_ACQ_REL);
        old_vals[k++] = amomin(&a[i], (int)s) + amomax(&a[(i + 1) % N], (int)(s << 2))
                      + (int)amominu(&u[i], s) + (int)amomaxu(&u[(i + 7) % N], s >> 2);
    }
    return 0;
}
