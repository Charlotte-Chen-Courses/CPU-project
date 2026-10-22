// cas_stack.c: a lock-free (Treiber) stack and a counter updated with compare-and-swap,
// which GCC turns into LR/SC retry loops.
#define N 64
struct node { struct node *next; int val; };
struct node nodes[N];
struct node *top;
int counter, popped[N];

static void push(struct node *n) {
    struct node *old = __atomic_load_n(&top, __ATOMIC_RELAXED);
    do n->next = old;
    while (!__atomic_compare_exchange_n(&top, &old, n, 0, __ATOMIC_RELEASE, __ATOMIC_RELAXED));
}

static struct node *pop(void) {
    struct node *old = __atomic_load_n(&top, __ATOMIC_ACQUIRE);
    while (old && !__atomic_compare_exchange_n(&top, &old, old->next, 0, __ATOMIC_ACQUIRE, __ATOMIC_ACQUIRE))
        ;
    return old;
}

int main(void) {
    for (int i = 0; i < N; i++) { nodes[i].val = i * 3 + 1; push(&nodes[i]); }
    for (int i = 0; i < N / 2; i++) popped[i] = pop()->val;
    for (int i = 0; i < 100; i++) {
        int c = __atomic_load_n(&counter, __ATOMIC_RELAXED);
        while (!__atomic_compare_exchange_n(&counter, &c, c + i, 0, __ATOMIC_SEQ_CST, __ATOMIC_RELAXED))
            ;
    }
    return 0;
}
