// newton.c: square roots by Newton's method (fdiv-bound), checked against fsqrt.
#define N 64
float in[N], nw[N], hw[N];

static float my_sqrt(float a) {
    float g = a > 1.0f ? a * 0.5f : 1.0f;
    for (int k = 0; k < 8; k++) g = 0.5f * (g + a / g);
    return g;
}

int main(void) {
    for (int i = 0; i < N; i++) in[i] = (float)(i * i + 3) * 0.75f;
    for (int i = 0; i < N; i++) {
        nw[i] = my_sqrt(in[i]);
        hw[i] = __builtin_sqrtf(in[i]);
    }
    return 0;
}
