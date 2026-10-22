// saxpy.c: y = a*x + y over 256 floats, 4 passes (independent FP work: throughput).
#define N 256
float x[N], y[N];
volatile float sink;

int main(void) {
    for (int i = 0; i < N; i++) { x[i] = (float)i * 0.5f - 37.25f; y[i] = (float)(N - i) * 0.125f; }
    for (int p = 0; p < 4; p++) {
        float a = 1.5f + (float)p * 0.25f;
        for (int i = 0; i < N; i++) y[i] = a * x[i] + y[i];
    }
    float s = 0.0f;
    for (int i = 0; i < N; i++) s += y[i];
    sink = s;
    return 0;
}
