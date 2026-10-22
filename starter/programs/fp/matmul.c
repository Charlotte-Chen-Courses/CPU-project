// matmul.c: C = A * B for 12x12 floats (loads, multiply-adds, stores).
#define N 12
float A[N][N], B[N][N], C[N][N];
volatile float sink;

int main(void) {
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) {
            A[i][j] = (float)(i * N + j) / 7.0f - 3.0f;
            B[i][j] = (float)((i + 2 * j) % 11) * 0.3f - 1.0f;
        }
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) {
            float acc = 0.0f;
            for (int k = 0; k < N; k++) acc += A[i][k] * B[k][j];
            C[i][j] = acc;
        }
    float t = 0.0f;
    for (int i = 0; i < N; i++) t += C[i][i];
    sink = t;
    return 0;
}
