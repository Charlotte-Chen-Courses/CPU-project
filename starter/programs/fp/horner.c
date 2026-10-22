// horner.c: a degree-12 polynomial at 200 points (a chain of dependent multiply-adds: latency).
#define NP 200
#define DEG 12
float coef[DEG + 1], out[NP];

int main(void) {
    for (int k = 0; k <= DEG; k++) coef[k] = 1.0f / (float)(k + 1) * ((k & 1) ? -1.0f : 1.0f);
    for (int i = 0; i < NP; i++) {
        float t = (float)i / (float)NP * 2.0f - 1.0f;
        float r = coef[DEG];
        for (int k = DEG - 1; k >= 0; k--) r = r * t + coef[k];
        out[i] = r;
    }
    return 0;
}
