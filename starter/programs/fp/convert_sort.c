// convert_sort.c: int -> float conversions, insertion sort on floats (compares), back to int.
#define N 96
int   ints[N], back[N];
float vals[N];

int main(void) {
    unsigned s = 12345;
    for (int i = 0; i < N; i++) { s = s * 1103515245u + 12345u; ints[i] = (int)(s >> 8) - (1 << 22); }
    for (int i = 0; i < N; i++) vals[i] = (float)ints[i] / 3.0f;
    for (int i = 1; i < N; i++) {
        float v = vals[i]; int j = i - 1;
        while (j >= 0 && vals[j] > v) { vals[j + 1] = vals[j]; j--; }
        vals[j + 1] = v;
    }
    for (int i = 0; i < N; i++) back[i] = (int)vals[i];
    return 0;
}
