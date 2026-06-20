#include "lqrCalc.h"

extern "C"
{
    void computeA_data(double L, double A_data[36]);
    void computeB_data(double L, double B_data[12]);
}
// -----------------------------------------------------------------
// 根据当前腿长 L 构建系统矩阵 A(6x6) 和 B(6x2)
// 用户需在此处填入从 MATLAB 推导出的实际表达式
// -----------------------------------------------------------------
void computeAB(float L, Matrixf<6, 6> &A, Matrixf<6, 2> &B)
{
    // ========== 固定物理参数 ==========
    const float g = 9.81f;
    const float R = 0.075f;
    const float l = 0.002f;
    const float Mw = 0.509f;
    const float Mp = 7.2f;
    const float M = 1.2f;

    // 转动惯量（依赖 L）
    float Iw = 0.5f * Mw * R * R;
    float Ip = (1.0f / 12.0f) * Mp * (2 * L) * (2 * L);
    float Im = (1.0f / 12.0f) * M * (0.122f * 0.122f + 0.182f * 0.182f);

    double A_d[36], B_d[12];
    computeA_data((double)L, A_d);
    computeB_data((double)L, B_d);

    float A_f[36], B_f[12];
    for (int i = 0; i < 36; ++i)
        A_f[i] = (float)A_d[i];
    for (int i = 0; i < 12; ++i)
        B_f[i] = (float)B_d[i];

    A = Matrixf<6, 6>(A_f);
    B = Matrixf<6, 2>(B_f);
}

// -----------------------------------------------------------------
// 计算 LQR 增益矩阵（供外部调用）
// -----------------------------------------------------------------
Matrixf<2, 6> computeLQRGain(float L, float dt)
{
    Matrixf<6, 6> A_cont;
    Matrixf<6, 2> B_cont;
    computeAB(L, A_cont, B_cont);

    Matrixf<6, 6> Q = matrixf::diag<6, 6>(
        Matrixf<6, 1>((float[]){500, 30, 20, 1, 5000, 1}));
    Matrixf<2, 2> R = matrixf::diag<2, 2>(
        Matrixf<2, 1>((float[]){50, 2.3f}));

    Matrixf<6, 6> Ad;
    Matrixf<6, 2> Bd;
    discretize(A_cont, B_cont, dt, Ad, Bd);

    Matrixf<6, 6> P;
    Matrixf<2, 6> K;
    dare_iteration(Ad, Bd, Q, R, P, K, 50, 1e-4f);
    return K;
}
