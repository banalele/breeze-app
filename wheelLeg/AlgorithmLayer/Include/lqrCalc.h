#ifndef LQRCALC_H
#define LQRCALC_H

#include "matrix.h"
#include <cmath>
#include <cstring>

// -----------------------------------------------------------------
// 矩阵指数（泰勒级数近似）
// -----------------------------------------------------------------
template <int N>
Matrixf<N, N> matrix_exp(const Matrixf<N, N> &A, float dt)
{
    const int max_order = 10;
    const float tol = 1e-6f;
    Matrixf<N, N> result = matrixf::eye<N, N>();
    Matrixf<N, N> term = matrixf::eye<N, N>();
    Matrixf<N, N> A_dt = A * dt;

    for (int k = 1; k <= max_order; ++k)
    {
        term = term * A_dt;
        term = term * (1.0f / k);
        result = result + term;
        float norm = 0;
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j)
                norm += fabs(term[i][j]);
        if (norm < tol)
            break;
    }
    return result;
}

// -----------------------------------------------------------------
// 离散化（零阶保持）
// -----------------------------------------------------------------
template <int N, int M>
void discretize(const Matrixf<N, N> &A, const Matrixf<N, M> &B, float dt,
                Matrixf<N, N> &Ad, Matrixf<N, M> &Bd)
{
    Ad = matrix_exp(A, dt);
    Matrixf<N, N> I = matrixf::eye<N, N>();
    Matrixf<N, N> Ad_minus_I = Ad - I;
    Matrixf<N, N> A_inv = matrixf::inv(A);
    Bd = Ad_minus_I * A_inv * B;
    if (Bd.norm() < 1e-6f)
    {
        Bd = B * dt;
    }
}

// -----------------------------------------------------------------
// 离散时间代数黎卡提方程迭代求解
// -----------------------------------------------------------------
template <int N, int M>
void dare_iteration(const Matrixf<N, N> &A, const Matrixf<N, M> &B,
                    const Matrixf<N, N> &Q, const Matrixf<M, M> &R,
                    Matrixf<N, N> &P, Matrixf<M, N> &K,
                    int max_iter = 100, float tol = 1e-4f)
{
    P = Q;
    for (int iter = 0; iter < max_iter; ++iter)
    {
        Matrixf<M, M> S = R + B.trans() * P * B;
        Matrixf<M, M> S_inv = matrixf::inv(S);
        Matrixf<M, N> K_new = S_inv * (B.trans() * P * A);
        Matrixf<N, N> P_new = A.trans() * P * A - K_new.trans() * S * K_new + Q;
        float diff = (P_new - P).norm();
        P = P_new;
        if (diff < tol)
        {
            K = K_new;
            break;
        }
        if (iter == max_iter - 1)
        {
            K = K_new;
        }
    }
}

// -----------------------------------------------------------------
// 根据当前腿长 L 计算实时 LQR 增益 K (2x6)
// -----------------------------------------------------------------
Matrixf<2, 6> computeLQRGain(float L, float dt);

#endif // LQR_CONTROLLER_H
