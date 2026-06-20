#ifndef STRAIGHTLEG_H
#define STRAIGHTLEG_H

#include "matrix.h"

namespace straightLeg {

    /*状态量枚举*/
    enum class X_state_e:int
    {
        X_thetal,
        X_thetald1,

        X_s,
        X_sd1,

        X_thetab,
        X_thetabd1,

        X_Num,
    };

    /*控制量枚举*/
    enum class U_enum_e:int
    {
        Tw,
        Tp,
        u_Num,
    } ;

    class State_info_t
    {
    public:
        State_info_t() : state_mat(), target_mat(), error_mat() ,thetal(0), thetald1(0), thetald2(0), s(0), sd1(0), thetab(0), thetabd1(0),
                         target_thetal(0), target_thetald1(0), target_s(0), target_sd1(0), target_thetab(0), target_thetabd1(0),
                         thetal_err(0), thetald1_err(0), s_err(0), sd1_err(0), thetab_err(0), thetabd1_err(0) {}

        State_info_t(const State_info_t& other) = default;
        ~State_info_t() = default;

        Matrixf<6, 1> state_mat;
        Matrixf<6, 1> target_mat;
        Matrixf<6, 1> error_mat;

        
        void update_state(float thetal, float thetald1, float s, float sd1, float thetab, float thetabd1)
        {
            this->thetal = thetal;
            this->thetald1 = thetald1;
            this->s = s;
            this->sd1 = sd1;
            this->thetab = thetab;
            this->thetabd1 = thetabd1;
        }


        void update_target(float target_thetal, float target_thetald1, float target_s, float target_sd1, float target_thetab, float target_thetabd1)
        {
            this->target_thetal = target_thetal;
            this->target_thetald1 = target_thetald1;
            this->target_s = target_s;
            this->target_sd1 = target_sd1;
            this->target_thetab = target_thetab;
            this->target_thetabd1 = target_thetabd1;
        }

        void update_error()
        {
            this->thetal_err = target_thetal - thetal;
            this->thetald1_err = target_thetald1 - thetald1;
            this->s_err = target_s - s;
            this->sd1_err = target_sd1 - sd1;
            this->thetab_err = target_thetab - thetab;
            this->thetabd1_err = target_thetabd1 - thetabd1;
        }

        void update_x_mat()
        {
            float state_data[6] = {thetal, thetald1, s, sd1, thetab, thetabd1};
            state_mat = Matrixf<6, 1>(state_data);
            float target_data[6] = {target_thetal, target_thetald1, target_s, target_sd1, target_thetab, target_thetabd1};
            target_mat = Matrixf<6, 1>(target_data);
            error_mat = state_mat - target_mat;
        }

        void get_state(float state[6]) const
        {
            state[0] = thetal;
            state[1] = thetald1;
            state[2] = s;
            state[3] = sd1;
            state[4] = thetab;
            state[5] = thetabd1;
        }

    private:
        float thetal; // 杆和竖直方向夹角，顺时针为正，l表示杆
        float thetald1;
        float thetald2;
        float s; // 位移，针对HGC模型的图，往右为正
        float sd1;

        float thetab; // 机体pitch角，往上为正，b表示机体
        float thetabd1;

        /*中间变量用 begin*/
        float s_now;

        float s_last;

        float thetal_now;

        float thetal_last;

        float thetald1_l_now;

        float thetald1_l_last;
        /*中间变量用 end*/
        float target_thetal; // 杆和竖直方向夹角，顺时针为正，l表示杆
        float target_thetald1;

        float target_s; // 位移，往右为正
        float target_sd1;

        float target_thetab; // 机体pitch角，往上为正，b表示机体
        float target_thetabd1;

        float thetal_err;   // 目标-测量
        float thetald1_err; // 目标-测量
        float s_err;        // 目标-测量
        float sd1_err;      // 目标-测量
        float thetab_err;   // 目标-测量
        float thetabd1_err; // 目标-测量
};



class K_matrix_t{
    public:
        float k_coeff[2][6] = {0};
        Matrixf<2, 6> k_mat;

        float k_fit_coeff[2][6][4] = {0};
        Matrixf<6, 4> K_fit_tw_mat; // Tw的多项式系数 (6个状态,4阶多项式)
        Matrixf<6, 4> K_fit_tp_mat; // Tp的多项式系数 (6个状态,4阶多项式)

        K_matrix_t() { init_k_mat(); }
        K_matrix_t(const K_matrix_t& other) = default;
        ~K_matrix_t() = default;

        void init_k_mat(){
            k_mat = Matrixf<2, 6>(&k_coeff[0][0]);
            K_fit_tw_mat = Matrixf<6, 4>(&k_fit_coeff[0][0][0]);
            K_fit_tp_mat = Matrixf<6, 4>(&k_fit_coeff[1][0][0]);
        }

        void update_k_mat(float l0){
            float l0_array[4] = {1.0f, l0, l0 * l0, l0 * l0 * l0};
            Matrixf<4, 1> l0_coeff_vec(l0_array);

            Matrixf<1, 6> K_Tw = (K_fit_tw_mat * l0_coeff_vec).trans();
            Matrixf<1, 6> K_Tp = (K_fit_tp_mat * l0_coeff_vec).trans();
            for (int i = 0; i < 6; i++)
            {
                k_coeff[0][i] = K_Tw[0][i];
                k_coeff[1][i] = K_Tp[0][i];
                k_mat[0][i] = K_Tw[0][i];
                k_mat[1][i] = K_Tp[0][i];
            }

        }
};

class Ex_leg_date_t{
    public:
        Ex_leg_date_t() : l0(0) {};
        ~Ex_leg_date_t() = default;
        float l0;

        void update_l0(float l0)
        {
            this->l0 = l0;
        }

        float get_l0(void)
        {
            return l0;
        }
};

class U_info_t{
    public:
        Matrixf<2, 1> u_mat;
        U_info_t() : u_mat() {}
        U_info_t(const U_info_t& other) = default;
        ~U_info_t() = default;

        void update_u(const Matrixf<2, 1>& u_matrix)
        {
            u[0] = u_matrix[0][0];  // Tw
            u[1] = u_matrix[1][0];  // Tp
        }

        float get_tw(){
            return u[0];
        }
        float get_tp(){
            return u[1];
        }
    private:
        float u[2] = {0}; // u[0]对应Tw，u[1]对应Tp
};

class StraightLeg {
    public:
        State_info_t state_info;
        K_matrix_t k_matrix;
        Ex_leg_date_t ex_leg_date;
        U_info_t u_info;

        StraightLeg(): state_info(), k_matrix(), ex_leg_date(), u_info() {}
        StraightLeg(const StraightLeg& other) = default;
        ~StraightLeg() = default;

        void lqr_calc(){
            state_info.update_x_mat();
            u_info.u_mat=k_matrix.k_mat * state_info.error_mat;
            u_info.update_u(u_info.u_mat);
        }

        float get_lqr_tw(){
            return u_info.get_tw();
        }

        float get_lqr_tp(){
            return u_info.get_tp();
        }
};
}




#endif  // STRAIGHTLEG_H
