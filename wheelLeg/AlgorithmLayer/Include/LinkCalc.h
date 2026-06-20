#ifndef LINKCALC_H
#define LINKCALC_H

#include "matrix.h"
#define Rad2Angle 57.2957804f

// 前向声明
class FourBarLink;
/*连杆坐标信息*/
struct Link_coord_t
{
public:
    float xa;
    float ya;

    float xe;
    float ye;

    float xb;
    float yb;

    float xb_d1;
    float yb_d1;

    float xc;
    float yc;

    float xc_d1;
    float yc_d1;

    float xd;
    float yd;

    float xd_d1;
    float yd_d1;

    float xp;
    float yp;
};

    struct Link_angle_t
    {
        public:
            /*弧度制*/
            float phi1;

            float phi1_d1;

            float phi2;

            float phi2_d1;

            float phi3;

            float phi4;

            float phi4_d1;

            float phi0;

            float phi0_d1;

            float good_phi0_d1;

            float vir_phi0;

            float vir_phi0_d1;

            /*目标角度*/
            float target_phi1;

            float target_phi4;

            /*角度制*/
            float phi1_;

            float phi2_;

            float phi3_;

            float phi4_;

            float phi0_;

            float phi0_last;

            float vir_phi0_;
    };

    /*等效直腿腿长信息*/
    struct Link_leg_length_t
    {
        float lbd;

        float l0;
        float l0_last;
        float l0_dot;
        float l0_dot_last;
        float l0_dot2;
        float l0_dot2_last;

        float good_l0_dot;
        float l_gravity;//质心系数，需要自行调整
    };

    /*杆的质心位置信息*/
    struct Link_centoird_t
    {
        float mx_l1;
        float my_l1;
        float mx_l2;
        float my_l2;
        float mx_l3;
        float my_l3;
        float mx_l4;
        float my_l4;
        float centriod_coffe;
    };

    /*等效直腿受力信息*/
    struct Link_force_t
    {
        //	float F_gravity;//重力补偿力
        //
        //  float F_inertial;//侧向惯性补偿力
        //
        //  float F_roll;//roll轴补偿力
        //
        //  float F;//保持腿长力,pid,伸腿为正

        float F_bl_target; // 合力,F+F_roll+F_inertial+F_gravity

        //	float Tp_sync;//双腿协调

        float Tp_target;

        //	float F_support;//支持力

        float G_torque;

        float G_support;

        float F_bl_mea;

        float Tp_mea;

        float Sd_F_Pos_Tor_Fix;

        float Sd_B_Pos_Tor_Fix;

        float torque_phi1_mea;

        float torque_phi4_mea;
    };

    // 主类：四连杆机构
    class FourBarLink
    {
        private:
            Link_coord_t link_coord;
            Link_angle_t link_angle;
            Link_leg_length_t link_leg_length;
            Link_centoird_t link_centroid;
            Link_force_t link_force;
            // 几何参数（根据实际情况定义）
            static constexpr float l1 = 0.1f; // 杆1长度
            static constexpr float l2 = 0.1f; // 杆2长度
            static constexpr float l3 = 0.1f; // 杆3长度
            static constexpr float l4 = 0.1f; // 杆4长度
            static constexpr float l5 = 0.2f; // 杆5长度（固定距离）

            // 质心参数
            static constexpr float l1_cen = 0.5f;
            static constexpr float l2_cen = 0.5f;
            static constexpr float l3_cen = 0.5f;
            static constexpr float l4_cen = 0.5f;
            static constexpr float M_l1 = 0.1f, M_l2 = 0.1f, M_l3 = 0.1f, M_l4 = 0.1f;

            float f_sd_output_torque = 0.0f;
            float b_sd_output_torque = 0.0f;

            void Coordinate_Cal();
            void Phi2_3_Cal();
            void C_Cal();
            void Led_Length_Cal();
            void Phi0_Cal();
            void Centroid_Cal();
            void Link_Debug();

            // 辅助函数
            static float lowpass(float last, float current, float alpha)
            {
                return alpha * current + (1.0f - alpha) * last;
            }

            public:
                FourBarLink();
                ~FourBarLink() = default;

                void update_
    }

#endif // LINKCALC_H
