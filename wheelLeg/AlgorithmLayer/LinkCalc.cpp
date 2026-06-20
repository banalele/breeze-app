#include "LinkCalc.h"


using namespace std;

void FourBarLink::Coordinate_Cal()
{
    float phi1 = link_angle.phi1;
    float phi4 = link_angle.phi4;
    float phi1_d1 = link_angle.phi1_d1;
    float phi4_d1 = link_angle.phi4_d1;
    // A点坐标
    coord_.xa = 0.0f;
    coord_.ya = 0.0f;

    // E点坐标
    coord_.xe = L5;
    coord_.ye = 0.0f;

    // B点坐标
    coord_.xb = L1 * cos(phi1);
    coord_.yb = L1 * sin(phi1);
    coord_.xb_d1 = -L1 * sin(phi1) * phi1_d1;
    coord_.yb_d1 = L1 * cos(phi1) * phi1_d1;

    // D点坐标
    coord_.xd = L1 * cos(phi4) + L5;
    coord_.yd = L1 * sin(phi4);
    coord_.xd_d1 = -L1 * sin(phi4) * phi4_d1;
    coord_.yd_d1 = L1 * cos(phi4) * phi4_d1;
}

void FourBarLink::Phi2_3_Cal()
{
    float xd = coord_.xd, yd = coord_.yd;
    float xd_d1 = coord_.xd_d1, yd_d1 = coord_.yd_d1;
    float xb = coord_.xb, yb = coord_.yb;
    float xb_d1 = coord_.xb_d1, yb_d1 = coord_.yb_d1;

    float temp_A0 = 2.0f * L2 * (xd - xb);
    float temp_A0_d1 = 2.0f * L2 * (xd_d1 - xb_d1);
    float temp_B0 = 2.0f * L2 * (yd - yb);
    float temp_B0_d1 = 2.0f * L2 * (yd_d1 - yb_d1);

    length_.lbd = sqrt(std::pow(xd - xb, 2) + pow(yd - yb, 2));
    float temp_C0 = length_.lbd * length_.lbd;
    float temp_C0_d1 = 2.0f * (xd - xb) * (xd_d1 - xb_d1) +
                       2.0f * (yd - yb) * (yd_d1 - yb_d1);

    float temp_D0 = std::sqrt(std::pow(temp_A0, 2) + std::pow(temp_B0, 2) - std::pow(temp_C0, 2));
    float temp_D0_d1 = ((temp_A0 * temp_A0_d1) + (temp_B0 * temp_B0_d1) - (temp_C0 * temp_C0_d1)) / temp_D0;

    float temp_2phi2 = std::atan2((temp_B0 + temp_D0), (temp_A0 + temp_C0));
    angle_.phi2 = temp_2phi2 * 2.0f;

    angle_.phi2_d1 = 2.0f * ((temp_A0 + temp_C0) * (temp_B0_d1 + temp_D0_d1) - (temp_A0_d1 + temp_C0_d1) * (temp_B0 + temp_D0)) /
                     (std::pow(temp_A0 + temp_C0, 2) + std::pow(temp_B0 + temp_D0, 2));

    angle_.phi3 = std::atan2((yb - yd) + L2 * std::sin(angle_.phi2),
                             (xb - xd) + L2 * std::cos(angle_.phi2));
}
