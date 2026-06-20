#include "zephyr/device.h"
#include "zephyr/drivers/sensor.h"
#include "zephyr/kernel.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(Straight_leg, LOG_LEVEL_INF);

#include "../AlgorithmLayer/Include/StraightLeg.h"

using namespace straightLeg;

int main()
{
	StraightLeg straight_leg;
	while (1)
	{
		straight_leg.state_info.update_state(0.1f, 0.01f, 0.2f, 0.02f, 0.05f, 0.005f);
		float state[6];
		straight_leg.state_info.get_state(state);
		LOG_INF("State: thetal=%.3f, thetald1=%.3f, s=%.3f, sd1=%.3f, thetab=%.3f, thetabd1=%.3f", state[0], state[1], state[2], state[3], state[4], state[5]);
		k_msleep(500);
	}
}
