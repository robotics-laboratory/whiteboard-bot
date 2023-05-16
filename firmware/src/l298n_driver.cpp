#include "l298n_driver.h"

void L298N_DRIVER::writeStatus()
{
    digitalWrite(_IN1, state[0]);
    digitalWrite(_IN2, state[1]);
    digitalWrite(_IN3, state[2]);
    digitalWrite(_IN4, state[3]);
}

void L298N_DRIVER::_updateStatus(bool fwd1, bool fwd2, bool fwd3, bool fwd4)
{
    state[0] = fwd1;
    state[1] = fwd2;
    state[2] = fwd3;
    state[3] = fwd4;
}

void L298N_DRIVER::update(int velocity_left, int velocity_right)
{
    if (velocity_left == 0 && velocity_right == 0)
    {
        _updateStatus(true, true, true, true);
        return;
    }
    if (velocity_left >= 0)
    {
        state[0] = true;
        state[1] = false;
    }
    else
    {
        state[0] = false;
        state[1] = true;
    }
    if (velocity_right >= 0)
    {
        state[2] = true;
        state[3] = false;
    }
    else
    {
        state[2] = false;
        state[3] = true;
    }
}