#include <Arduino.h>

class L298N
{
  public:
    L298N(int in1, int in2, int in3, int in4) : _IN1(in1), _IN2(in2), _IN3(in3), _IN4(in4)
    {
        for (int i = 0; i < 4; i++)
            state[i] = false;
    };
    void update(int velocity_left, int velocity_right);
    void write();
  private:
    void _update(bool fwd1, bool fwd2, bool fwd3, bool fwd4);
    bool state[4];
    int _IN1;
    int _IN2;
    int _IN3;
    int _IN4;
};

void L298N::write()
{
    digitalWrite(_IN1, state[0]);
    digitalWrite(_IN2, state[1]);
    digitalWrite(_IN3, state[2]);
    digitalWrite(_IN4, state[3]);
}

void L298N::_update(bool fwd1, bool fwd2, bool fwd3, bool fwd4)
{
    state[0] = fwd1;
    state[1] = fwd2;
    state[2] = fwd3;
    state[3] = fwd4;
}

void L298N::update(int velocity_left, int velocity_right)
{
    if (velocity_left == 0 && velocity_right == 0)
    {
        _update(true, true, true, true);
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