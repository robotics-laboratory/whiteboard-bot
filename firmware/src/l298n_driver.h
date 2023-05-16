#include <Arduino.h>

#ifndef WHITEBOARD_BOT_L298N_DRIVER_H
#define WHITEBOARD_BOT_L298N_DRIVER_H

class L298N_DRIVER {
  public:
    L298N_DRIVER(int in1, int in2, int in3, int in4) : _IN1(in1), _IN2(in2), _IN3(in3), _IN4(in4)
    {
        for (int i = 0; i < 4; i++)
            state[i] = false;
    };
    void update(int velocity_left, int velocity_right);
    void writeStatus();
  private:
    void _updateStatus(bool fwd1, bool fwd2, bool fwd3, bool fwd4);
    bool state[4];
    int _IN1;
    int _IN2;
    int _IN3;
    int _IN4;
};

#endif  // WHITEBOARD_BOT_L298N_DRIVER_H
