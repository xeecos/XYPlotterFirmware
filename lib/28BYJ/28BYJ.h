#ifndef _28BYJ_h
#define _28BYJ_h
#include <Arduino.h>
class SP28BYJ
{
  public:
    SP28BYJ(int pin1, int pin2, int pin3, int pin4);
    void setSpeed(int speed);
    void step(bool dir);
    void moveTo(long position);
    void move(long position);
    bool run();
    void reset();
    void setDir(bool v);
    bool getDir();
    void setBack(int v);
    int getBack();
    void lock();

  private:
    bool _dir;
    int _p1;
    int _p2;
    int _p3;
    int _p4;
    int _time;
    int _back;
    int _currentStep;
    long _currentPosition;
    long _targetPosition;
};
#endif