#include "28BYJ.h"
SP28BYJ::SP28BYJ(int pin1, int pin2, int pin3, int pin4)
{
    _p1 = pin1;
    _p2 = pin2;
    _p3 = pin3;
    _p4 = pin4;
    pinMode(_p1, OUTPUT);
    pinMode(_p2, OUTPUT);
    pinMode(_p3, OUTPUT);
    pinMode(_p4, OUTPUT);
    _targetPosition = 0;
    _currentPosition = 0;
    _time = 12000;
    _dir = 0;
    _back = 0;
}
void SP28BYJ::setSpeed(int speed)
{
    if (speed < 0)
    {
        speed = -speed;
    }
    _time = 1200 + 50000 / speed;
}
void SP28BYJ::step(bool dir)
{
    _currentStep += dir ? -1 : 1;
    if (_currentStep > 7)
    {
        _currentStep = 0;
    }
    if (_currentStep < 0)
    {
        _currentStep = 7;
    }
    switch (_currentStep)
    {
    case 0:
    {
        digitalWrite(_p1, HIGH);
        digitalWrite(_p2, LOW);
        digitalWrite(_p3, LOW);
        digitalWrite(_p4, LOW);
    }
    break;
    case 1:
    {
        digitalWrite(_p1, HIGH);
        digitalWrite(_p2, HIGH);
        digitalWrite(_p3, LOW);
        digitalWrite(_p4, LOW);
    }
    break;
    case 2:
    {
        digitalWrite(_p1, LOW);
        digitalWrite(_p2, HIGH);
        digitalWrite(_p3, LOW);
        digitalWrite(_p4, LOW);
    }
    break;
    case 3:
    {
        digitalWrite(_p1, LOW);
        digitalWrite(_p2, HIGH);
        digitalWrite(_p3, HIGH);
        digitalWrite(_p4, LOW);
    }
    break;
    case 4:
    {
        digitalWrite(_p1, LOW);
        digitalWrite(_p2, LOW);
        digitalWrite(_p3, HIGH);
        digitalWrite(_p4, LOW);
    }
    break;
    case 5:
    {
        digitalWrite(_p1, LOW);
        digitalWrite(_p2, LOW);
        digitalWrite(_p3, HIGH);
        digitalWrite(_p4, HIGH);
    }
    break;
    case 6:
    {
        digitalWrite(_p1, LOW);
        digitalWrite(_p2, LOW);
        digitalWrite(_p3, LOW);
        digitalWrite(_p4, HIGH);
    }
    break;
    case 7:
    {
        digitalWrite(_p1, HIGH);
        digitalWrite(_p2, LOW);
        digitalWrite(_p3, LOW);
        digitalWrite(_p4, HIGH);
    }
    break;
    }
}
void SP28BYJ::lock()
{
    step(true);
    step(false);
}
void SP28BYJ::setBack(int v)
{
    if (v >= 0)
    {
        _back = v;
    }
}
void SP28BYJ::moveTo(long position)
{
    // Serial.print(_p1);
    // Serial.print(":");
    // Serial.println(position-_currentPosition);
    _targetPosition = position;
}
void SP28BYJ::move(long position)
{
    moveTo(_targetPosition + position);
}
bool SP28BYJ::run()
{
    if (_targetPosition > _currentPosition)
    {
        step(true);
        _currentPosition += 1;
        return true;
    }
    else if (_targetPosition < _currentPosition)
    {
        step(false);
        _currentPosition += -1;
        return true;
    }
    return false;
}
void SP28BYJ::reset()
{
    _currentPosition = 0;
    _targetPosition = 0;
}
int SP28BYJ::getBack()
{
    return _back;
}
void SP28BYJ::setDir(bool v)
{
    _dir = v;
}
bool SP28BYJ::getDir()
{
    return _dir;
}