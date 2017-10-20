#include <Arduino.h>
#define STATES_COUNT 100
#define COMMANDS_COUNT 10

struct MotionState
{
    bool dirX;
    bool dirY;
    double e2;
    double err;
    double dx;
    double dy;
    long tx;
    long ty;
    uint8_t power;
    uint8_t speed;
    uint8_t accel;
};

MotionState _states[STATES_COUNT];
MotionState _state;
uint8_t _statesIndex = 0;

String commands[COMMANDS_COUNT];
int8_t _cmdsIndex = 0;

int8_t _power = 0;
int16_t _speed = 0;
int16_t _accel = 0;
long _positionX = 0;
long _positionY = 0;
long _targetX = 0;
long _targetY = 0;
long _lastX = 0;
long _lastY = 0;
String _buffer = "";
void setup()
{
    Serial.begin(115200);
}

void loop()
{
    if (Serial.available())
    {
        char c = Serial.read();
        parseBuffer(c);
    }
    run();
}
void parseBuffer(char c)
{
    if (c == '\n')
    {
        pushCommand(_buffer);
        _buffer = "";
    }
    else
    {
        _buffer += c;
    }
}
void run()
{
    if (positionToGo())
    {
        step();
    }
    else
    {
        nextState();
    }
}
void nextState(){
    if(_statesIndex>0){
        _state = _states[0];
        setPower(_state.power);
        setSpeed(_state.speed);
        setAccel(_state.accel);
        for(int i=0;i<_statesIndex-1;i++){
            _states[i] = _states[i+1];
        }
        _statesIndex--;
    }else{
        nextCommand();
    }
}
void pushCommand(String cmd)
{
    commands[_cmdsIndex] = cmd;
    _cmdsIndex++;
}
void nextCommand()
{
    if(_cmdsIndex>0){
        
    }
    String cmd = commands[0];
    for (int i = 0; i < _cmdsIndex-1; i++)
    {
        commands[i] = commands[i + 1];
    }
    _cmdsIndex--;
    parseCommand(cmd);
}
void parseCommand(String cmd)
{
    cmd.toLowerCase();
    char code = cmd.charAt(0);
    long x1, y1, x2, y2, x3, y3, x4, y4;
    switch (code)
    {
    case 'm':
    {
        moveTo(x1, y1);
        break;
    }
    case 'l':
    {

        lineTo(x1, y1);
        break;
    }
    case 'h':
    {
        lineTo(x1, _targetY);
        break;
    }
    case 'v':
    {
        lineTo(_targetX, y1);
        break;
    }
    case 'c':
    {
        curveTo(x1, y1, x2, y2, x3, y3, x4, y4);
        break;
    }
    case 'q':
    {

        quadTo(x1, y1, x2, y2, x3, y3);
        break;
    }
    case 'z':
    {
        lineTo(_lastX, _lastY);
        break;
    }
    case 'p':
    {
        setPower(0);
        setSpeed(0);
        setAccel(0);
        finishCommand();
        break;
    }
    }
}

void finishCommand()
{
    Serial.print("ok ");
    Serial.println(commandsLength());
}
int8_t commandsLength()
{
    return _cmdsIndex + 1;
}
void setPower(uint8_t power)
{
    _states[_statesIndex].power = power;
    Serial.print("power: ");
    Serial.println(power);
}
void setSpeed(uint8_t speed)
{
    _states[_statesIndex].speed = speed;
    Serial.print("speed: ");
    Serial.println(speed);
}
void setAccel(uint8_t accel)
{
    _states[_statesIndex].accel = accel;
    Serial.print("accel: ");
    Serial.println(accel);
}
void openPower()
{
    Serial.println("openPower");
}
void closePower()
{
    Serial.println("closePower");
}
/**
 * motion control
 */
#define CURVE_SECTION 0.5
#define NUM_STEPS 20
bool positionToGo()
{
    long distX = xPositionToGo(_targetX);
    long distY = yPositionToGo(_targetY);
    return (distX != 0 || distY != 0);
}
void step()
{
    _targetX = _state.tx;
    _targetY = _state.ty;
    long distX = xPositionToGo(_targetX);
    long distY = yPositionToGo(_targetY);
    if (distX == 0 && distY == 0)
        return;
    _state.e2 = _state.err;
    if (_state.e2 > -_state.dx)
    {
        _state.err -= _state.dy;
        stepX(_state.dirX);
    }
    if (_state.e2 < _state.dy)
    {
        _state.err += _state.dx;
        stepY(_state.dirY);
    }
    wait();
}
void stepX(bool dir)
{
    _positionX += dir ? 1 : -1;
}
void stepY(bool dir)
{
    _positionY += dir ? 1 : -1;
}
void moveTo(long x, long y)
{
    closePower();
    _lastX = x;
    _lastY = y;
    linearInterpolation(_positionX, _positionY, x, y);
}
void lineTo(long x, long y)
{
    openPower();
    linearInterpolation(_positionX, _positionY, x, y);
}
long xPositionToGo(long tx)
{
    return tx - _positionX;
}
long yPositionToGo(long ty)
{
    return ty - _positionY;
}
void wait()
{
    //
}
void linearInterpolation(long x1, long y1, long x2, long y2)
{
    long dx, dy;
    bool sx, sy;
    MotionState newState;
    newState.dx = abs(x2 - x1);
    newState.dy = abs(y2 - y1);
    newState.dirX = x1 < x2;
    newState.dirY = y1 < y2;
    newState.err = (dx > dy ? dx : -dy) / 2;
    newState.tx = x2;
    newState.ty = y2;
    if (_statesIndex > STATES_COUNT-1)
    {
        for(int i=0;i<STATES_COUNT;i++){
            _states[i]=_states[i+1];
        }
    }
    _states[_statesIndex] = newState;
    _statesIndex++;
}
void curveTo(long x1, long y1, long x2, long y2, long x3, long y3, long x4, long y4)
{
    double dx1 = x2 - x1;
    double dy1 = y2 - y1;
    double dx2 = x3 - x2;
    double dy2 = y3 - y2;
    double dx3 = x4 - x3;
    double dy3 = y4 - y3;
    double subdiv_step = 1.0 / (NUM_STEPS + 1);
    double subdiv_step2 = subdiv_step * subdiv_step;
    double subdiv_step3 = subdiv_step * subdiv_step * subdiv_step;
    double pre1 = 3.0 * subdiv_step;
    double pre2 = 3.0 * subdiv_step2;
    double pre4 = 6.0 * subdiv_step2;
    double pre5 = 6.0 * subdiv_step3;
    double tmp1x = x1 - x2 * 2.0 + x3;
    double tmp1y = y1 - y2 * 2.0 + y3;
    double tmp2x = (x2 - x3) * 3.0 - x1 + x4;
    double tmp2y = (y2 - y3) * 3.0 - y1 + y4;
    double fx = x1;
    double fy = y1;
    double dfx = (x2 - x1) * pre1 + tmp1x * pre2 + tmp2x * subdiv_step3;
    double dfy = (y2 - y1) * pre1 + tmp1y * pre2 + tmp2y * subdiv_step3;
    double ddfx = tmp1x * pre4 + tmp2x * pre5;
    double ddfy = tmp1y * pre4 + tmp2y * pre5;
    double dddfx = tmp2x * pre5;
    double dddfy = tmp2y * pre5;
    int step = NUM_STEPS;
    while (step--)
    {
        fx += dfx;
        fy += dfy;
        dfx += ddfx;
        dfy += ddfy;
        ddfx += dddfx;
        ddfy += dddfy;
        lineTo(fx, fy);
    }
    lineTo(x4, y4);
}
void quadTo(long x1, long y1, long x2, long y2, long x3, long y3)
{
    int step = NUM_STEPS;
    double fx, fy;
    for (int i = 0; i < step; i++)
    {
        float t = i / step;
        float u = 1 - t;
        fx = x1 * u * u + x2 * 2 * u * t + x3 * t * t;
        fy = y1 * u * u + y2 * 2 * u * t + y3 * t * t;
        lineTo(fx, fy);
    }
    lineTo(x3, y3);
}

void arc(long sx, long sy, long cx, long cy, long ex, long ey, bool ccw)
{
    double angleA, angleB, angle, radius, length, aX, aY, bX, bY;
    aX = (sx - cx);
    aY = (sy - cy);
    bX = (ex - cx);
    bY = (ey - cy);
    if (ccw)
    {
        angleA = atan2(aY, aX);
        angleB = atan2(bY, bX);
    }
    else
    {
        angleA = atan2(bY, bX);
        angleB = atan2(aY, aX);
    }
    if (angleB <= angleA)
    {
        angleB += 2 * M_PI;
    }
    angle = angleB - angleA;
    radius = sqrt(aX * aX + aY * aY);
    length = radius * angle;
    int steps, s, step;
    steps = (int)ceil(length / CURVE_SECTION);
    for (s = 1; s <= steps; s++)
    {
        step = (ccw) ? s : (steps - s);
        float t = step / steps;
        lineTo(cx + radius * cos(angleA + angle * t), cy + radius * sin(angleA + angle * t));
    }
}