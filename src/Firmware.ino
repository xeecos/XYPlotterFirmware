#include <Arduino.h>
#define STATES_COUNT 128
#define COMMANDS_COUNT 2
#define PULSES_PER_MM 100
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
long _startX = 0;
long _startY = 0;
long _lastX = 0;
long _lastY = 0;
bool _isFinish = true;
String _buffer = "";
uint16_t _currentDelays = 5000;
uint16_t _targetDelays = 0;
uint16_t _currentAccel = 10;
/**
 * Hardware Define
 * */
uint8_t _powerPin = 5;
void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("opened");
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
void nextState()
{
    if (_statesIndex > 0)
    {
        _state = _states[0];
        _targetDelays = 5000 / (1 + _state.speed * 49);
        _currentAccel = _state.accel;
        for (int i = 0; i < _statesIndex - 1; i++)
        {
            _states[i] = _states[i + 1];
        }
        _statesIndex--;
    }
    else
    {
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
    if (_cmdsIndex > 0)
    {
        _isFinish = false;
        String cmd = commands[0];
        for (int i = 0; i < _cmdsIndex - 1; i++)
        {
            commands[i] = commands[i + 1];
        }
        _cmdsIndex--;
        parseCommand(cmd);
    }
    else
    {
        if (!_isFinish)
        {
            _isFinish = true;
            finishCommand();
        }
        else
        {
            if (_currentDelays < 5000)
            {
                _currentDelays += 10;
            }
        }
    }
}
void parseCommand(String cmd)
{
    cmd.toLowerCase();
    char code = cmd.charAt(0);
    long x1, y1, x2, y2, x3, y3, x4, y4;
    static double v[8];
    String vStr = "";
    uint8_t vIndex = 0;
    uint8_t len = cmd.length();
    for (int i = 1; i < len; i++)
    {
        char c = cmd.charAt(i);
        if (c == ' ' || c == ',')
        {
            if (vIndex > 0)
            {
                v[vIndex - 1] = vStr.toDouble();
            }
            vStr = "";
            vIndex++;
        }
        else
        {
            vStr += c;
            if (vIndex > 0 && i == len - 1)
            {
                v[vIndex - 1] = vStr.toDouble();
            }
        }
    }
    switch (code)
    {
    case 'm':
    {
        x1 = v[0] * PULSES_PER_MM;
        y1 = v[1] * PULSES_PER_MM;
        moveTo(x1, y1);
        break;
    }
    case 'l':
    {
        x1 = v[0] * PULSES_PER_MM;
        y1 = v[1] * PULSES_PER_MM;
        lineTo(x1, y1);
        break;
    }
    case 'h':
    {
        x1 = v[0] * PULSES_PER_MM;
        lineTo(x1, _lastY);
        break;
    }
    case 'v':
    {
        y1 = v[0] * PULSES_PER_MM;
        lineTo(_lastX, y1);
        break;
    }
    case 'c':
    {
        x1 = v[0] * PULSES_PER_MM;
        y1 = v[1] * PULSES_PER_MM;
        x2 = v[2] * PULSES_PER_MM;
        y2 = v[3] * PULSES_PER_MM;
        x3 = v[4] * PULSES_PER_MM;
        y3 = v[5] * PULSES_PER_MM;
        curveTo(x1, y1, x2, y2, x3, y3);
        break;
    }
    case 'q':
    {
        x1 = v[0] * PULSES_PER_MM;
        y1 = v[1] * PULSES_PER_MM;
        x2 = v[2] * PULSES_PER_MM;
        y2 = v[3] * PULSES_PER_MM;
        quadTo(x1, y1, x2, y2);
        break;
    }
    case 'z':
    {
        lineTo(_startX, _startY);
        break;
    }
    case 'p':
    {
        setPower(v[0]);
        setSpeed(v[1]);
        setAccel(v[2]);
        finishCommand();
        break;
    }
    }
}

void finishCommand()
{
    Serial.print("ok:");
    Serial.println(commandsLength());
}
int8_t commandsLength()
{
    return _cmdsIndex + 1;
}
void setPower(uint8_t power)
{
    _states[_statesIndex].power = fmaxf(0.0, fminf(100.0, power));
}
void setSpeed(uint8_t speed)
{
    _states[_statesIndex].speed = fmaxf(0.0, fminf(100.0, speed));
}
void setAccel(uint8_t accel)
{
    _states[_statesIndex].accel = fmaxf(0.0, fminf(100.0, accel));
}
void openPower()
{
    pinMode(_powerPin, OUTPUT);
    analogWrite(_powerPin, _states[_statesIndex].power * 2.55);
}
void closePower()
{
    analogWrite(_powerPin, 0);
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
    _startX = x;
    _startY = y;
#ifdef DEBUG
    String str = "move:";
    str += x;
    str += ",";
    str += y;
    Serial.println(str);
#endif
    linearInterpolation(_lastX, _lastY, x, y);
}
void lineTo(long x, long y)
{
    openPower();
#ifdef DEBUG
    String str = "line:";
    str += x;
    str += ",";
    str += y;
    Serial.println(str);
#endif
    linearInterpolation(_lastX, _lastY, x, y);
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
    if (_currentDelays < _targetDelays)
    {
        _currentDelays += _currentAccel;
    }
    else if (_currentDelays > _targetDelays)
    {
        _currentDelays -= _currentAccel;
    }
    if (_currentDelays > 2000)
    {
        delay(floorf(_currentDelays / 1000));
        delayMicroseconds(_currentDelays % 1000);
    }
    else
    {
        delayMicroseconds(_currentDelays);
    }
}
void linearInterpolation(long x1, long y1, long x2, long y2)
{
#ifdef DEBUG
    String str = "linear:";
    str += x1;
    str += ",";
    str += y1;
    str += " ";
    str += x2;
    str += ",";
    str += y2;
    Serial.println(str);
#endif
    MotionState newState;
    newState.dx = abs(x2 - x1);
    newState.dy = abs(y2 - y1);
    newState.dirX = x1 < x2;
    newState.dirY = y1 < y2;
    newState.err = (newState.dx > newState.dy ? newState.dx : -newState.dy) / 2;
    newState.tx = x2;
    newState.ty = y2;
    _lastX = x2;
    _lastY = y2;
    _targetX = _lastX;
    _targetY = _lastY;
    if (_statesIndex > STATES_COUNT - 1)
    {
        for (int i = 0; i < STATES_COUNT; i++)
        {
            _states[i] = _states[i + 1];
        }
    }
    _states[_statesIndex] = newState;
    _statesIndex++;
}
void curveTo(long x1, long y1, long x2, long y2, long x3, long y3)
{
#ifdef DEBUG
    String str = "curve:";
    str += _lastX;
    str += ",";
    str += _lastY;
    str += " ";
    str += x1;
    str += ",";
    str += y1;
    str += " ";
    str += x2;
    str += ",";
    str += y2;
    str += " ";
    str += x3;
    str += ",";
    str += y3;
    Serial.println(str);
#endif
    long x0 = _lastX;
    long y0 = _lastY;
    double subdiv_step = 1.0 / (NUM_STEPS + 1);
    double subdiv_step2 = subdiv_step * subdiv_step;
    double subdiv_step3 = subdiv_step * subdiv_step * subdiv_step;
    double pre1 = 3.0 * subdiv_step;
    double pre2 = 3.0 * subdiv_step2;
    double pre4 = 6.0 * subdiv_step2;
    double pre5 = 6.0 * subdiv_step3;
    double tmp1x = x0 - x1 * 2.0 + x2;
    double tmp1y = y0 - y1 * 2.0 + y2;
    double tmp2x = (x1 - x2) * 3.0 - x1 + x3;
    double tmp2y = (y1 - y2) * 3.0 - y1 + y3;
    double fx = x0;
    double fy = y0;
    double dfx = (x1 - x0) * pre1 + tmp1x * pre2 + tmp2x * subdiv_step3;
    double dfy = (y1 - y0) * pre1 + tmp1y * pre2 + tmp2y * subdiv_step3;
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
    lineTo(x3, y3);
}
void quadTo(long x1, long y1, long x2, long y2)
{
#ifdef DEBUG
    String str = "quad:";
    str += _lastX;
    str += ",";
    str += _lastY;
    str += " ";
    str += x1;
    str += ",";
    str += y1;
    str += " ";
    str += x2;
    str += ",";
    str += y2;
    Serial.println(str);
#endif
    long x0 = _lastX;
    long y0 = _lastY;
    int step = NUM_STEPS;
    double fx, fy;
    for (int i = 0; i < step; i++)
    {
        float t = i / step;
        float u = 1 - t;
        fx = x0 * u * u + x1 * 2 * u * t + x2 * t * t;
        fy = y0 * u * u + y1 * 2 * u * t + y2 * t * t;
        lineTo(fx, fy);
    }
    lineTo(x2, y2);
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