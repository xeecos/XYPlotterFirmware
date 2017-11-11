#include <Arduino.h>
#include <MeStepper.h>
#define SBI(n, b) (n |= _BV(b))
#define CBI(n, b) (n &= ~_BV(b))
#define POINTS_COUNT 100
#define PULSES_PER_MM 100
#define CURVE_SECTION 0.5
#define NUM_STEPS 10
#define NUM_AXIS 2
// #define DEBUG true
enum MOTION_STATE
{
    MOTION_ACCELETING = 0,
    MOTION_NORMAL = 1,
    MOTION_DECELETING = 2
};
typedef struct
{
    volatile long err;
    volatile double delta[NUM_AXIS];
    volatile bool dir[NUM_AXIS];
    volatile long accelCount = 0;
    volatile long decelCount = 0;
    volatile long targetPosition[NUM_AXIS];
    volatile long targetCount[NUM_AXIS];
    volatile long totalCount = 0;
    volatile uint16_t speed = 1000;
    volatile uint16_t exitSpeed = 1000;
    volatile uint16_t acceleration = 1;
    volatile uint8_t power = 0;
} PlannerPoint;
typedef struct
{
    volatile long currentPosition[NUM_AXIS] = {0, 0};
    volatile long targetPosition[NUM_AXIS];
    volatile long startPosition[NUM_AXIS];
    volatile long lastPosition[NUM_AXIS] = {0, 0};
    volatile long defaultAcceleration = 64;
    volatile long defaultAccelCount = 200;
    volatile long defaultDecelCount = 200;
    volatile uint16_t defaultSpeed = 1000;
    volatile long currentSpeed = 100;
    volatile uint8_t power = 0;
    volatile uint16_t speed = 1000;
    volatile uint16_t acceleration = 100;
    volatile uint8_t plannedLength = 0;
    volatile uint8_t plannedIndex = 0;
    volatile uint8_t isRunningState = MOTION_ACCELETING;
    volatile bool isRelative = false;
    uint8_t powerPin = 10;
    String buffer;
    volatile int bufferIndex = 0;
    volatile long during = 1000;
    volatile long currentTime = 0;
} SystemStatus;
static SystemStatus _sys;
static PlannerPoint _points[POINTS_COUNT];
static PlannerPoint _prevPoint;
volatile bool busy = false;
volatile bool isbusy = false;
MeStepper steppers[NUM_AXIS] = {MeStepper(SLOT_1), MeStepper(SLOT_2)};

void setup()
{
    Serial.begin(250000);
    delay(1000);
    // initTimer();
    for (int i = 0; i < NUM_AXIS; i++)
    {
        steppers[i].setMicroStep(16);
        steppers[i].disableOutputs();
    }
    Serial.println("opened");
    setPower(0);
    moveTo(0, 0);
    finishCommand();
}

void loop()
{
    if (Serial.available())
    {
        char c = Serial.read();
        if (c == '\n')
        {
            finishCommand();
            // if (!isbusy)
            parseCommand(_sys.buffer);
            _sys.buffer = "";
            // _sys.bufferIndex = 0;
            // for (int i = 0; i < 128; i++)
            // {
            //     _sys.buffer[i] = 0;
            // }
        }
        else
        {
            _sys.buffer += c;
            // _sys.buffer[_sys.bufferIndex] = c;
            // _sys.bufferIndex++;
            // if (_sys.bufferIndex > 128)
            // {
            //     _sys.bufferIndex = 0;
            //     while (Serial.read() != '\n')
            //     {
            //         break;
            //     }
            //     Serial.println("error!!!!!");
            // }
        }
    }
    motion();
}
void parseCommands()
{
    isbusy = true;
    // cli();
    // String cmds;
    // int i = 0;
    // for (i = 0; i <= _sys.bufferIndex; i++)
    // {
    //     cmds += _sys.buffer[i];
    // }
    // int counter = 0;
    // int lastIndex = 0;
    // for (i = 0; i < cmds.length(); i++)
    // {
    //     if (cmds.charAt(i) == ';')
    //     {
    //         parseCommand(cmds.substring(lastIndex, i));
    //         finishCommand();
    //         lastIndex = i + 1;
    //     }
    // }
    // parseCommand(cmds.substring(lastIndex, i));
    // sei();
    isbusy = false;
    // if (_sys.plannedLength % 2 == 0)
}
void parseCommand(String cmd)
{
    cmd.toLowerCase();
    // Serial.println(cmd);
    int cmdId;
    if (cmd.length() > 2)
    {
        if (hasCommand(cmd, 'g'))
        {
            cmdId = getCommand(cmd, 'g');
            switch (cmdId)
            {
            case 0:
            case 1:
            {
                if (hasCommand(cmd, 'p') && cmdId == 1)
                {
                    setPower(getCommand(cmd, 'p'));
                }
                if (cmdId == 0)
                {
                    setPower(0);
                }
                if (hasCommand(cmd, 'f'))
                {
                    setSpeed(getCommand(cmd, 'f'));
                }
                if (hasCommand(cmd, 'a'))
                {
                    setAccel(getCommand(cmd, 'a'));
                }
                double x = (hasCommand(cmd, 'x') ? fmax(-30, fmin(30, getCommand(cmd, 'x'))) * PULSES_PER_MM : _sys.targetPosition[0]);
                double y = (hasCommand(cmd, 'y') ? fmax(-30, fmin(30, getCommand(cmd, 'y'))) * PULSES_PER_MM : _sys.targetPosition[1]);
                if (_sys.isRelative)
                {
                    x += _sys.targetPosition[0];
                    y += _sys.targetPosition[1];
                }
                if (cmdId == 0)
                {
                    moveTo(x, y);
                }
                else
                {
                    lineTo(x, y);
                }
                _sys.targetPosition[0] = x;
                _sys.targetPosition[1] = y;
            }
            break;
            case 28:
            {
                waitingForFinish();
                _sys.currentPosition[0] = 0;
                _sys.currentPosition[1] = 0;
            }
            break;
            case 90:
            {
                waitingForFinish();
                _sys.isRelative = false;
            }
            break;
            case 91:
            {
                waitingForFinish();
                _sys.isRelative = true;
            }
            break;
            }
        }
        else if (hasCommand(cmd, 'm'))
        {
            cmdId = getCommand(cmd, 'm');
            switch (cmdId)
            {
            case 1:
            {
                waitingForFinish();
                if (hasCommand(cmd, 'p') && getCommand(cmd, 'p') == 1)
                {
                    for (int i = 0; i < NUM_AXIS; i++)
                    {
                        steppers[i].enableOutputs();
                    }
                }
                else
                {
                    for (int i = 0; i < NUM_AXIS; i++)
                    {
                        steppers[i].disableOutputs();
                    }
                }
                break;
            }
            case 4:
            {
                waitingForFinish();
                if (hasCommand(cmd, 'p'))
                {
                    firePower(getCommand(cmd, 'p'));
                }
                else
                {
                    firePower(0);
                }
                break;
            }
            }
        }
    }
}

void addMotion(long x, long y, uint8_t power, uint16_t speed, uint16_t acceleration)
{
    _points[_sys.plannedIndex].targetPosition[0] = x;
    _points[_sys.plannedIndex].targetPosition[1] = y;
    _points[_sys.plannedIndex].dir[0] = _prevPoint.targetPosition[0] < _points[_sys.plannedIndex].targetPosition[0];
    _points[_sys.plannedIndex].dir[1] = _prevPoint.targetPosition[1] < _points[_sys.plannedIndex].targetPosition[1];
    _points[_sys.plannedIndex].targetCount[0] = abs(_points[_sys.plannedIndex].targetPosition[0] - _prevPoint.targetPosition[0]);
    _points[_sys.plannedIndex].targetCount[1] = abs(_points[_sys.plannedIndex].targetPosition[1] - _prevPoint.targetPosition[1]);
    _points[_sys.plannedIndex].delta[0] = (_points[_sys.plannedIndex].targetCount[0]);
    _points[_sys.plannedIndex].delta[1] = (_points[_sys.plannedIndex].targetCount[1]);
    _points[_sys.plannedIndex].totalCount = _points[_sys.plannedIndex].delta[0] + _points[_sys.plannedIndex].delta[1];
    _points[_sys.plannedIndex].speed = speed;
    if (_sys.plannedIndex > 0)
    {
        _points[_sys.plannedIndex - 1].exitSpeed = _points[_sys.plannedIndex].speed;
    }
    _points[_sys.plannedIndex].acceleration = acceleration;
    long dist = sqrt(_points[_sys.plannedIndex].delta[0] * _points[_sys.plannedIndex].delta[0] + _points[_sys.plannedIndex].delta[1] * _points[_sys.plannedIndex].delta[1]) / 2;
    _points[_sys.plannedIndex].accelCount = _points[_sys.plannedIndex].totalCount - min(dist, _sys.defaultAccelCount);
    _points[_sys.plannedIndex].decelCount = 3; //min(dist, _sys.defaultDecelCount);
    _points[_sys.plannedIndex].power = power;
    _points[_sys.plannedIndex].err = (_points[_sys.plannedIndex].delta[0] > _points[_sys.plannedIndex].delta[1] ? _points[_sys.plannedIndex].delta[0] : -_points[_sys.plannedIndex].delta[1]) / 2;

    _prevPoint = _points[_sys.plannedIndex];
    _sys.plannedIndex += 1;
    _sys.plannedLength += 1;
}
void finishCommand()
{
    Serial.print("ok:");
    if (commandsLength() < 1)
    {
        closePower();
    }
    Serial.println(commandsLength());
}
int8_t commandsLength()
{
    return _sys.plannedLength;
}
void setPower(uint8_t power)
{
    _sys.power = power;
}
void setSpeed(uint16_t speed)
{
    _sys.speed = speed;
}
void setAccel(uint16_t accel)
{
    _sys.acceleration = accel;
}
void firePower(uint8_t power)
{
    pinMode(_sys.powerPin, OUTPUT);
    analogWrite(_sys.powerPin, power);
}
void openPower()
{
    firePower(_sys.power);
}
void closePower()
{
    firePower(0);
}
void shiftPlanned()
{
    if (_sys.plannedLength > 0)
    {
        for (int i = 0; i < _sys.plannedLength - 1; i++)
        {
            _points[i] = _points[i + 1];
        }
        _sys.plannedIndex -= 1;
        _sys.plannedLength -= 1;
    }
}
uint8_t isRunningState()
{
    return _sys.isRunningState;
}
void motionFinish()
{
    if (_sys.plannedLength > 0)
    {
        shiftPlanned();
        if (_sys.plannedLength > 0)
        {
            firePower(_points[0].power);
        }
#ifdef DEBUG
        String str = "finish:";
        str += _sys.plannedLength;
        Serial.println(str);
#endif
    }
}
void waitingForBusy()
{
    for (;;)
    {
        if (_sys.plannedLength < POINTS_COUNT)
        {
            return;
        }
    }
}
void waitingForFinish()
{
    for (;;)
    {
        if (_sys.plannedLength < 1)
        {
            return;
        }
    }
}
void initTimer()
{
    cli();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 100;
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS11);
    TIMSK1 |= (1 << OCIE1A);
    sei();
}
int tt = 0;
// ISR(TIMER1_COMPA_vect)
void motion()
{
    if (micros() - _sys.currentTime >= _sys.during)
    {
        // if (busy)
        // {
        //     return;
        // }
        // busy = true;
        run();
        // busy = false;
        _sys.currentTime = micros();
    }
}
void run()
{
    if (!motionStep())
    {
        motionFinish();
    }
}
bool motionStep()
{
    if (_sys.plannedLength < 1)
    {
        return false;
    }
    // firePower(_points[0].power);
    long distX = _points[0].delta[0];
    long distY = _points[0].delta[1];
    if (distX <= 0 && distY <= 0)
        return false;
    long e2 = _points[0].err;
    if (e2 > -_points[0].delta[0])
    {
        _points[0].err -= _points[0].delta[1];
        stepX(_points[0].dir[0]);
    }
    if (e2 < _points[0].delta[1])
    {
        _points[0].err += _points[0].delta[0];
        stepY(_points[0].dir[1]);
    }
    nextWait();
    return true;
}
void stepX(bool dir)
{
    _sys.currentPosition[0] += dir ? 1 : -1;
    _points[0].delta[0]--;
    _points[0].totalCount--;
    steppers[0].step(dir);
}
void stepY(bool dir)
{
    _sys.currentPosition[1] += dir ? 1 : -1;
    _points[0].delta[1]--;
    _points[0].totalCount--;
    steppers[1].step(dir);
}
void nextWait()
{
    if (_points[0].totalCount > _points[0].accelCount)
    {
        _sys.isRunningState = MOTION_ACCELETING;
    }
    else if (_points[0].totalCount > _points[0].decelCount)
    {
        _sys.isRunningState = MOTION_NORMAL;
    }
    else
    {
        _sys.isRunningState = MOTION_DECELETING;
    }
    switch (_sys.isRunningState)
    {
    case MOTION_ACCELETING:
    {
        if (_sys.currentSpeed < _points[0].speed)
        {
            _sys.currentSpeed += _points[0].acceleration;
            if (_sys.currentSpeed > _points[0].speed)
            {
                _sys.currentSpeed = _points[0].speed;
            }
        }
        break;
    }
    case MOTION_DECELETING:
    {
        if (_sys.currentSpeed > _points[0].exitSpeed)
        {
            _sys.currentSpeed -= _points[0].acceleration;
            if (_sys.currentSpeed < _points[0].exitSpeed)
            {
                _sys.currentSpeed = _points[0].exitSpeed;
            }
        }
        break;
    }
    case MOTION_NORMAL:
    {
        // OCR1A;
        break;
    }
    }
    tt = 0;
    _sys.during = 1600000 / _sys.currentSpeed - 1;
    _sys.during = _sys.during > 10000 ? 10000 : (_sys.during < 10 ? 10 : _sys.during);
}
/**
 * Motion Parse
 * */

void moveTo(long x, long y)
{
    waitingForBusy();
#ifdef DEBUG
    String str = "move:";
    str += x;
    str += ",";
    str += y;
    Serial.println(str);
#endif
    _sys.startPosition[0] = x;
    _sys.startPosition[1] = y;
    addMotion(x, y, 0, _sys.speed, _sys.acceleration);
}
void lineTo(long x, long y)
{
    waitingForBusy();
#ifdef DEBUG
    String str = "line:";
    str += x;
    str += ",";
    str += y;
    str += ",";
    str += _sys.speed;
    Serial.println(str);
#endif
    addMotion(x, y, _sys.power, _sys.speed, _sys.acceleration);
}

bool hasCommand(String cmd, char cmdId)
{
    int index = 0;
    char c = 0;
    while (c != cmdId)
    {
        if (index >= cmd.length())
        {
            return false;
        }
        else
        {
            c = cmd.charAt(index);
        }
        index++;
    }
    return true;
}
double getCommand(String cmd, char cmdId)
{
    int index = cmd.indexOf(cmdId);
    int startIndex = cmd.indexOf(cmdId) + 1;
    char c = 0;
    while (c != ' ')
    {
        index++;
        if (index >= cmd.length())
        {
            break;
        }
        else
        {
            c = cmd.charAt(index);
        }
    }
    return cmd.substring(startIndex, index).toDouble();
}