#include <Arduino.h>
#include <MeStepper.h>
#include <28BYJ.h>
#define POINTS_COUNT 10
#define PULSES_PER_MM 80 //166.66
#define CURVE_SECTION 0.5
#define NUM_STEPS 10
#define NUM_AXIS 2
#define MAKEBLOCK 1
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
    volatile uint16_t speed = 10;
    volatile uint16_t exitSpeed = 10;
    volatile uint16_t acceleration = 10;
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
    volatile uint16_t defaultSpeed = 10;
    volatile long currentSpeed = 10;
    volatile uint8_t power = 0;
    volatile uint16_t speed = 10;
    volatile uint16_t acceleration = 10;
    volatile uint8_t plannedLength = 0;
    volatile uint8_t plannedIndex = 0;
    volatile uint8_t isRunningState = MOTION_ACCELETING;
    volatile bool isRelative = false;
    uint8_t powerPin = 6;
    String buffer = "";
    String spiCallback = "";
    volatile int spiCallbackIndex = 0;
    volatile char spiBuffer[64];
    volatile int spiBufferIndex = 0;
    volatile int bufferIndex = 0;
    volatile long during = 1000;
    volatile long currentTime = 0;
    volatile bool isSpiReceiving = false;
    volatile bool isSpiProcessing = false;
} SystemStatus;
static SystemStatus _sys;
static PlannerPoint _points[POINTS_COUNT];
static PlannerPoint _prevPoint;
volatile bool busy = false;
volatile bool isbusy = false;
#ifdef MAKEBLOCK
MeStepper steppers[NUM_AXIS] = {MeStepper(1), MeStepper(2)};
#else
SP28BYJ steppers[NUM_AXIS] = {SP28BYJ(2, 3, 7, 8), SP28BYJ(9, 10, 11, 12)};
#endif
int t = 0;
long ttt = 0;
long timeout = 0;
void setup()
{
    Serial.begin(115200);
    delay(1000);
    pinMode(MISO, OUTPUT);
    // turn on SPI in slave mode
    SPCR |= _BV(SPE);
    // turn on interrupts
    SPCR |= _BV(SPIE);
    initTimer();
#ifdef MAKEBLOCK
    pinMode(47, OUTPUT);
    pinMode(48, OUTPUT);
    pinMode(6, OUTPUT);
    digitalWrite(48, LOW);
    delayMicroseconds(250);
    digitalWrite(47, HIGH);
    for (int i = 0; i < NUM_AXIS; i++)
    {
        steppers[i].setMicroStep(16);
        steppers[i].disableOutputs();
    }
#endif
    Serial.println("opened");
    setPower(0);
    moveTo(0, 0);
    for (;;)
    {
        // motion();
        if (_sys.isSpiProcessing)
        {
            _sys.isSpiReceiving = false;
            if (_sys.spiBufferIndex > 1)
            {
                _sys.spiBuffer[_sys.spiBufferIndex] = 0;
                _sys.buffer = "";
                for (int i = 0; i < _sys.spiBufferIndex; i++)
                {
                    _sys.buffer += _sys.spiBuffer[i];
                }
                _sys.spiBufferIndex = 0;
                Serial.println(_sys.buffer);
                parseCommand();
                _sys.buffer = "";
            }
            _sys.isSpiProcessing = false;
            _sys.isSpiReceiving = true;
            SPDR = 0x0;
            while (!(SPSR & (1 << SPIF)))
                ;
            SPDR = 0xff;
            while (!(SPSR & (1 << SPIF)))
                ;
            SPDR = 0xff;
            while (!(SPSR & (1 << SPIF)))
                ;
            _sys.isSpiReceiving = false;
            timeout = 0;
        }
        else
        {
            timeout++;
            delayMicroseconds(1);
            if (timeout > 10000)
            {
                _sys.isSpiProcessing = true;
                Serial.println("timeout!");
                timeout = 0;
                SPDR = 0x0;
                while (!(SPSR & (1 << SPIF)))
                    ;
                SPDR = 0xff;
                while (!(SPSR & (1 << SPIF)))
                    ;
            }
        }
    }
}
// double tttt = 0;
// long kkk = 0;
ISR(SPI_STC_vect)
{
    char c = SPDR;
    /*test speed
    if (tttt == 0)
    {
        tttt = micros() / 1000.0;
    }
    if (c > 9)
    {
        ttt++;
    }
    if (ttt >= 1024)
    {
        kkk++;
        float n = (micros() / 1000.0 - tttt) / 1000.0;
        Serial.print(ttt / n / 1024.0);
        Serial.print(":");
        Serial.print(kkk);
        Serial.print("\n");
        ttt = 0;
        tttt = micros() / 1000.0;
    }*/
    if (c > 0 && c < 0xff)
    {
        if (!_sys.isSpiProcessing)
        {
            _sys.isSpiReceiving = true;
        }
        if (_sys.spiBufferIndex < 64)
        {
            if (c == '\n')
            {
                _sys.isSpiProcessing = true;
            }
            else
            {
                _sys.spiBuffer[_sys.spiBufferIndex] = c;
                _sys.spiBufferIndex++;
            }
        }
        else
        {
            _sys.spiBufferIndex = 0;
        }
    }
    else
    {
    }
}
void loop()
{
}

void parseCommand()
{
    _sys.buffer.toLowerCase();
    // Serial.println(_sys.buffer);
    int cmdId;
    if (_sys.buffer.length() > 2)
    {
        if (hasCommand('g'))
        {
            cmdId = getCommand('g');
            switch (cmdId)
            {
            case 0:
            case 1:
            {
                if (hasCommand('p') && cmdId == 1)
                {
                    setPower(getCommand('p'));
                }
                if (cmdId == 0)
                {
                    setPower(0);
                }
                if (hasCommand('f'))
                {
                    setSpeed(getCommand('f'));
                }
                if (hasCommand('a'))
                {
                    setAccel(getCommand('a'));
                }
                double x = hasCommand('x') ? fmax(-10, fmin(60, getCommand('x'))) * PULSES_PER_MM : (_sys.isRelative ? 0 : _sys.targetPosition[0]);
                double y = hasCommand('y') ? fmax(-10, fmin(60, getCommand('y'))) * PULSES_PER_MM : (_sys.isRelative ? 0 : _sys.targetPosition[1]);
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
        else if (hasCommand('m'))
        {
            cmdId = getCommand('m');
            switch (cmdId)
            {
            case 1:
            {
                waitingForFinish();
#ifdef MAKEBLOCK
                if (hasCommand('p') && getCommand('p') == 1)
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
#endif
                break;
            }
            case 4:
            {
                waitingForFinish();
                if (hasCommand('p'))
                {
                    firePower(getCommand('p'));
                }
                else
                {
                    firePower(0);
                }
                break;
            }
            }
        }
        finishCommand();
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
    if (commandsLength() < 1)
    {
        // closePower();
    }
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
    cli(); //stop interrupts
    //set timer1 interrupt at 1kHz
    TCCR1A = 0; // set entire TCCR1A register to 0
    TCCR1B = 0; // same for TCCR1B
    TCNT1 = 0;  //initialize counter value to 0
    // set timer count for 1khz increments
    OCR1A = 1999; // = (16*10^6) / (1000*8) - 1
    //had to use 16 bit timer1 for this bc 1999>255, but could switch to timers 0 or 2 with larger prescaler
    // turn on CTC mode
    // TCCR1B |= (1 << WGM12);
    // Set CS11 bit for 8 prescaler
    // TCCR1B |= (1 << CS11);
    // TCCR1A = _BV(WGM10);
    TCCR1B |= (1 << WGM12);
    // Set CS11 bit for 8 prescaler
    TCCR1B |= (1 << CS11);
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
    sei(); //allow interrupts
}
int tt = 0;
ISR(TIMER1_COMPA_vect)
{
    if (_sys.isSpiProcessing)
    {
        motion();
    }
}
void motion()
{
    // if (micros() - _sys.currentTime >= _sys.during)
    // {
    // if (busy)
    // {
    //     return;
    // }
    // busy = true;
    run();
    // busy = false;
    //     _sys.currentTime = micros();
    // }
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
    if (e2 > -distX)
    {
        _points[0].err -= distY;
        stepX(_points[0].dir[0]);
    }
    if (e2 < distY)
    {
        _points[0].err += distX;
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
    // _sys.during = 160000 / (_sys.currentSpeed + 1);
    _sys.currentSpeed = _sys.currentSpeed > 5000 ? 5000 : (_sys.currentSpeed < 5 ? 5 : _sys.currentSpeed);
    long during = min(62500, ((2000000 / _sys.currentSpeed))) - 1;
    OCR1A = during;
    // _sys.during = _sys.during > 100000 ? 100000 : (_sys.during < 1500 ? 1500 : _sys.during);
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

bool hasCommand(char cmdId)
{
    int index = 0;
    char c = 0;
    while (c != cmdId)
    {
        if (index >= _sys.buffer.length())
        {
            return false;
        }
        else
        {
            c = _sys.buffer.charAt(index);
        }
        index++;
    }
    return true;
}
double getCommand(char cmdId)
{
    int index = _sys.buffer.indexOf(cmdId);
    int startIndex = _sys.buffer.indexOf(cmdId) + 1;
    char c = 0;
    while (c != ' ')
    {
        index++;
        if (index >= _sys.buffer.length())
        {
            break;
        }
        else
        {
            c = _sys.buffer.charAt(index);
        }
    }
    return _sys.buffer.substring(startIndex, index).toDouble();
}