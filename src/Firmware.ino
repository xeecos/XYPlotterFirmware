#include <Arduino.h>
#include <MeStepper.h>
#include <28BYJ.h>
#define MAKEBLOCK 1
// #define DEBUG true
#define POINTS_COUNT 6
#ifdef MAKEBLOCK
#define PULSES_PER_MM 200
#else
#define PULSES_PER_MM 60
#endif
#define CURVE_SECTION 0.5
#define NUM_STEPS 10
#define NUM_AXIS 3
enum MOTION_STATE
{
    MOTION_ACCELETING = 0,
    MOTION_NORMAL = 1,
    MOTION_DECELETING = 2
};
typedef struct
{
    volatile long err;
    volatile long delta[NUM_AXIS];
    volatile long restCount[NUM_AXIS];
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
    volatile bool prevDir[NUM_AXIS] = {0, 0};
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
    uint8_t powerPin = 10;
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
    volatile bool isReceiving = false;
} SystemStatus;
static SystemStatus _sys;
static PlannerPoint _points[POINTS_COUNT];
static PlannerPoint _prevPoint;
volatile bool busy = false;
volatile bool isbusy = false;
#ifdef MAKEBLOCK
MeStepper steppers[NUM_AXIS] = {MeStepper(1), MeStepper(2), MeStepper(3)};
#else
SP28BYJ steppers[NUM_AXIS] = {SP28BYJ(2, 3, 7, 8), SP28BYJ(9, 10, 11, 12)};
#endif
int t = 0;
long ttt = 0;
long timeout = 0;
void setup()
{
    Serial.begin(115200);
    Serial.println("start");
    delay(300);
    initTimer();
#ifdef MAKEBLOCK
    pinMode(10, OUTPUT);
    digitalWrite(10, LOW);
    delayMicroseconds(250);
    for (int i = 0; i < NUM_AXIS; i++)
    {
        steppers[i].setMicroStep(8);
        steppers[i].disableOutputs();
    }
#else
    steppers[0].setBack(0);
    steppers[1].setBack(0);
#endif
    setPower(0);
    setSpeed(500);
    setAccel(100);
    // firePower(80);
    // moveTo(0,0,0);
    // lineTo(0, 0,0);
    Serial.println("opened");
    // testRun();
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
    TCCR1B |= (1 << WGM12);
    // Set CS11 bit for 8 prescaler
    TCCR1B |= (1 << CS11);
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
#ifdef MAKEBLOCK
    // pinMode(MISO, OUTPUT);
    // SPCR |= _BV(SPE);
    // SPCR |= _BV(SPIE);
#endif
    TCCR0A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
    TCCR0B = _BV(CS22);
    OCR0A = 180;
    OCR0B = 50;
    sei(); //allow interrupts
    Serial.println(TCCR0A);
}
#ifdef MAKEBLOCK
int bw = 30;
void fireBlock(int speed, int power)
{
    firePower(power);
    for (int i = 0; i < bw; i++)
    {
        for (int j = 0; j < bw; j++)
        {
            stepX(i % 2 == 0);
            delayMicroseconds(800);
            for (int k = 0; k < speed; k++)
                delayMicroseconds(80);
        }
        for (int kk = 0; kk < 4; kk++)
        {
            stepY(1);
            delayMicroseconds(800);
            for (int k = 0; k < speed; k++)
                delayMicroseconds(80);
        }
    }
    firePower(0);
    for (int i = 0; i < bw * 4; i++)
    {
        stepY(0);
        delay(1);
    }
    delay(2000);
}
void moveBlock(int x, int y)
{
    bool dirX = x > 0;
    bool dirY = y > 0;
    x = x * (x > 0 ? 1 : -1);
    y = y * (y > 0 ? 1 : -1);
    for (int i = 0; i < bw * x; i++)
    {
        stepX(dirX);
        delay(1);
    }
    for (int i = 0; i < bw * y; i++)
    {
        stepY(dirY);
        delay(1);
    }
}
void testRun()
{
    int i, j;
    for (i = 0; i < NUM_AXIS; i++)
    {
        steppers[i].setMicroStep(16);
        steppers[i].enableOutputs();
    }
    for (i = 0; i < 32; i++)
    {
        for (j = 0; j < 32; j++)
        {
            fireBlock(j + 1, i * 2 + 80);
            moveBlock(2, 0);
        }
        moveBlock(-j * 2, 5);
    }
    moveBlock(0, -i * 4);
    firePower(0);
    for (int i = 0; i < NUM_AXIS; i++)
    {
        steppers[i].setMicroStep(16);
        steppers[i].disableOutputs();
    }
}
ISR(SPI_STC_vect)
{
    char c = SPDR;
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
#endif
void loop()
{
    if (Serial.available())
    {
        char c = Serial.read();
        _sys.isReceiving = true;
        if (c == '\n')
        {
            _sys.isReceiving = false;
            parseCommand();
            Serial.println("ok");
            _sys.buffer = "";
        }
        else
        {
            _sys.buffer += c;
        }
    }

#ifdef MAKEBLOCK
    // if (_sys.isSpiProcessing)
    // {
    //     _sys.isSpiReceiving = false;
    //     if (_sys.spiBufferIndex > 1)
    //     {
    //         _sys.spiBuffer[_sys.spiBufferIndex] = 0;
    //         _sys.buffer = "";
    //         for (int i = 0; i < _sys.spiBufferIndex; i++)
    //         {
    //             _sys.buffer += _sys.spiBuffer[i];
    //         }
    //         _sys.spiBufferIndex = 0;
    //         Serial.println(_sys.buffer);
    //         parseCommand();
    //         _sys.buffer = "";
    //     }
    //     _sys.isSpiProcessing = false;
    //     _sys.isSpiReceiving = true;
    //     SPDR = 0x0;
    //     while (!(SPSR & (1 << SPIF)))
    //         ;
    //     SPDR = 0xff;
    //     while (!(SPSR & (1 << SPIF)))
    //         ;
    //     SPDR = 0xff;
    //     while (!(SPSR & (1 << SPIF)))
    //         ;
    //     _sys.isSpiReceiving = false;
    //     timeout = 0;
    // }
    // else
    // {
    //     timeout++;
    //     delayMicroseconds(1);
    //     if (timeout > 10000)
    //     {
    //         _sys.isSpiProcessing = true;
    //         Serial.println("timeout!");
    //         timeout = 0;
    //         SPDR = 0x0;
    //         while (!(SPSR & (1 << SPIF)))
    //             ;
    //         SPDR = 0xff;
    //         while (!(SPSR & (1 << SPIF)))
    //             ;
    //     }
    // }
#endif
}

void parseCommand()
{
    _sys.buffer.toLowerCase();
    Serial.println(_sys.buffer);
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
                double x = hasCommand('x') ? fmax(-260, fmin(260, -getCommand('x'))) * PULSES_PER_MM : (_sys.isRelative ? 0 : _sys.targetPosition[0]);
                double y = hasCommand('y') ? fmax(-260, fmin(260, getCommand('y'))) * PULSES_PER_MM : (_sys.isRelative ? 0 : _sys.targetPosition[1]);
                double z = hasCommand('z') ? fmax(-260, fmin(260, -getCommand('z'))) * PULSES_PER_MM : (_sys.isRelative ? 0 : _sys.targetPosition[2]);
                if (_sys.isRelative)
                {
                    x += _sys.targetPosition[0];
                    y += _sys.targetPosition[1];
                    z += _sys.targetPosition[2];
                }
                if (cmdId == 0)
                {
                    moveTo(x, y, z);
                }
                else
                {
                    lineTo(x, y, z);
                }
            }
            break;
            case 4:
            {
                if (hasCommand('p'))
                {
                    delayTo(getCommand('p'));
                }
            }
            break;
            case 28:
            {
                waitingForFinish();
                _sys.plannedIndex = 0;
                _sys.plannedLength = 0;
                _sys.currentPosition[0] = 0;
                _sys.currentPosition[1] = 0;
                _sys.currentPosition[2] = 0;
                _sys.targetPosition[0] = 0;
                _sys.targetPosition[1] = 0;
                _sys.targetPosition[2] = 0;
                _prevPoint.targetPosition[0] = 0;
                _prevPoint.targetPosition[1] = 0;
                _prevPoint.targetPosition[2] = 0;
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
                // waitingForFinish();
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
                // waitingForFinish();
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
            case 5:
            {
                if (hasCommand('x') && hasCommand('y'))
                {
                    long x = getCommand('x');
                    long y = getCommand('y');
                    // steppers[0].setBack(x);
                    // steppers[1].setBack(y);
                }
                break;
            }
            case 6:
            {
                // if (hasCommand('x') && hasCommand('y'))
                // {
                //     long x = getCommand('x') / 2;
                //     long y = getCommand('y') / 2;
                //     for (int i = 0; i < 5; i++)
                //     {
                //         moveTo(-x + _sys.targetPosition[0], -y + _sys.targetPosition[1]);
                //         moveTo(x + _sys.targetPosition[0], y + _sys.targetPosition[1]);
                //     }
                // }
                break;
            }
            }
        }
        finishCommand();
    }
}
void addMotion(long x, long y, long z, uint8_t power, uint16_t speed, uint16_t acceleration)
{
    _sys.targetPosition[0] = x;
    _sys.targetPosition[1] = y;
    _sys.targetPosition[2] = z;
    _points[_sys.plannedIndex].targetPosition[0] = x;
    _points[_sys.plannedIndex].targetPosition[1] = y;
    _points[_sys.plannedIndex].targetPosition[2] = z;
    _points[_sys.plannedIndex].dir[0] = _prevPoint.targetPosition[0] < _points[_sys.plannedIndex].targetPosition[0];
    _points[_sys.plannedIndex].dir[1] = _prevPoint.targetPosition[1] < _points[_sys.plannedIndex].targetPosition[1];
    _points[_sys.plannedIndex].dir[2] = _prevPoint.targetPosition[2] < _points[_sys.plannedIndex].targetPosition[2];
    _points[_sys.plannedIndex].targetCount[0] = abs(_points[_sys.plannedIndex].targetPosition[0] - _prevPoint.targetPosition[0]);
    _points[_sys.plannedIndex].targetCount[1] = abs(_points[_sys.plannedIndex].targetPosition[1] - _prevPoint.targetPosition[1]);
    _points[_sys.plannedIndex].targetCount[2] = abs(_points[_sys.plannedIndex].targetPosition[2] - _prevPoint.targetPosition[2]);
    _points[_sys.plannedIndex].delta[0] = (_points[_sys.plannedIndex].targetCount[0]);
    _points[_sys.plannedIndex].delta[1] = (_points[_sys.plannedIndex].targetCount[1]);
    _points[_sys.plannedIndex].delta[2] = (_points[_sys.plannedIndex].targetCount[2]);
    _points[_sys.plannedIndex].restCount[0] = (_points[_sys.plannedIndex].targetCount[0]);
    _points[_sys.plannedIndex].restCount[1] = (_points[_sys.plannedIndex].targetCount[1]);
    _points[_sys.plannedIndex].restCount[2] = (_points[_sys.plannedIndex].targetCount[2]);
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
        _sys.prevDir[0] = _points[0].dir[0];
        _sys.prevDir[1] = _points[0].dir[1];
        _sys.prevDir[2] = _points[0].dir[2];
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
        while (stepZ(_points[0].dir[2]))
        {
            delay(160);
        };
        shiftPlanned();
        if (_sys.plannedLength > 0)
        {
            // if (_sys.prevDir[0] != _points[0].dir[0])
            // {
            //     _sys.currentPosition[0] -= steppers[0].getBack() * (_points[0].dir[0] ? 1 : -1);
            // }
            // if (_sys.prevDir[1] != _points[0].dir[1])
            // {
            //     _sys.currentPosition[1] -= steppers[1].getBack() * (_points[0].dir[1] ? 1 : -1);
            // }
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
int tt = 0;
ISR(TIMER1_COMPA_vect)
{
    // #ifdef MAKEBLOCK
    //     if (_sys.isSpiProcessing)
    //     {
    //         motion();
    //     }
    // #else
    if (!_sys.isReceiving)
    {
        run();
    }
    // #endif
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
    firePower(_points[0].power);
    if (_points[0].restCount[0] <= 0 && _points[0].restCount[1] <= 0)
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
    _points[0].restCount[0]--;
    _points[0].totalCount--;
    steppers[0].step(dir);
}
void stepY(bool dir)
{
    _sys.currentPosition[1] += dir ? 1 : -1;
    _points[0].restCount[1]--;
    _points[0].totalCount--;
    steppers[1].step(dir);
}
bool stepZ(bool dir)
{
    if (_points[0].restCount[2] > 0)
    {
        _sys.currentPosition[2] += dir ? 1 : -1;
        _points[0].restCount[2]--;
        steppers[2].step(dir);
    }
    return _points[0].restCount[2] > 0;
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
#ifdef MAKEBLOCK
    _sys.currentSpeed = _sys.currentSpeed > 20000 ? 20000 : (_sys.currentSpeed < 5 ? 5 : _sys.currentSpeed);
    long during = min(62500, max(100, ((2000000 / _sys.currentSpeed)))) - 1;
#else
    _sys.currentSpeed = (_sys.currentSpeed > 5000 ? 5000 : (_sys.currentSpeed < 5 ? 5 : _sys.currentSpeed));
    long during = min(62500, max(3000, ((625000 / _sys.currentSpeed)))) - 1;
#endif
    OCR1A = during;
}
/**
 * Motion Parse
 * */

void moveTo(long x, long y, long z)
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
    _sys.startPosition[2] = z;
    addMotion(x, y, z, 0, _sys.speed, _sys.acceleration);
}
void lineTo(long x, long y, long z)
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
    addMotion(x, y, z, _sys.power, _sys.speed, _sys.acceleration);
}
void delayTo(int ms)
{
    waitingForFinish();
    delay(ms);
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

double tttt = 0;
long kkk = 0;
///*test speed
void testSpeed(char c)
{
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
    }
}
//*/