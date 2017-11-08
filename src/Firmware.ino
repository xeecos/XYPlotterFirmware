#include <Arduino.h>
#include <MeStepper.h>
#define SBI(n, b) (n |= _BV(b))
#define CBI(n, b) (n &= ~_BV(b))

#define POINTS_COUNT 6
#define PULSES_PER_MM 100
#define CURVE_SECTION 0.5
#define NUM_STEPS 20
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
    bool running = false;
    volatile long currentPosition[NUM_AXIS] = {0, 0};
    volatile long targetPosition[NUM_AXIS];
    volatile long startPosition[NUM_AXIS];
    volatile long lastPosition[NUM_AXIS] = {0, 0};
    long timerCount = 0;
    long defaultTime = 4250;
    long defaultAcceleration = 64;
    long defaultAccelCount = 200;
    long defaultDecelCount = 200;
    uint16_t defaultSpeed = 500;
    volatile long maxSpeed = 100;
    volatile long minSpeed = 100;
    volatile long currentSpeed = 1;
    volatile uint8_t power = 0;
    volatile uint16_t speed = 100;
    volatile uint16_t acceleration = 100;
    volatile uint8_t plannedLength = 0;
    volatile uint8_t plannedIndex = 0;
    volatile uint8_t isRunningState = MOTION_ACCELETING;
    uint8_t powerPin = 10;
    String buffer = "";
} SystemStatus;
static SystemStatus _sys;
static PlannerPoint _points[POINTS_COUNT];
static PlannerPoint _prevPoint;
volatile bool busy = false;
MeStepper steppers[NUM_AXIS] = {MeStepper(SLOT_1), MeStepper(SLOT_2)};

void setup()
{
    Serial.begin(115200);
    delay(1000);
    initTimer();
    for (int i = 0; i < NUM_AXIS; i++)
    {
        steppers[i].setMicroStep(16);
        steppers[i].disableOutputs();
    }
    Serial.println("opened");
    setPower(0);
    moveTo(0, 0);
    for (;;)
    {
        if (Serial.available())
        {
            char c = Serial.read();
            parseBuffer(c);
        }
    }
}

void loop()
{
}
void parseBuffer(char c)
{
    if (c == '\n')
    {
        parseCommand(_sys.buffer);
        _sys.buffer = "";
    }
    else
    {
        _sys.buffer += c;
    }
}

static String vStr = "";
static double v[8];
void parseCommand(String cmd)
{
    cmd.toLowerCase();
    char code = cmd.charAt(0);
    long x1, y1, x2, y2, x3, y3, x4, y4;
    uint8_t vIndex = 0;
    uint8_t len = cmd.length();
    vStr = "";
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
        _sys.lastPosition[0] = x1;
        _sys.lastPosition[1] = y1;
        break;
    }
    case 'l':
    {
        x1 = v[0] * PULSES_PER_MM;
        y1 = v[1] * PULSES_PER_MM;
        lineTo(x1, y1);
        _sys.lastPosition[0] = x1;
        _sys.lastPosition[1] = y1;
        break;
    }
    case 'h':
    {
        x1 = v[0] * PULSES_PER_MM;
        lineTo(x1, _sys.lastPosition[1]);
        _sys.lastPosition[0] = x1;
        break;
    }
    case 'v':
    {
        y1 = v[0] * PULSES_PER_MM;
        lineTo(_sys.lastPosition[0], y1);
        _sys.lastPosition[1] = y1;
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
        curveTo(_sys.lastPosition[0], _sys.lastPosition[1], x1, y1, x2, y2, x3, y3);
        _sys.lastPosition[0] = x3;
        _sys.lastPosition[1] = y3;
        break;
    }
    case 'q':
    {
        x1 = v[0] * PULSES_PER_MM;
        y1 = v[1] * PULSES_PER_MM;
        x2 = v[2] * PULSES_PER_MM;
        y2 = v[3] * PULSES_PER_MM;
        quadTo(_sys.lastPosition[0], _sys.lastPosition[1], x1, y1, x2, y2);
        _sys.lastPosition[0] = x2;
        _sys.lastPosition[1] = y2;
        break;
    }
    case 'z':
    {
        lineTo(_sys.startPosition[0], _sys.startPosition[1]);
        _sys.lastPosition[0] = _sys.startPosition[0];
        _sys.lastPosition[1] = _sys.startPosition[1];
        break;
    }
    case 'p':
    {
        setPower(v[0]);
        setSpeed(v[1]);
        setAccel(v[2]);
        break;
    }
    case 'e':
    {
        firePower(v[0]);
        break;
    }
    case 'o':
    {
        if (v[0] > 0)
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
    }
    finishCommand();
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
    if (_sys.plannedIndex > 1)
    {
        _points[_sys.plannedIndex - 1].exitSpeed = _points[_sys.plannedIndex].speed;
    }
    _points[_sys.plannedIndex].acceleration = acceleration;
    long dist = sqrt(_points[_sys.plannedIndex].delta[0] * _points[_sys.plannedIndex].delta[0] + _points[_sys.plannedIndex].delta[1] * _points[_sys.plannedIndex].delta[1]) / 2;
    _points[_sys.plannedIndex].accelCount = _points[_sys.plannedIndex].totalCount - min(dist, _sys.defaultAccelCount);
    _points[_sys.plannedIndex].decelCount = min(dist, _sys.defaultDecelCount);
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
/**
 * motion control in Timer
 */

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
void lockTimer()
{
    cli();
    // noInterrupts();
}
void unlockTimer()
{
    sei();
    // interrupts();
}
void setRun()
{
    _sys.running = true;
}
void setStop()
{
    _sys.running = false;
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
            _sys.maxSpeed = _points[0].speed;
            _sys.minSpeed = fmax(10, _points[0].exitSpeed);
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
void initTimer()
{
    // lockTimer(); // disable all interrupts
    // TCCR1A = 0;
    // TCCR1B = 0;
    // TCNT1 = 0;
    cli();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 2000;           // compare match register 16MHz/256/1Hz
    TCCR1B |= (1 << WGM12); // CTC mode
    TCCR1B |= (1 << CS11);
    // TCCR1B |= (1 << CS10);   // 64 prescaler
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
    sei();
    // unlockTimer(); // enable all interrupts
}
int tt = 0;
ISR(TIMER1_COMPA_vect)
{
    if (busy)
    {
        return;
    }
    sei();
    busy = true;
    run();
    busy = false;
    cli();
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
    if (tt % 2 == 0)
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
            if (_sys.currentSpeed < _sys.maxSpeed)
            {
                _sys.currentSpeed += _points[0].acceleration;
                if (_sys.currentSpeed > _sys.maxSpeed)
                {
                    _sys.currentSpeed = _sys.maxSpeed;
                }
            }
            break;
        }
        case MOTION_DECELETING:
        {
            if (_sys.currentSpeed > _sys.minSpeed)
            {
                _sys.currentSpeed -= _points[0].acceleration;
                if (_sys.currentSpeed < _sys.minSpeed)
                {
                    _sys.currentSpeed = _sys.minSpeed;
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
        OCR1A = 1600000 / _sys.currentSpeed - 1;
    }
    tt++;
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
    _sys.currentSpeed = _sys.defaultSpeed;
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
void curveTo(long x0, long y0, long x1, long y1, long x2, long y2, long x3, long y3)
{
    double subdiv_step = 1.0 / (NUM_STEPS + 1);
    double subdiv_step2 = subdiv_step * subdiv_step;
    double subdiv_step3 = subdiv_step * subdiv_step * subdiv_step;
    double pre1 = 3.0 * subdiv_step;
    double pre2 = 3.0 * subdiv_step2;
    double pre4 = 6.0 * subdiv_step2;
    double pre5 = 6.0 * subdiv_step3;
    double tmp1x = x0 - x1 * 2.0 + x2;
    double tmp1y = y0 - y1 * 2.0 + y2;
    double tmp2x = (x1 - x2) * 3.0 - x0 + x3;
    double tmp2y = (y1 - y2) * 3.0 - y0 + y3;
    double fx = x0;
    double fy = y0;
    double dfx = (x1 - x0) * pre1 + tmp1x * pre2 + tmp2x * subdiv_step3;
    double dfy = (y1 - y0) * pre1 + tmp1y * pre2 + tmp2y * subdiv_step3;
    double ddfx = tmp1x * pre4 + tmp2x * pre5;
    double ddfy = tmp1y * pre4 + tmp2y * pre5;
    double dddfx = tmp2x * pre5;
    double dddfy = tmp2y * pre5;
    int step = NUM_STEPS;
#ifdef DEBUG
    String str = "curve:";
    str += x0;
    str += ",";
    str += y0;
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
    while (step--)
    {
        fx += dfx;
        fy += dfy;
        dfx += ddfx;
        dfy += ddfy;
        ddfx += dddfx;
        ddfy += dddfy;
        lineTo(floorf(fx * 100) / 100, floorf(fy * 100) / 100);
    }
    lineTo(floorf(x3 * 100) / 100, floorf(y3 * 100) / 100);
}
void quadTo(long x0, long y0, long x1, long y1, long x2, long y2)
{
#ifdef DEBUG
    String str = "quad:";
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
    int step = NUM_STEPS;
    double fx, fy;
    for (int i = 0; i < step; i++)
    {
        float t = (float)i / (float)step;
        float u = 1.0 - t;
        fx = x0 * u * u + x1 * 2 * u * t + x2 * t * t;
        fy = y0 * u * u + y1 * 2 * u * t + y2 * t * t;
        lineTo(floorf(fx * 100) / 100, floorf(fy * 100) / 100);
    }
    lineTo(floorf(x2 * 100) / 100, floorf(y2 * 100) / 100);
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