#include <Arduino.h>
#include <MeStepper.h>
#define STATES_COUNT 6
#define COMMANDS_COUNT 6
#define POINTS_COUNT 6
#define PULSES_PER_MM 100
#define CURVE_SECTION 0.5
#define NUM_STEPS 100
#define NUM_AXIS 2
// #define DEBUG true
enum MOTION_STATE
{
    MOTION_ACCELETING = 0,
    MOTION_NORMAL = 1,
    MOTION_DECELETING = 2
};
struct PlannerPoint
{
    long err;
    double delta[NUM_AXIS];
    bool dir[NUM_AXIS];
    long accelCount = 0;
    long decelCount = 0;
    long targetPosition[NUM_AXIS];
    long targetCount[NUM_AXIS];
    long totalCount = 0;
    uint8_t enterSpeed = 1;
    uint8_t speed = 1;
    uint8_t exitSpeed = 1;
    uint8_t acceleration = 1;
    uint8_t power = 0;
};
struct SystemStatus
{
    bool running = false;
    long currentPosition[NUM_AXIS] = {0, 0};
    long targetPosition[NUM_AXIS];
    long startPosition[NUM_AXIS];
    long lastPosition[NUM_AXIS] = {0, 0};
    long timerCount = 0;
    long time = 4250;
    long defaultTime = 4250;
    long defaultAcceleration = 64;
    long defaultAccelCount = 200;
    long defaultDecelCount = 200;
    long maxTime = 2250;
    long minTime = 25;
    PlannerPoint points[POINTS_COUNT];
    PlannerPoint prevPoint;
    uint8_t power = 0;
    uint8_t defaultSpeed = 1;
    uint8_t speed = 100;
    uint8_t acceleration = 1;
    uint8_t statesIndex = 0;
    uint8_t statesLength = 0;
    uint8_t plannedIndex = 0;
    uint8_t plannedLength = 0;
    uint8_t isRunningState = MOTION_ACCELETING;
    uint8_t powerPin = 10;
    String buffer = "";
};
SystemStatus _sys;
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
    moveTo(0, 0);
}

void loop()
{
    if (Serial.available())
    {
        char c = Serial.read();
        parseBuffer(c);
    }
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

void addMotion(long x, long y, uint8_t power, uint8_t speed, uint8_t acceleration)
{
    if (_sys.plannedLength >= POINTS_COUNT)
    {
        waitingPushPoint();
    }
    PlannerPoint prevPoint = _sys.prevPoint;
    PlannerPoint nextPoint;
    nextPoint.targetPosition[0] = x;
    nextPoint.targetPosition[1] = y;
    nextPoint.dir[0] = prevPoint.targetPosition[0] < nextPoint.targetPosition[0];
    nextPoint.dir[1] = prevPoint.targetPosition[1] < nextPoint.targetPosition[1];
    nextPoint.targetCount[0] = abs(nextPoint.targetPosition[0] - prevPoint.targetPosition[0]);
    nextPoint.targetCount[1] = abs(nextPoint.targetPosition[1] - prevPoint.targetPosition[1]);
    nextPoint.delta[0] = (nextPoint.targetCount[0]);
    nextPoint.delta[1] = (nextPoint.targetCount[1]);
    nextPoint.totalCount = nextPoint.delta[0] + nextPoint.delta[1];
    nextPoint.enterSpeed = _sys.plannedIndex > 0 ? prevPoint.exitSpeed : _sys.defaultSpeed;
    nextPoint.speed = fmax(1, speed);
    prevPoint.exitSpeed = fmax(1, sqrt(nextPoint.speed));
    nextPoint.acceleration = acceleration;
    long dist = sqrt(nextPoint.delta[0] * nextPoint.delta[0] + nextPoint.delta[1] * nextPoint.delta[1]) / 2;
    nextPoint.accelCount = nextPoint.totalCount - min(dist, _sys.defaultAccelCount);
    nextPoint.decelCount = min(dist, _sys.defaultDecelCount);
    nextPoint.power = power;
    nextPoint.err = (nextPoint.delta[0] > nextPoint.delta[1] ? nextPoint.delta[0] : -nextPoint.delta[1]) / 2;
    _sys.points[_sys.plannedIndex] = nextPoint;
    _sys.prevPoint = nextPoint;
    _sys.plannedIndex++;
    _sys.plannedLength++;
}
void finishCommand()
{
    if (commandsLength() < POINTS_COUNT)
    {
        Serial.print("ok:");
        Serial.println(commandsLength());
    }
}
int8_t commandsLength()
{
    return _sys.plannedLength;
}
void setPower(uint8_t power)
{
    _sys.power = fmaxf(0.0, fminf(100.0, power));
}
void setSpeed(uint8_t speed)
{
    _sys.speed = fmaxf(0.0, fminf(100.0, speed));
}
void setAccel(uint8_t accel)
{
    _sys.acceleration = fmaxf(0.0, fminf(100.0, accel));
}
void firePower(uint8_t power)
{
    pinMode(_sys.powerPin, OUTPUT);
    analogWrite(_sys.powerPin, fmaxf(0.0, fminf(255.0, power)));
}
void openPower()
{
    pinMode(_sys.powerPin, OUTPUT);
    analogWrite(_sys.powerPin, _sys.power * 2.55);
}
void closePower()
{
    analogWrite(_sys.powerPin, 0);
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
            _sys.points[i] = _sys.points[i + 1];
        }
        _sys.plannedIndex--;
        _sys.plannedLength--;
    }
    if (_sys.plannedLength <= 0)
    {
        finishCommand();
    }
}
void lockTimer()
{
    noInterrupts();
}
void unlockTimer()
{
    interrupts();
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
        lockTimer();
        shiftPlanned();
        unlockTimer();
        firePower(_sys.points[0].power);
        _sys.minTime = fmax(25, 100.0 / (_sys.points[0].speed / 100.0));
        _sys.maxTime = fmin(_sys.defaultTime, _sys.defaultTime / (_sys.points[0].exitSpeed / 50.0));
    }
}
void waitingPushPoint()
{
    while (true)
    {
        if (_sys.plannedLength < POINTS_COUNT)
        {
            break;
        }
        delay(200);
#ifdef DEBUG
        String str = "waiting:";
        str += _sys.plannedLength;
        Serial.println(str);
#endif
    }
}
void initTimer()
{
    lockTimer(); // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;

    OCR1A = _sys.time;      // compare match register 16MHz/256/1Hz
    TCCR1B |= (1 << WGM12); // CTC mode
    TCCR1B |= (1 << CS11);
    // TCCR1B |= (1 << CS10);   // 64 prescaler
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt

    unlockTimer(); // enable all interrupts
}
int tt = 0;
ISR(TIMER1_COMPA_vect)
{
    run();
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
    long distX = _sys.points[0].delta[0];
    long distY = _sys.points[0].delta[1];
    if (distX <= 0 && distY <= 0)
        return false;
    long e2 = _sys.points[0].err;
    if (e2 > -_sys.points[0].delta[0])
    {
        _sys.points[0].err -= _sys.points[0].delta[1];
        stepX(_sys.points[0].dir[0]);
    }
    if (e2 < _sys.points[0].delta[1])
    {
        _sys.points[0].err += _sys.points[0].delta[0];
        stepY(_sys.points[0].dir[1]);
    }
    nextWait();
    return true;
}
void stepX(bool dir)
{
    _sys.currentPosition[0] += dir ? 1 : -1;
    _sys.points[0].delta[0]--;
    _sys.points[0].totalCount--;
    steppers[0].step(dir);
}
void stepY(bool dir)
{
    _sys.currentPosition[1] += dir ? 1 : -1;
    _sys.points[0].delta[1]--;
    _sys.points[0].totalCount--;
    steppers[1].step(dir);
}
void nextWait()
{
    if (_sys.points[0].totalCount > _sys.points[0].accelCount)
    {
        _sys.isRunningState = MOTION_ACCELETING;
    }
    else if (_sys.points[0].totalCount > _sys.points[0].decelCount)
    {
        _sys.isRunningState = MOTION_NORMAL;
    }
    else
    {
        _sys.isRunningState = MOTION_DECELETING;
    }
    if (tt % 2 == 0)
    {
        switch (_sys.isRunningState)
        {
        case MOTION_ACCELETING:
        {
            if (_sys.time > _sys.minTime)
            {
                _sys.time -= _sys.defaultAcceleration;
                if (_sys.time < _sys.minTime)
                {
                    _sys.time = _sys.minTime;
                }
                else
                {
                }
            }
            break;
        }
        case MOTION_DECELETING:
        {
            if (_sys.time < _sys.maxTime)
            {
                _sys.time += _sys.defaultAcceleration;
            }
            else
            {
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
        OCR1A = _sys.time;
    }
    tt++;
}
/**
 * Motion Parse
 * */

void moveTo(long x, long y)
{
    waitingPushPoint();
#ifdef DEBUG
    String str = "move:";
    str += x;
    str += ",";
    str += y;
    Serial.println(str);
#endif
    _sys.startPosition[0] = x;
    _sys.startPosition[1] = y;
    _sys.time = _sys.defaultTime;
    addMotion(x, y, 0, _sys.speed, _sys.acceleration);
}
void lineTo(long x, long y)
{
    waitingPushPoint();
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