#include <DirectIO.h>
#include <TimerOne.h>
OutputPin dirPinX(5);
OutputPin stepPinX(4);
OutputPin dirPinY(3);
OutputPin stepPinY(2);


long xPosition = 0;
long yPosition = 0;

double pulsePerMM = 62.289;

float curve_section = 0.5;
bool isRelative = false;
String gcode = "";

float targetX = 0.0;
float targetY = 0.0;
long count = 0;
int motionDir = 1;
long targetDuringTime = 200;
long currentDuringTime = 2000;
int motionAcc = 1;
bool isMoving = false;


void updateMotion()
{
    if(isMoving){
        if(motionDir>0&&currentDuringTime>targetDuringTime){
            if(motionDir>150){
                motionAcc = -1;
                motionDir=150;
            }
            motionDir += motionAcc;
            if(motionDir<0){
                motionDir = 0;
            }
            currentDuringTime -= motionDir;
            if(currentDuringTime<targetDuringTime){
                currentDuringTime = targetDuringTime;
                motionDir = 0;
            }
        }else if(motionDir<0&&currentDuringTime<50000){
            
            if(motionDir<-150){
                motionAcc = -1;
                motionDir=-150;
            }
            motionDir -= motionAcc;
            if(motionDir>0){
                motionDir = 0;
            }
            currentDuringTime -= motionDir;
        }
    }
}


void setup(){
    Serial.begin(115200);
    Timer1.initialize(500);
    Timer1.attachInterrupt(updateMotion);
    pinMode(7,OUTPUT);
    analogWrite(7,0);
    Serial.print("start\n");
    
}

void loop(){
    if(Serial.available()){
        char c = Serial.read();
        if(c=='\n'){
            parseGCode();
            gcode = "";
        }else{
            gcode += c;
        }
    }
}
float getX(){
    return xPosition/pulsePerMM;
}
float getY(){
    return yPosition/pulsePerMM;
}
void parseGCode(){
    gcode.toLowerCase();
    char c;
    int cmd,index,startIndex,endIndex;
    float val;
    if(gcode.length()>2){
        if(hasCommand('g')){
            cmd = getCommand('g');
            switch(cmd){
                case 0:
                case 1:{
                    if(hasCommand('p')){
                        analogWrite(7,getCommand('p'));
                    }
                    if(hasCommand('f')){
                        targetDuringTime = (long)(50000/getCommand('f'));
                    }
                    double x = hasCommand('x')?getCommand('x'):0;
                    double y = hasCommand('y')?getCommand('y'):0;
                    if(isRelative){
                        move(x*pulsePerMM,y*pulsePerMM,cmd);
                    }else{
                        moveTo(hasCommand('x')?(x*pulsePerMM):xPosition,hasCommand('y')?(y*pulsePerMM):yPosition,cmd);
                        targetX = x;
                        targetY = y;
                    }
                }
                break;
                case 2:
                case 3:{
                    if(hasCommand('p')){
                        analogWrite(7,getCommand('p'));
                    }
                    arc(cmd==3);
                }
                break;
                case 28:{
                    xPosition = 0;
                    yPosition = 0;
                }
                break;
                case 90:{
                    isRelative = false;
                }
                break;
                case 91:{
                    isRelative = true;
                }
                break;
            }
        }else if(hasCommand('m')){
            cmd = getCommand('m');
            switch(cmd){
                case 4:
                    if(hasCommand('p')){
                        analogWrite(7,getCommand('p'));
                    }else{
                        analogWrite(7,0);
                    }
                break;
            }
        }
    }else{
    }
    commandEnd();
}
void commandEnd(){
    Serial.print("oMGok\n");
}
long toDistanceX(double x){
    return (x - xPosition);
}
long toDistanceY(double y){
    return (y - yPosition);
}

void stepX(bool dir){
    dirPinX = dir;
    stepPinX = LOW;
    delayMicroseconds(1);
    stepPinX = HIGH;
    xPosition+=dir?1:-1;
}
void stepY(bool dir){
    dirPinY = dir;
    stepPinY = LOW;
    delayMicroseconds(1);
    stepPinY = HIGH;
    yPosition+=dir?1:-1;
}

void move(long dx,long dy,bool precision){
    long tx = xPosition+dx;
    long ty = yPosition+dy;
    moveTo(tx,ty,precision);
}
int errX = 1;
int errY = 1;
void moveTo(long tx, long ty, bool precision) {  
    isMoving = true;
    long x0 = xPosition;
    long y0 = yPosition;
    long x1 = tx;
    long y1 = ty;
    currentDuringTime = max(targetDuringTime+500,1000);
    motionDir = 1;
    motionAcc = 1;
    long dx,dy,sx,sy;
    if(precision){
        dx = abs(x1-x0);
        sx = x0<x1;
        dy = abs(y1-y0);
        sy = y0<y1; 
        long err = (dx>dy ? dx : -dy)/2, e2;
        long ttt = millis();
        while(true){
            long distX = toDistanceX(x1);
            long distY = toDistanceY(y1);
            if(labs(distX)<200&&labs(distY)<200){
                motionDir = -1;
                motionAcc = 1;
            }
            if (distX==0 && distY==0) break;
            e2 = err;
            if (e2 >-dx) { 
                err -= dy;
                stepX(sx);   
            }
            if (e2 < dy) { 
                err += dx;
                stepY(sy);
            }
            if(currentDuringTime>1000){
                delay((int)(currentDuringTime/1000));
                delayMicroseconds(currentDuringTime%1000);
            }else{
                delayMicroseconds(currentDuringTime);
            }
            
            //     delayUs;delayUs;
        }
    }else{
        motionDir = 1;
        while(true){
            dx = toDistanceX(tx);
            dy = toDistanceY(ty);
            if(labs(dx)<200&&labs(dy)<200){
                motionDir = -1;
                motionAcc = 1;
            }
            if (dx==0 && dy==0) break;
            if (dx!=0) { 
                stepX(dx>0);
            }
            if (dy!=0) { 
                stepY(dy>0);
            }
            if(currentDuringTime>1000){
                delay((int)(currentDuringTime/1000));
                delayMicroseconds(currentDuringTime%1000);
            }else{
                delayMicroseconds(currentDuringTime);
            }
        }
    }
    isMoving = false;
}
void arc(bool ccw){
    float cx = isRelative?(getX()+getCommand('i')):getCommand('i');
    float cy = isRelative?(getY()+getCommand('j')):getCommand('j');
    float endX = isRelative?(getX()+getCommand('x')):getCommand('x');
    float endY = isRelative?(getY()+getCommand('y')):getCommand('y');
    float angleA, angleB, angle, radius, length, aX, aY, bX, bY;
    aX = (getX() - cx);
    aY = (getY() - cy);
    bX = (endX - cx);
    bY = (endY - cy);
    if(ccw){
        angleA = atan2(aY, aX);
        angleB = atan2(bY, bX);
    }else{
        angleA = atan2(bY, bX);
        angleB = atan2(aY, aX);
    }
    if (angleB <= angleA) {
        angleB += 2 * M_PI;
    }
    angle = angleB - angleA;
    radius = sqrt(aX * aX + aY * aY);
    length = radius * angle;
    int steps, s, step;
    steps = (int) ceil(length / curve_section);
    for (s = 1; s <= steps; s++) {
        step = (ccw) ? s : (steps - s); 
        moveTo((cx + radius * cos(angleA + angle * ((float) step / steps)))*pulsePerMM,(cy + radius * sin(angleA + angle * ((float) step / steps)))*pulsePerMM,1);
    }
}
bool hasCommand(char cmd){
    int index = 0;
    char c = 0;
    while(c!=cmd){
        if(index>=gcode.length()){
            return false;
        }else{
            c = gcode.charAt(index);
        }
        index++;
    }
    return true;
}
double getCommand(char cmd){
    int index = gcode.indexOf(cmd);
    int startIndex = gcode.indexOf(cmd)+1;
    char c = 0;
    while(c!=' '){
        index++;
        if(index>=gcode.length()){
            break;
        }else{
            c = gcode.charAt(index);
        }
    }
    return gcode.substring(startIndex,index).toDouble();
}