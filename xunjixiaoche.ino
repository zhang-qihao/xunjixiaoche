/**
*
 * @version 2021/10
 * @author zhang-qihao
*/
#include <stdlib.h>
#include <Servo.h>  // 系统自带的舵机库文件

#define ON  1       //使能LED
#define OFF 0       //禁止LED

#define ROW  5  // 行数
#define  COL  8 // 列数
#define  VEX  (ROW*COL) // 行 * 列
#define DEP  4  // 深度

/// 定义左右LED灯
unsigned char LED_LEFT = A2;    // 小车在一开始启动的灯
unsigned char LED_RIGHT = A3;   // 表示起点的灯，一旦灯亮了的话，表明小车已经到起点了
unsigned char LED_START = A3;   // 表示检测路口，如果前面的传感器到达一个路口小车，灯亮，下面的传感器也到下一个路口则灯灭，记录一个路口
unsigned char LED_NODE = 4;
unsigned char LED_TEST = 3;

/// 定义按键引脚为arduino的模拟口A0
const char KeyPos = A0;
/// 定义车身后的按键
unsigned char KEY_RETURN = A5;

///
unsigned char PIN_START = A1;

/// 耍小聪明：提前定义 x=4 y=* 上的点，用来提前设置障碍
unsigned char PIN_Barr40 = A15;
unsigned char PIN_Barr41 = A14;
unsigned char PIN_Barr42 = A13;
unsigned char PIN_Barr43 = A12;
unsigned char PIN_Barr44 = A11;
unsigned char PIN_Barr45 = A10;
unsigned char PIN_Barr46 = A9;
unsigned char PIN_Barr47 = A8;

/// 定义小车状态
int car_status1 = 1;

///
int test12 = 2;

/// 定义车前面一排的红外传感器的引脚
const int TrackPinLF1 = 25; // L：left左  R：right右 从中间往外编号1-3
const int TrackPinLF2 = 27;
const int TrackPinLF3 = 29;
const int TrackPinRI3 = 31;
const int TrackPinRI2 = 33;
const int TrackPinRI1 = 35;
/// 定义小车下方红外传感器针脚
const char TrackPinLM2 = 8;
const char TrackPinRM2 = 9;
/// 定义各个循迹红外引脚采集的数据的变量,分别接收对应上面8个传感器
unsigned char TrackValueLF1;
unsigned char TrackValueLF2;
unsigned char TrackValueLF3;
unsigned char TrackValueRI3;
unsigned char TrackValueRI2;
unsigned char TrackValueRI1;
unsigned char TrackValueLM2;    // M: middle中间
unsigned char TrackValueRM2;

///
unsigned int gnTmTuigan = 0;
unsigned int TM_TUIGAN = 4300;

/// 以下定义了电机控制管脚，左右各控制两个电机
// 左边两车轮
const unsigned char  WheelPinLFG = 47;
const unsigned char  WheelPinLFB = 49;
const unsigned char  WheelPinLFPWM = 11;    // 左车轮电机PWM调速
// 右边两车轮
const unsigned char  WheelPinRFG = 51;
const unsigned char  WheelPinRFB = 53;
const unsigned char  WheelPinRFPWM = 10;    // 右车轮电机PWM调速

/// 定义前超声传感器的发出与接收脚
unsigned char USEchoB = 46; // 信号输入
unsigned char USTrigB = 48; // 信号输出

/// 定义舵机引脚
int ServoPin = 3;
Servo s1;   // 定义舵机对象s1
int shen=5;     // 定义推杆升引脚
int jiang=4;    // 定义推杆降引脚

int init_start = 2;
int start_init_value = 0;

unsigned long gnTmStart = 0;    // 开始时间，单位毫秒
unsigned long gnTmCurr;         // 当前时间，单位毫秒
unsigned char gbDist = 0;       //
unsigned char gbInitPos = 1;    // 小车停放位置，0：左向右 1：右向左
unsigned char gbDireFirst = 1;  // 小车初始方向，0：左右  1：直行
unsigned char gbDireReturn = 1; // 0：左右 1：直行
int carStatus = 0; // -1：左转 1：右转 2：前行 -2：后退
// 起点(x0,y0),终点(x9,y9),当前点(x1,y1),下一目标点(x2,y2)
int x0 = 0, y0 = 0, x1 = 0, y1 = 0 , x2 = 0, y2 = 0, x9 = ROW - 1, y9 = COL - 1;
int nDire;
int nDirePre;   // 小车上一个方向，-1：左转 1：右转 2：前行 -2：后退
unsigned char gnAllWhite = 0;

unsigned char gnRunDir = 4; // 小车运行方向,0：停 1：东 2：南 3：西 4：北
unsigned char gbRun = 1;    // 小车运行状态，0：停 1：运行
unsigned int gnLoop = 0;    // 小车运行次数，0第一次
unsigned char gbBlock = 0;  // 0：前方无障碍 >0表示发现多次

/// 定义小车行驶速度
int gnbackSpeed = 255;          // 小车后退速度
int gnSpeed = 255 ;
const int SPEED_MAX = 255;      // 小车最大速度
const int SPEED_ROTARY = 255;
const int SPEED_LOW = 255;
const int SPEED_CHANGE = 255;

unsigned long gnwaitRunTime = 200;
unsigned long gnwaitNow = 0;
unsigned long gnwaitLast = 0;

unsigned char  ganDistUltra[3][8] = {0};    // 用于存放读取的超声距离值

const int maxnum = ROW * COL;
const int maxint = 255;
unsigned char ganVexTab[ROW][COL] = {0};            // 二维顶点表
unsigned char ganVexList[ROW * COL] = {0};          // 一维顶点表
unsigned char ganVexPath[ROW * COL + 1] = {0};      // 路径表, 第0个表示表中已有顶点集的个数
unsigned char ganVexLen[(maxnum + 1)*maxnum / 2];   // 记录图的两点间路径长度
unsigned char ganDist[maxnum] = {0};        // 表示当前点到源点的最短路径长度，各数组都从下标1开始
unsigned char  ganPrev[maxnum] = {255};     // 记录当前点的前一个结点，各数组都从下标1开始


/**
 *
 * @param v
 * @param u
 * @return
 */
int getVexSn( int v, int u)
{
    if ( v >= u )   return v * (v + 1) / 2 + u;
    else return u * (u + 1) / 2 + v;
}

/**
 * @brief 求最短路径
 * @param n
 * @param v
 * @param dist
 * @param prev
 * @param c
 */
void Dijkstra(int n, int v, unsigned char *dist, unsigned char *prev, unsigned char *c)
{
    int i, j;
    static bool s[maxnum] = {0};    // 判断是否已存入该点到S集合中

    if ( ( (gnLoop % 2) == 0 && ( gnRunDir == 4 || gnRunDir == 2 ) )
         || ( (gnLoop % 2) == 1 && gnRunDir == 1 ))
    {
        for ( i = n - 1; i >= 0; --i)
        {
            dist[i] = c[getVexSn(v, i)];
            s[i] = 0;     // 初始都未用过该点
            if (dist[i] == maxint)
                prev[i] = maxint;
            else
                prev[i] = v;
        }
        dist[v] = 0;
        s[v] = 1;
        for ( i = n - 1; i > 0; --i)
        {
            int tmp = maxint;
            int u = v;
            // 找出当前未使用的点j的dist[j]最小值
            for ( j = n - 1; j >= 0; --j)
                if ((!s[j]) && dist[j] < tmp)
                {
                    u = j;              // u保存当前邻接点中距离最小的点的号码
                    tmp = dist[j];
                }
            s[u] = 1;    // 表示u点已存入S集合中

            // 更新dist
            for ( j = n - 1; j >= 0; --j) {
                if ((!s[j]) && c[getVexSn(u, j)] < maxint)
                {
                    int newdist = dist[u] + c[getVexSn(u, j)];
                    if (newdist < dist[j])
                    {
                        dist[j] = newdist;
                        prev[j] = u;
                    }
                }
            }
        }
    }
    else
    {
        for ( i = 0; i < n; ++i)
        {
            dist[i] = c[getVexSn(v, i)];
            s[i] = 0;     // 初始都未用过该点
            if (dist[i] == maxint)
                prev[i] = maxint;
            else
                prev[i] = v;
        }
        dist[v] = 0;
        s[v] = 1;
        // 依次将未放入S集合的结点中，取dist[]最小值的结点，放入结合S中
        // 一旦S包含了所有V中顶点，dist就记录了从源点到所有其他顶点之间的最短路径长度
        // 注意是从第二个节点开始，第一个为源点
        for ( i = 1; i < n; ++i)
        {
            int tmp = maxint;
            int u = v;
            // 找出当前未使用的点j的dist[j]最小值
            for ( j = 0; j < n; ++j)
                if ((!s[j]) && dist[j] < tmp)
                {
                    u = j;              // u保存当前邻接点中距离最小的点的号码
                    tmp = dist[j];
                }
            s[u] = 1;    // 表示u点已存入S集合中

            // 更新dist
            for ( j = 0; j < n; ++j) {
                if ((!s[j]) && c[getVexSn(u, j)] < maxint)
                {
                    int newdist = dist[u] + c[getVexSn(u, j)];
                    if (newdist < dist[j])
                    {
                        dist[j] = newdist;
                        prev[j] = u;
                    }
                }
            }
        }
    }
}

/**
 * @brief 查找从源点v到终点u的路径，并输出
 * @param x1 小车当前x
 * @param y1 小车当前y
 * @param x2 小车需要前往的下一个点的x
 * @param y2 小车需要前往的下一个点的y
 * @return 0
 */
int searchPath(int x1, int y1, int x2, int y2)
{
    unsigned char que[maxnum];
    unsigned char tot = 0;
    unsigned char v = x1 * COL + y1;
    unsigned char u = x2 * COL + y2;
    int i;

    for ( i = 0; i < maxnum; ++i)
        ganDist[i] = maxint;
    Dijkstra( maxnum,  v, ganDist, ganPrev, ganVexLen );

    que[tot] = u;
    tot++;
    unsigned char tmp = ganPrev[u];
    while (tmp != v && tmp < maxint && tot < maxnum )
    {
        que[tot] = tmp;
        tot++;
        tmp = ganPrev[tmp];
    }
    if ( tmp == v ) {
        que[tot] = v;
        u = que[tot - 1];
    } else {
        u = v;
    }
    switch ( u - v) {
        case 1: return 2;
        case -1: return -2;
        case COL: return 1;
        case -COL: return -1;
        default:
            DijkstraInit();
            return 0;
    }
}

/**
 * @brief Dijkstra初始化
 */
void  DijkstraInit()
{
    int i, j, n;
    // 初始化ganVexLen[][]为maxint
    for ( i = 0; i < (maxnum + 1)*maxnum / 2; ++i)
        ganVexLen[i] = maxint;
    for ( n = 0; n < maxnum; n++) {
        i = n / COL;
        j = n % COL;
        if ( j - 1 >= 0 ) {
            ganVexLen[getVexSn(n, n - 1)] =  1;
        }
        if ( j + 1 < COL ) {
            ganVexLen[getVexSn(n, n + 1)] =  1;
        }
        if ( i + 1 < ROW ) {
            ganVexLen[getVexSn(n, n + COL)] =  1;
        }
        if ( i - 1 >= 0 ) {
            ganVexLen[getVexSn(n, n - COL)] =  1;
        }
    }
    if (gbInitPos ) { //停放位置，0左向右1右向左 )
        //  setBarrier( 0, 0 ); //设置对方配送站为障碍
    } else {
        //  setBarrier( 0, COL - 1 ); //设置对方配送站为障碍
    }
    n = ROW * COL;
    for ( i = 0; i < maxnum; ++i)
        ganDist[i] = maxint;
}

/**
 * @brief 在指定点设置障碍物，1：有障碍，0：无
 * @param x 障碍x坐标
 * @param y 障碍y坐标
 */
void setBarrier(int x, int y) {
    int n;
    if ( x >= 0 && x < ROW && y >= 0 && y < COL ) {
        ganVexTab[x][y] = 1;
        n = x * COL + y;
        if ( y - 1 >= 0 ) {
            ganVexLen[getVexSn(n, n - 1)] =  maxint;
        }
        if ( y + 1 < COL ) {
            ganVexLen[getVexSn(n, n + 1)] =  maxint;
        }
        if ( x + 1 < ROW ) {
            ganVexLen[getVexSn(n, n + COL)] =  maxint;
        }
        if ( x - 1 >= 0 ) {
            ganVexLen[getVexSn(n, n - COL)] =  maxint;
        }
    }
    else if ( x < 0 && y < 0 ) {
        for ( x = 0; x < ROW ; x++)
            for (  y = 0 ; y < COL; y++ )ganVexTab[x][y] = 0; //
    }
    for ( x = 1; x < ROW; x++) {
        for ( y = 1; y < COL; y++) {
            if ( ganVexTab[x][y] != 0 && ganVexTab[x - 1][y - 1] != 0 ) { //(x,y)点为路障, 对角也为路障，则直角也为路障
                if ( getBarrier( x - 2, y + 1) == 0 ) ganVexTab[x - 1][y] = maxint;
            }
        }
        for ( y = 0; y < COL - 1; y++) {
            if ( ganVexTab[x][y] != 0 && ganVexTab[x - 1][y + 1] != 0 ) { //(x,y)点为路障, 对角也为路障，则直角也为路障
                if ( getBarrier( x - 2, y - 1) == 0 )ganVexTab[x - 1][y] = maxint;
            }
        }
    }
}
/**
 * @brief 得到指定点是否有障碍物
 * @param x 需要确定是否有障碍物的点的x
 * @param y 需要确定是否有障碍物的点的y
 * @return 返回是否有障碍物，1：有障碍，0：无
 */
int getBarrier( int x, int y ) {
    if ( x >= 0 && x < ROW && y >= 0 && y < COL )return ganVexTab[x][y] ;
    else return -1;
}
/**
 * @brief 设置路径
 * @param x 小车当前x坐标
 * @param y 小车当前y坐标
 * @return i>最大返回-1，否则返回0
 */
int setPath( int x, int y )
{
    int i = x * COL + y, j;
    if ( x < 0 && y < 0 ) { //
        for ( i = 0; i <= ROW * COL; i++)ganVexPath[i] = 0; //路径表
    } else if ( x >= 0 && x < ROW && y >= 0 && y < COL ) {
        for ( j = 1; j <= ganVexPath[0]; j++) {
            if ( ganVexPath[j] == i)break;         //说明之前经过该点
        }
        if ( j > ganVexPath[0] ) { //这是个新路径
            ganVexPath[0]++;
            ganVexPath[ganVexPath[0]] = i;
        } else {
            ganVexPath[0] = j;
        }
    } else return -1;
    return i;
}
/**
 * @brief 清楚上一条路径
 */
void clearLastPath(  )
{
    ganVexPath[0]--;
    if ( ganVexPath[0] < 0 )ganVexPath[0] = 0;
}
/**
 * @brief 清除路径中(x, y)之后的点
 * @param x 需要清除的点的x
 * @param y 需要清除的点的y
 * @return 返回清除之后路径中的点数
 */
int clearPath( int x, int y )
{
    int i, j, n = ganVexPath[0];
    j = x * COL + y;
    for ( i = 0; i < n ; i++ ) {
        if ( ganVexPath[i + 1] == j ) {
            ganVexPath[0] = i;
            break;
        }
    }
    return ganVexPath[0];
}
/**
 * @brief 判断(x,y)点是否已加入路径
 * @param x
 * @param y
 * @return 加入返回是第几个加入路径，否则返回0
 */
int getPath( int x, int y  )
{
    int i = x * COL + y;
    int j;
    if ( x >= 0 && y >= 0 && i < ROW * COL ) {
        for ( j = ganVexPath[0]; j > 0 ; j-- ) {
            if ( ganVexPath[j] == i )return j;
        }
    }
    return 0;
}
/**
 * @brief 得到路径中的最后一个点
 * @return 路径中的最后一个点
 */
int getPrePos( )
{
    return ganVexPath[ganVexPath[0]];
}
/**
 * @brief 得到最后第n个位置
 * @param n： 0：表示终点 1：最后第一个
 * @return 最后第n个位置
 */
int getLastPos( int n )
{
    return ganVexPath[ganVexPath[0] - n];
}
/**
 * @brief 得到历史第n个位置
 * @param n n=1第一个
 * @return 历史第n个位置
 */
int getHistPos( int n )
{
    return ganVexPath[n + 1];
}
/**
 * @brief 开灯
 * @param pin 需要使能的引脚
 */
void LedOn( int pin )
{
    digitalWrite(pin, HIGH);
}
/**
 * @brief 关灯
 * @param pin 需要接地的引脚
 */
void LedOff( int pin )
{
    digitalWrite(pin, LOW);
}
/**
 * @brief 灯闪烁
 * @param pin 需要使能的引脚
 * @param time 闪烁的时间
 */
void LedFlash( int pin, int time )
{
    digitalWrite(pin, LOW);
    delay(10);
    if ( time > 10)time = 10;
    while (1) {
        digitalWrite(pin, HIGH);
        delay(100);
        digitalWrite(pin, LOW);
        if ( time <= 1) {
            if ( time == 1 )delay(100);
            break;
        }
        time--;
        delay(100);
    }
}
/**
  Function       brake
  @author        Danny
  @date          2017.07.25
  @brief         小车刹车
  @param[in]     time:延时时间
  @param[out]    void
  @retval        void
  @par History   无
*/
void brake(int time)
{
    //左电机停止
    digitalWrite(WheelPinLFG, LOW);      //左电机前进使能
    digitalWrite(WheelPinLFB, LOW);       //左电机后退禁止

    //右电机停止
    digitalWrite(WheelPinRFG, LOW);      //右电机前进禁止
    digitalWrite(WheelPinRFB, LOW);       //右电机后退禁止
    if ( time > 0 )delay(time);
}
/**
 * @brief 小车前进的函数
 * @param nSpeedL 左轮速度
 * @param nSpeedR 右轮速度
 */
void MyRun( int nSpeedL, int nSpeedR)
{
//    Serial.print("运行速度 L:");
//    Serial.print(nSpeedL);
//    Serial.print(" R:");
//    Serial.println(nSpeedR);
    //左电机前进
    double dRate = 1;
    if ( nSpeedL > 0 ) {
        nSpeedL = nSpeedL * dRate;
        if ( nSpeedL > SPEED_MAX )nSpeedL = SPEED_MAX;
        digitalWrite(WheelPinLFG, HIGH);      //左电机前进使能
        digitalWrite(WheelPinLFB, LOW);       //左电机后退禁止
        analogWrite(WheelPinLFPWM, nSpeedL);
    } else if ( nSpeedL < 0 ) {
        nSpeedL = -nSpeedL;
        nSpeedL = nSpeedL * dRate;
        if ( nSpeedL > SPEED_MAX )nSpeedL = SPEED_MAX;
        digitalWrite(WheelPinLFG, LOW);      //左电机前进使能
        digitalWrite(WheelPinLFB, HIGH);       //左电机后退禁止
        analogWrite(WheelPinLFPWM, nSpeedL);
    } else {
        digitalWrite(WheelPinLFG, LOW);      //左电机前进使能
        digitalWrite(WheelPinLFB, LOW);       //左电机后退禁止
    }

    //右电机停止
    if ( nSpeedR > 0 ) {
        if ( nSpeedR > SPEED_MAX )nSpeedR = SPEED_MAX;
        digitalWrite(WheelPinRFG, HIGH);      //左电机前进使能
        digitalWrite(WheelPinRFB, LOW);       //左电机后退禁止
        analogWrite(WheelPinRFPWM, nSpeedR);
    } else if ( nSpeedR < 0 ) {
        nSpeedR = -nSpeedR;
        if ( nSpeedR > SPEED_MAX )nSpeedR = SPEED_MAX;
        digitalWrite(WheelPinRFG, LOW);      //左电机前进使能
        digitalWrite(WheelPinRFB, HIGH);       //左电机后退禁止
        analogWrite(WheelPinRFPWM, nSpeedR);
    } else {
        digitalWrite(WheelPinRFG, LOW);      //左电机前进使能
        digitalWrite(WheelPinRFB, LOW);       //左电机后退禁止
    }
}
/**
 * @brief 测试距离
 */
int NewDistanceTest() {
    int MyFdistance;
    digitalWrite(USTrigB, LOW);               //给触发脚低电平2μs
    delayMicroseconds(2);
    digitalWrite(USTrigB, HIGH);              //给触发脚高电平10μs，这里至少是10μs
    delayMicroseconds(10);
    digitalWrite(USTrigB, LOW);
    MyFdistance = pulseIn(USEchoB, HIGH); // 读取高电平时间(单位：微秒)
    ganDistUltra[0][5] = MyFdistance = MyFdistance / 58;
    return MyFdistance;
}
/**
 * @brief 获得超声传感器测出的距离
 * @param ul -1左超声，0中，1右
 * @return 超声的距离值
 */
int GetDist(int ul) {
    if ( ul == 0 )return ganDistUltra[0][5];
    if ( ul < 0 )return ganDistUltra[1][5];
    else return ganDistUltra[2][5];
}
/**
 * @brief 判断障碍是否在所需距离内
 * @param ul -1左超声，0中，1右
 * @param min 最近距离
 * @param max 最远距离
 * @return 超声测距在指定范围内返回1 否则返回0
 */
int IsRangeDist( int ul, int min, int max )
{
    int j;
    if ( ul == 0 )j = 0;
    else if ( ul < 0 )j = 1;
    else j = 2;
    //终点(gx9,gy9) 下一目标点(x2,y2)
    //if (x2 == x9 && y2 == y9 && (ganDistUltra[j][5] < 5 || ganDistUltra[j][5] > 10)) return 0;
    if ( ganDistUltra[j][5] < min || ganDistUltra[j][5] > max )return 0;
    else return 1;
}
/**
  Function       bubble
  @author        Danny
  @date          2017.07.26
  @brief         超声波测五次的数据进行冒泡排序
  @param[in1]    a:超声波数组首地址
  @param[in2]    n:超声波数组大小
  @param[out]    void
  @retval        void
  @par History   无
*/
void bubble(unsigned char *a, int n)
{
    int i, j, temp;
    for (i = 0; i < n - 1; i++)
    {
        for (j = i + 1; j < n; j++)
        {
            if (a[i] > a[j])
            {
                temp = a[i];
                a[i] = a[j];
                a[j] = temp;
            }
        }
    }
}
/**
 * @brief 得到所有的传感器的值
 */
void TrackTest() {

    TrackValueLF1 = digitalRead(TrackPinLF1);
    TrackValueLF2 = digitalRead(TrackPinLF2);
    TrackValueLF3 = digitalRead(TrackPinLF3);
    TrackValueRI3 = digitalRead(TrackPinRI3);
    TrackValueRI2 = digitalRead(TrackPinRI2);
    TrackValueRI1 = digitalRead(TrackPinRI1);
    TrackValueRM2 = digitalRead(TrackPinRM2);
    TrackValueLM2 = digitalRead(TrackPinLM2);
}
/**
 * @brief 判断按键是否触发
 * @param KeyPin 按键引脚
 * @return 1：有 0：无
 */
int KeyIn( int KeyPin )
{
    //int key = digitalRead(KeyPin), key1;
    int key1;
    //delay(10);
    key1 = digitalRead(KeyPin);
    return key1;
}
/**
 * @author        SW
 * @date          2017.10.15
 * @brief         取得4个寻线传感器的值
 * @param[in]     int nPos，负为左，正为右，越接近0越靠近中间，为5的时候是全部
 * @retval        int ,对应传感器的值，为0表示为黑线，输入为0时如果全黑返回1,否则返回0
 * @par History   无
*/
int TrackValue(int nPos )
{
    if ( nPos == -3 && TrackValueLF1 == LOW )return 1;
    else if ( nPos == -2 && TrackValueLF2 == LOW ) return 1;
    else if ( nPos == -1 && TrackValueLF3 == LOW )return 1;
    else if ( nPos == 1 && TrackValueRI3 == LOW )return 1;
    else if ( nPos == 2 && TrackValueRI2 == LOW )return 1;
    else if ( nPos == 3 && TrackValueRI1 == LOW )return 1;
    else if ( nPos == -4 && TrackValueLM2 == HIGH )return 1;//下面左边的部分
    else if ( nPos == 4  && TrackValueRM2 == HIGH )return 1;//下面右边部分
    else if ( nPos == 5 ) {
        if ( TrackValueLF1 == LOW && TrackValueLF2 == LOW && TrackValueLF3 == LOW && TrackValueRI3 == LOW && TrackValueRI2 == LOW && TrackValueRI1 == LOW )return 1;
    }
    return 0;
}
/**
 * @brief 是否节点,中间两黑加两边至少一黑，为路口节点
 * @return 为路口节点，返回1，否则返回0
 */
int IsNode()
{
    static int nTm = 0;
    static int bStatus = 0;
    int nNode = 0;
    gnTmCurr = millis() ;  //当前时间，单位毫秒
    if ( !TrackValue(-2)  && !TrackValue(2) )nNode = 2; //两侧都为黑
    else if ( !TrackValue(-2) && !TrackValue(-1) && !TrackValue(1))nNode = -1;          //最左侧为黑
    else if ( !TrackValue(2) && !TrackValue(-1) && !TrackValue(1))nNode = 1;     //最右侧为黑

    if (  nNode != 0 ) { //发现两侧出现黑，有可能是路口
        bStatus++;
        if ( bStatus == 1  ) {
            return nNode; //两侧至少有一个为黑
        } return 0;
        nTm++; //如果现在也是路口,则次数加1
    } else bStatus = 0;
    return 0;
}

/*前进
  speed -- 前进速度
  返回 -- 0:非路，1:路口
*/
int gnlastTime = millis();
int gnWaitTime = 0;
int gnNowTime;

/**
 * @brief 压线行驶
 * @param speed
 */
void RunMidGo(int speed )
{
    while ( TrackValue(-4) && TrackValue(4) ) {
        TrackTest();    //四路循迹传感器状态
        brake(0);
        MyRun( speed, speed);
    }
    brake(10);
    MyRun( -speed, -speed);
}
/**
 * @brief 判断前面是否有障碍
 * @param dis 距离
 * @return 1表示有障碍，0 表示无障碍
 */
int test_hinder(int dis) {
    if ( dis > 15 && dis <= 25) { //如果直行且这个距离内有障碍则设置为障碍点
        //判断小车前面障碍物的坐标
        if ( gnRunDir == 4 ) {
            setBarrier( x2 + 1, y2 ); //目标位置设为障碍
        } else if ( gnRunDir == 2 ) {
            setBarrier( x2 - 1, y2 ); //目标位置设为障碍
        } else if ( gnRunDir == 3 ) {
            setBarrier( x2, y2 - 1 ); //目标位置设为障碍
        } else if ( gnRunDir == 1 ) {
            setBarrier( x2, y2 + 1 ); //目标位置设为障碍
        }
        delay(100);
        return 1;  //表示前方有障碍
    }//判断到前面有障碍的，并且处理的if语句
    return 0;
}
/**
 * @brief 表示下一步要走的方向
 * @return -1：左转 1：右转 2：前行 -2：后退
 */
int next_direction() {
    //表示小车到达起点
    if ( x1 == x0 && y1 == y0 && (gnLoop & 1) == 0) { // 小车第一次启动
        //如果状态是直行状态，，并且前面还没有障碍
        if ( gbDireFirst && nDire == 0) {
            return 1;
        }
        else if (gbInitPos ) {  // 如果初始状态为右向左
            return -2;
        }
        else {
            return 2;
        }
    } else if ( x1 == x9 && y1 == y9 && (gnLoop & 1) == 1 ) { //在终点
        return -1;
    } else {
        if ( gnLoop % 2 == 0 ) return searchPath( x1, y1, x9, y9 );   //(x1,y1)当前点坐标，(x9,y9)最终目标点
        else if ( gnLoop % 2 == 1 ) return searchPath( x1, y1, x0, y0 );
    }
}
/**
 * @brief 根据下一步要走的方向，获得到小车下一步要走的点坐标 改变全局变量，x2 ,y2 的值
 * @param dir_par 下一步要走的方向
 */
void get_next_point(int dir_par) {
    switch ( dir_par ) {
        case 1:     //向4方向  原本的x坐标加一
        case -1:    //向2方向  原本的x坐标减一
            x2 = x1 + nDire;
            y2 = y1;
            break;
        case 2:     //向1方向  原本的y坐标加一
            x2 = x1;
            y2 = y1 + 1;
            break;
        case -2:    //向3方向，原本的y坐标减一
            x2 = x1;
            y2 = y1 - 1;
            break;
    }
}
/**
 * @brief 得到小车改变后的方向
 * @param init_direction 初始的方向
 * @return 返回一个改变后的方向
*/
int get_change_direction(int init_direction) {
    int result_direction;
    //顺时针，右转，方向+
    //逆时针，左转，方向-
    if ( init_direction == 4 ) {

        if ( y2 > y1 )result_direction = 5;
        else if ( y2 < y1 )result_direction = 3;
        else if ( x2 < x1 )result_direction = 2;
        else result_direction = 4;

    } else if ( init_direction == 2 ) {

        if ( y2 > y1 )result_direction = 1;
        else if ( y2 < y1 )result_direction = 3;
        else if ( x2 > x1 )result_direction = 4;
        else result_direction = 2;

    } else if ( init_direction == 3 ) {

        if ( x2 > x1 )result_direction = 4;
        else if ( x2 < x1 )result_direction = 2;
        else if ( y2 > y1 )result_direction = 1;
        else result_direction = 3;

    } else if ( init_direction == 1 ) {

        if ( x2 > x1 )result_direction = 0;
        else if ( x2 < x1 )result_direction = 2;
        else if ( y2 < y1 )result_direction = 3;
        else result_direction = 1;
    }
    return result_direction;
}
/**
 * @brief 得到小车转弯的状态
 * @param new_dir 新的方向
 * @param old_dir 老的方向
 * @return 小车转弯的状态 -1：左转 1：右转 2：前行 -2：后退
*/
int get_car_turn_status(int new_dir, int old_dir) {
    int result = 0;
    //因为顺时针小车方向是+，所以现在的方向比原来的方向的值要大，也就是gnRunDir > nDirePre ->右转 gnRunDir < nDirePre ->左转 否则 不转
    if (abs(new_dir < old_dir) == 2) { //说明小车需要后退
        //如果小车后退说明小车的方向不变，则说明原本的方向应该等于现在的方向
        gnRunDir = old_dir;
        result = -2;
    } else if ( new_dir < old_dir ) {  //现在的方向，比以前的方向要小，所以要左转
        result = -1;
    }
    else if ( new_dir > old_dir ) {
        result = 1;
        //end if ( gnRunDir > nDirePre )
    } else {
        result = 2;
    }
    if ( new_dir < 1 )gnRunDir = 4;
    else if ( new_dir > 4 )gnRunDir = 1;
    return result;
}
/**
 * @brief 控制小车转到线上
*/
void control_get_lost() {
    while (1) {
        //如果车底下，两个灯都在黑线上
        TrackTest();
        if (TrackValue(-1)  && TrackValue(1)) {
            while (1) {
                TrackTest();
                if (!TrackValue(-1) && !TrackValue(1)) break;
            }
            break;
        }
    }
    TrackTest();
}
/**
 * @brief 小车细微的调整方向
*/
void smallChangeDirection() {

    TrackTest();
    if ((TrackValue(-1) && !TrackValue(1)) || (!TrackValue(2) && TrackValue(3))) { // 如果-1碰到白色且1碰到黑色 或者 2碰到黑色且3碰到白色,则右转
        //则小车
        MyRun(0, 0);
        MyRun(SPEED_CHANGE, -SPEED_CHANGE);
        while (!(!TrackValue(-1) && !TrackValue(1) && TrackValue(2) && TrackValue(-2))) {
            TrackTest();
            //识别一下路口
            if (Identifying_junctions()) break;
        }
        MyRun(0, 0);
    } else if ((TrackValue(1) && !TrackValue(-1)) || (!TrackValue(-2) && TrackValue(-3))) { // 如果1碰到白色且-1碰到黑色 或者 -2碰到黑色且-3碰到白色, 则左转
        MyRun(0, 0);
        MyRun(-SPEED_CHANGE, SPEED_CHANGE);
        while (!(!TrackValue(-1) && !TrackValue(1) && TrackValue(2) && TrackValue(-2))) {
            TrackTest();
            //识别一下路口
            if (Identifying_junctions()) break;
        }
        MyRun(0, 0);
    }
}
/**
 * @brief 推杆
 * @param mode 模式 1：升 -1：降 default：降
 * @param time 时间
 */
void tuigan( int mode, int time )
{
    if( mode == 1){
        digitalWrite(shen, HIGH);      //推杆上
        digitalWrite(jiang, LOW);
    }
    else if( mode == -1 ){
        digitalWrite(shen, LOW);      //推杆下
        digitalWrite(jiang, HIGH);
    }else {
        digitalWrite(shen, LOW);      //推杆下
        digitalWrite(jiang, LOW);
    }
    delay(time);
}
/**
 * @brief 舵机
 * @param mode 模式 0：    1：
 */
void duoji( int mode )
{
    if( mode == 0 ){
        s1.write(180);
    }
    else if( mode == 1 )
    {
        s1.write(70);
    }
}
/**
 * @brief 舵机喂食函数
 */
void food()
{
    tuigan( 1, 1000 );      //推杆上升制定毫秒
    duoji(1);
    delay(2000);
    duoji(0);
    tuigan( -1, 1500 );      //推杆下降指定毫秒
    tuigan( 0, 0 );      //推杆下降指定毫秒
}
/**
 * @brief  识别路口的方法
   @return 1：表示已经记录一个路口，0：表示没有到路口
*/
int Identifying_junctions() {
    TrackTest();
    if (!TrackValue(1) && !TrackValue(-1) && ((!TrackValue(2) && !TrackValue(3)) || (!TrackValue(-2) && !TrackValue(-3))) && car_status1 == 0) { //表示小车前面的传感器已经到了下一个路口
        LedOn(LED_START);
        //如果前面的传感器已经触碰到下一个路口，则等下面的传感器触碰到黑线，则记到达了下一个路口
        //下面的灰度传感器的值如果为HIGH的话，则表明小车下面的传感器已经到黑线上了，并且小车的状态值为1的时候，才会去记住一个路口
        //期间会有小灯亮起
        TrackTest();
        while (TrackValue(-4)==0 && TrackValue(4)==0) {
            MyRun(SPEED_MAX - 50, SPEED_MAX - 50);
            TrackTest();
        }
        Serial.print("左 ");
        Serial.print(TrackValue(-4));
        Serial.print("右 ");
        Serial.println(TrackValue(4));
        Serial.print("***到达路口(");Serial.print(x2);Serial.print(",");Serial.print(y2);Serial.println(")***");
        x1 = x2;
        y1 = y2;
        if ( x1 == 0 && y1 == 7 || x1 == 0 && y1 == 0)  // 到达两起点
            delay(200);
        if ( x1 == 4 && y1 == 0 || x1 == 4 && y1 == 7)  // 到达两终点
            delay(200);
        car_status1 = 1;
        LedOff(LED_START);
        TrackTest();
        MyRun(0, 0);
        return 1;
    }
    return 0;
}
/**
 * @brief 使得需要直行的小车，但是方向不正确的小车调整方向
*/
void make_car_center_1() {

    TrackTest();
    if (TrackValue(-1) && TrackValue(1)  && (!TrackValue(2) || !TrackValue(3)) ) { //如果中间的两个在白色的上面，并且右边的在传感器在黑色的上面，则小车右转
        MyRun(0, 0);
        MyRun(SPEED_CHANGE, -SPEED_CHANGE);
        while (TrackValue(-1) && TrackValue(1)) {
            TrackTest();
            if (Identifying_junctions()) {
                break;
            }
        }
        MyRun(0, 0);
    } else if (TrackValue(-1) && TrackValue(1)  && (!TrackValue(-2) || !TrackValue(-3)) ) { //如果中间的两个在白色的上面，并且左边边的在传感器在黑色的上面，则小车左转
        MyRun(0, 0);
        MyRun(-SPEED_CHANGE, SPEED_CHANGE);
        while (TrackValue(-1) && TrackValue(1)) {
            TrackTest();
            if (Identifying_junctions()) {
                break;
            }
        }
        MyRun(0, 0);
    }
}
/**
 * @brief 判断非路口
 */
void is_no_node() {
    //如果转弯转错了，把中间的甩出去，调整方向
    //如果小车的中间不在黑线上，但是旁边的传感器在黑线上，
    //则调整方向使得小车居中
    //判断非路口
    //  y = 7 x = 0 y = 0 x = 4
    TrackTest();
    if (  TrackValue(-2) && TrackValue(-1) && TrackValue(1) && TrackValue(2)  ) //全白需要后退
    {
        if (gnRunDir == 1) {
            y1 = y2 = COL - 1;
            x1 = x2;
        } else if (gnRunDir == 2) {
            x1 = x2 =  0;
            y1 = y2;
        } else if (gnRunDir == 3) {
            y1 = y2 = 0;
            x1 = x2;
        } else if (gnRunDir == 4) {
            x1 = x2 = ROW - 1;
            y1 = y2;
        }
    }
}
/**
 * @brief 验证车是否真的到终点
*/
void is_car_end() {
    //验证小车是否到终点或者起点
    TrackTest();
    if (((x1 == x0 && y1 == y0) || (x1 == x9 && y1 == y9)) && !TrackValue(-1) && !TrackValue(1) ) {
        if (gnRunDir == 1) {
            y1 = y1 - 1;
            y2 = y1;
        } else if (gnRunDir == 2) {
            x1 = x1 + 1;
            x2 = x1;
        } else if (gnRunDir == 3) {
            y1 = y1 + 1;
            y2 = y1;
        } else if (gnRunDir == 4) {
            x1 = x1 - 1;
            x2 = x1;
        }
    }
}
/**
 * @brief 检查是否是边界
 */
void check_border() {
    if ( !TrackValue(-1) && !TrackValue(1)) {
        if (x1 == 0 && gnRunDir == 2 ) {
            //小车此时的坐标是 x值是边缘，并且方面朝南
            //但是小车此时前面的传感器不为全白
            //也就是前面中间两个传感器还在黑线上，则x坐标自增一
            x1 = x1 + 1;
            x2 = x1;
        } else if (x1 == 4 && gnRunDir == 4 ) {
            x1 = x1 - 1;
            x2 = x1;
        } else if (y1 == 0 && gnRunDir == 3) {
            y1 = y1 + 1;
            y2 = y1;
        } else if (y1 == 7 && gnRunDir == 1) {
            y1 = y1 - 1;
            y2 = y1;
        }
    }
}
/**
 * @brief 程序初始化函数
 */
void setup()
{
    Serial.println("Initializing...");
    int i, j;
    // 设置串口数据传输的波特率为9600
    Serial.begin(9600);

    ////////// 初始化引脚的输入输出模式
    // 车轮
    pinMode(WheelPinLFG, OUTPUT);
    pinMode(WheelPinLFB, OUTPUT);
    pinMode(WheelPinRFG, OUTPUT);
    pinMode(WheelPinRFB, OUTPUT);
    // LED灯
    pinMode(LED_LEFT, OUTPUT);
    pinMode(LED_RIGHT, OUTPUT);
    pinMode(LED_START, OUTPUT);
    pinMode(LED_NODE, OUTPUT);
    pinMode(LED_TEST, OUTPUT);
    // 前方红外传感器
    pinMode(TrackPinLF1, INPUT);
    pinMode(TrackPinLF2, INPUT);
    pinMode(TrackPinLF3, INPUT);
    pinMode(TrackPinRI3, INPUT);
    pinMode(TrackPinRI2, INPUT);
    pinMode(TrackPinRI1, INPUT);
    // 提前设置的路障
    pinMode(PIN_Barr40, INPUT);
    pinMode(PIN_Barr41, INPUT);
    pinMode(PIN_Barr42, INPUT);
    pinMode(PIN_Barr43, INPUT);
    pinMode(PIN_Barr44, INPUT);
    pinMode(PIN_Barr45, INPUT);
    pinMode(PIN_Barr46, INPUT);
    pinMode(PIN_Barr47, INPUT);
    // 车后方开关/按键
    pinMode(KeyPos, INPUT);
    pinMode(KEY_RETURN, INPUT);
    // 推杆升降
    pinMode(shen, OUTPUT);
    pinMode(jiang, OUTPUT);
    // 超声波传感器
    pinMode(USEchoB, INPUT);    //定义超声波输入脚
    pinMode(USTrigB, OUTPUT);   //定义超声波输出脚
    // 预设点初始化
    pinMode(init_start, INPUT);

    ////////// 引脚电压信号写入
    // LED
    digitalWrite(LED_LEFT, LOW);
    digitalWrite(LED_RIGHT, LOW);
    // 提前设置的路障
    digitalWrite(PIN_Barr40, HIGH);
    digitalWrite(PIN_Barr41, HIGH);
    digitalWrite(PIN_Barr42, HIGH);
    digitalWrite(PIN_Barr43, HIGH);
    digitalWrite(PIN_Barr44, HIGH);
    digitalWrite(PIN_Barr45, HIGH);
    digitalWrite(PIN_Barr46, HIGH);
    digitalWrite(PIN_Barr47, HIGH);
    // 车后方开关/按键
    digitalWrite(KeyPos, HIGH);
    digitalWrite(KEY_RETURN, HIGH);
    // 前方红外传感器
    digitalWrite(TrackPinLF1, HIGH);
    digitalWrite(TrackPinLF2, HIGH);
    digitalWrite(TrackPinLF3, HIGH);
    digitalWrite(TrackPinRI3, HIGH);
    digitalWrite(TrackPinRI2, HIGH);
    digitalWrite(TrackPinRI1, HIGH);

    //舵机引脚
    s1.attach(ServoPin);
//    for (int pos = 180; pos >=0 ; pos--)
//      s1.write(pos);
    s1.write(90);
    duoji(0);
    tuigan(-1, 2000 );    //如果推杆下降2s
    tuigan(0, 0 );

    //推杆复位
    tuigan(-1,0);
    //舵机初始角度
    duoji(0);

    // 测试舵机功能
//    while(1){
//        for (int i = 0; i <= 180; i++)
//            s1.write(i);
//        for (int i = 180; i >=0; i--)
//            s1.write(i);
//    }

    ////////// 车轮功能测试函数
//    Serial.println("小车车轮测试...");
//    MyRun(100, 100);
//    while(1){};


    //////// 车前传感器功能测试函数
//    Serial.println("小车前方传感器测试...");
//    while(1){
//        TrackTest();
//        delay(1000);
//        Serial.print("传感器值：");
//        Serial.print(TrackValue(-3));
//        Serial.print(" ");
//        Serial.print(TrackValue(-2));
//        Serial.print(" ");
//        Serial.print(TrackValue(-1));
//        Serial.print(" ");
//        Serial.print(TrackValue(0));
//        Serial.print(" ");
//        Serial.print(TrackValue(1));
//        Serial.print(" ");
//        Serial.print(TrackValue(2));
//        Serial.print(" ");
//        Serial.println(TrackValue(3));
//    }


    ////////// 下方传感器功能测试函数
//   Serial.println("小车下方传感器测试...");
//   while(1){
//     TrackTest();
//     delay(1000);
//     Serial.print("传感器测试值：左");
//     Serial.print(TrackValue(-4));
//     Serial.print("    右");
//     Serial.println(TrackValue(4));
//   }

    ////////// LED功能测试函数（闪烁）
//    Serial.println("LED测试...");
//    while(1){
//      LedOn( LED_LEFT );
//      LedOn( LED_RIGHT );
//      delay(500);
//      LedOff( LED_LEFT );
//      LedOff( LED_RIGHT );
//      delay(500);
//    }

    ////////// 超声传感器功能测试函数
//   Serial.println("超声传感器测试...");
//   while(1){
//     Serial.print("传感器测试值：");
//     delay(1000);
//     int dis = NewDistanceTest();
//     Serial.print(dis);
//     Serial.print("  是否有障碍物 ");
//     Serial.println(test_hinder(dis));
//   }

    ////////// 测试小车的开关有没有用
//    Serial.println("小车后方开关测试...");
//    while(1){
//        gbInitPos = KeyIn(KeyPos);
//        if ( gbInitPos )Serial.println("right -> left");
//        else Serial.println("left -> rigth");
//        delay(1000);
//    }


    ////////// cheating, not recommended
//  if( !digitalRead(PIN_Barr40) )setBarrier( 4, 0 );
//  if( !digitalRead(PIN_Barr41) )setBarrier( 4, 1 );
//  if( !digitalRead(PIN_Barr42) )setBarrier( 4, 2 );
//  if( !digitalRead(PIN_Barr43) )setBarrier( 4, 3 );
//  if( !digitalRead(PIN_Barr44) )setBarrier( 4, 4 );
//  if( !digitalRead(PIN_Barr45) )setBarrier( 4, 5 );
//  if( !digitalRead(PIN_Barr46) )setBarrier( 4, 6 );
//  if( !digitalRead(PIN_Barr47) )setBarrier( 4, 7 );

    LedOn(LED_LEFT);
    gnTmStart = 0;
    start_init_value = digitalRead(init_start);
    //初始化障碍表和路径表
    setBarrier(-1, -1);
    setPath( -1, -1 );
    DijkstraInit();

    // 提前设置路障
    Serial.println("提前设置的路障信息：");
    for( i = 0 ; i < 8; i++){
        Serial.print(4*10+i);
        Serial.print(":");
        Serial.print( getBarrier(4, i) );
        Serial.print(" ");
    }
    Serial.println();

    // 读取小车的状态
    gbInitPos = KeyIn(KeyPos);  //尾部开关,拔到左为0 右为1, 停放位置，0左向右1右向左
    gbDireReturn = KeyIn(KEY_RETURN); //return，0左向右1右向左
    Serial.print("小车停放状态（0左向右1右向左）gbInitPos=");
    Serial.println(gbInitPos);
    Serial.print("小车停放状态（0左向右1右向左）gbDireReturn=");
    Serial.println(gbDireReturn);
    // ?
//    if (gbInitPos)
//        setBarrier(0, 0);
//    else
//        setBarrier(0, 7);


    Serial.println("Waiting(等待小车停放就位)...");
    LedOn(LED_RIGHT);
    while ( 1 ) { //检测小车是否就位，中间两个传感器压线加上左或右额外两个传感器压线时出循环，否则等待
        TrackTest();
        if ( ( TrackValue(-3) && TrackValue(-2) && !TrackValue(-1) && !TrackValue(1) && !TrackValue(2) && !TrackValue(3) ) ||
             ( !TrackValue(-3) && !TrackValue(-2) && !TrackValue(-1) && !TrackValue(1) && TrackValue(2) && TrackValue(3) ) )break;
        if( digitalRead(PIN_START) )break;
    }

    Serial.println("Waiting(等待挥手启动程序)...");

    // 小车就位后，小车放置就位，检测前方是否有一定距离内有无物体，有就启动，否则等待
    while ( 1 ) {
        if( digitalRead(PIN_START) )break;
        int dis = NewDistanceTest();
        if (dis > 5 && dis < 10)    break;
    }
    delay(1000); // 延时0.5秒,保证挥手不影响小车启动后的测距

    // LED灭
    LedOff(LED_LEFT);
    LedOff(LED_RIGHT);

    // 小车设置的前进
    MyRun( gnSpeed, gnSpeed);

    // 设置小车一开始的默认状态，1是直行，0是左右
    gbDireFirst = 0;
    TrackTest();

    /**
     * @brief 根据小车尾部开关初始化小车行驶方向
     */
    if ( gbInitPos )    // gbInitPos：停放位置，0左向右1右向左
    { //1右向左
        Serial.println("小车右向左");
        x0 = 0; y0 = COL - 1; x9 = ROW - 1; y9 = 0;
        x1 = x2 = x0; y1 = y2 = y0;
        if ( !TrackValue(-3) && !TrackValue(-2) && !TrackValue(-1) && !TrackValue(1)) {
            gbDireFirst = 1;   //0左右1直行
            gnRunDir = 4;      //运行方向,0停1东2南3西4北
            Serial.println("小车优先直行，方向向北");
        } else {
            gbDireFirst = 0;   //0左右1直行
            gnRunDir = 3;
            Serial.println("小车优先转弯，方向向西");
        }
    }//if ( gbInitPos )
    else //左向右
    {
        Serial.println("小车左向右");
        x0 = 0; y0 = 0;  x9 = ROW - 1; y9 = COL - 1;
        x1 = x2 = x0; y1 = y2 = y0;
        if (!TrackValue(-1) && !TrackValue(2) && !TrackValue(1) && !TrackValue(3) ) {
            gbDireFirst = 1;   //0左右1直行
            gnRunDir = 4;
            Serial.println("小车优先直行，方向向北");
        } else {
            gbDireFirst = 0;   //0左右1直行
            gnRunDir = 1;
            Serial.println("小车优先转弯，方向向东");
        }
    }

    setPath( x1, y1 );
    //得到小车一开始运行的时间
    gnTmStart = millis();  //开始时间，单位毫秒
    //打印信号小车开始运行
    Serial.println("小车配置完成，开始运行loop");
}// setup()结尾

//  int x0 = 0, y0 = COL-1, x1 = 0, y1 = COL-1, x2 = 0, y2 = COL-1, x9 = ROW-1, y9 = 0;
/**
 * Function       loop
 * @author        Danny     updated by zhang-qihao
 * @date          2017.07.25
 * @brief         先调用setup初始化配置里面的按键扫描函数，循迹模式开启
 * @param[in]     void
 * @retval        void
 * @par History   无
*/
void loop()
{
    Serial.print("小车当前位置x1 = ");
    Serial.print(x1);
    Serial.print("  y1 = ");
    Serial.println(y1);
    // 得到小车当前的时间
    gnTmCurr = millis() ; //当前时间，单位毫秒
    // 得到前面四个传感器的状态
    TrackTest();
    // 小车是否停止
    if ( !gbRun )   return ;

    //判断小车是否走完一圈

    // 小车位于终点且跑完半程  ||  小车位于起点且跑完全程
    if ( (x1 == x9 && y1 == y9 && gnLoop % 2 == 0 ) || (x1 == x0 && y1 == y0 && gnLoop % 2 == 1) ) {
        if ( gnLoop == 0 )setPath( x1 , y1 );
        Serial.println("到达终点或起点");
        // 下面的if-else控制小车到达终点或起点时,小车改变方向,小车背对箱子(终点)或者小车朝向北方(起点)
        if (gbInitPos ) //停放位置，0左向右1右向左
        {
            Serial.println("开始左转");
            MyRun(0, 0);
            MyRun(-SPEED_MAX + 50, SPEED_MAX - 50);
            control_get_lost();
            MyRun(0, 0);
        }
        else
        {
            Serial.println("开始右转");
            MyRun(0, 0);
            MyRun(SPEED_MAX - 50, -SPEED_MAX + 50);
            control_get_lost();
            MyRun(0, 0);
        }
        x2 = x1;
        y2 = y1;

        brake(0);
        if ( gnLoop % 2 == 0  ) //终点
        {
            MyRun(0, 0);
            MyRun(-gnbackSpeed, -gnbackSpeed);
            Serial.println("后退投喂");
            delay(100); // ?
            MyRun(0, 0);
            gnlastTime = millis() ;
            // 投喂物品
            food();
        }
        else//起点
        {
            MyRun(0, 0);
            MyRun(-gnbackSpeed, -gnbackSpeed);
            brake(0);
            LedOn(LED_RIGHT);
            delay(1500);
            LedOff(LED_RIGHT);
            Serial.println("到达起点");
            //获取当前时间
            gnlastTime = millis() ;
        }
        // 如果跑完全程
        if ( gnLoop % 2 == 0 )  gnRunDir = 2;
        else gnRunDir = 4;
        // 小车完成一次半程行驶
        gnLoop++;
    }//if ( (x1 == x9 && y1 == y9 && gnLoop % 2 == 0 ) || x1 == x0 && y1 == y0 && gnLoop % 2 == 1 ) { //说明跑到终点位置
    //前面这个if语句说明小车已经到了终点或者起点并且怎么处理
    //这个表示小车的当前点已经到达了目标点，则需要去重新计算下一步的目标点
    //MyLoop

    if ( x1 == x2 && y1 == y2) {
        TrackTest();
        //检测5-25之间是否有障碍物，并且躲避掉
        if (gnLoop <= 1) {
            int num = 5;
            while (num--) {
                int dis = NewDistanceTest();
                nDire = test_hinder(dis);
                if (nDire)  break;
                delay(5);
            }
            nDire == 0?Serial.println("前方无障碍"):Serial.println("前方有障碍");
        }
        //判断小车下一步要行动的方向

        // 小车到达终点且跑完半程且小车初始左右方向行驶
        if ( x1 == x9 && y1 == y9 && gnLoop % 2 == 1 && gbDireReturn == 0 ) {
            // gbInitPos：停放位置，0：左向右  1：右向左
            if (gbInitPos ) {
                nDire = 2;
            }
            else {
                nDire = -2;
            }
        }
        else    nDire = next_direction();

        Serial.print("nDire: ");
        Serial.println(nDire);
        //获得到小车下一步要走的点坐标
        get_next_point(nDire);
        Serial.print("小车下一个行驶到的点next point: x = ");
        Serial.print(x2);
        Serial.print("  y = ");
        Serial.println(y2);

        //记录一下原来的方向
        nDirePre = gnRunDir;
        //顺时针，右转，方向+
        //逆时针，左转，方向-
        //获得到改变后的方向
        gnRunDir = get_change_direction(gnRunDir);
        //得到小车前进，后退，左转，右转的标志 左转-1 右转1 前进 2 后退-2
        carStatus = 0;
        //拿到小车下一步的转向
        carStatus = get_car_turn_status(gnRunDir, nDirePre);
        Serial.print("小车下一步的转向gnRunDir:");
        Serial.println(gnRunDir);
        Serial.print("小车状态carStatus:");
        Serial.println(carStatus);

        //小车调整一下转个向
        if ( carStatus == -1) {//小车左转
            Serial.println("小车左转");
            LedOn(LED_LEFT);
            MyRun(0, 0);
            MyRun(-SPEED_LOW, SPEED_LOW);
            //保证转弯能有90度
            control_get_lost();
            MyRun(0, 0);
            LedOff(LED_LEFT);
        } else if (carStatus == 1) { //小车右转
            Serial.println("小车右转");
            LedOn(LED_TEST);
            MyRun(0, 0);
            MyRun(SPEED_LOW, -SPEED_LOW);
            //保证转弯能有90度
            control_get_lost();
            MyRun(0, 0);
            LedOff(LED_TEST);
        }
    }//endif ( x1 == x2 && y1 == y2 )

    if (carStatus == -2) { //小车后退
        Serial.println("小车后退");
        MyRun(0, 0);
        MyRun(-gnbackSpeed, -gnbackSpeed);
        //如果小车后退到下一个一个节点，也就是底下的传感器对到下面的黑线
        while (1) {
            TrackTest();
            if (!TrackValue(-4) || !TrackValue(4)) {
                x2 = x1;
                y2 = y1;
                break;
            }
        }
        MyRun(0, 0);
    }

    // 如果小车当前点的坐标和下一个点的坐标不相等，则开始向下一个点移动
    // 如果小车的横纵坐标不相等，则一直在里面循环
    // 检测小车检测是否出错
    is_no_node();
    Serial.println("前往下一个点");
    while ( x2 != x1 || y2 != y1 ) //找到下一个点
    {
        make_car_center_1();
        MyRun(SPEED_ROTARY, SPEED_ROTARY);
        TrackTest();
        //细微的调整小车走直线
        smallChangeDirection();
        //设置状态
        TrackTest();
        if ( TrackValue(3) && TrackValue(-3) && TrackValue(2) && TrackValue(-2) && !TrackValue(1) && !TrackValue(-1) && car_status1 == 1) {
            car_status1 = 0;
        }
        //检测障碍物
        if (!((x2 == 4 && y2 == 7) || (x2 == 4 && y2 == 0))) {  // 小车没有位于终点
            Identifying_junctions();
            int dis1 = NewDistanceTest();
            // ? Identifying_junctions()
            if ( dis1 > 15 && dis1 <= 25 && Identifying_junctions()) //距离小于一定值,则需要后退
            {
                LedOn(LED_LEFT);
                LedOn(LED_RIGHT);
                car_status1 = 1;
                Serial.println("前方有路障，刹车");
                MyRun(0, 0);
                //说明不是个障碍物
                setBarrier( x2, y2 );
                //是个障碍物
                //先判断小车的下面是否在黑线上，如果在，则小车不后退，
                //继续判断方向
                //小车后退
                delay(100);
                MyRun( -gnbackSpeed, -gnbackSpeed);
                //目标位置设为障碍
                while (1) {
                    //如果底下两个传感器有在黑线上，则停止后退
                    TrackTest();
                    if (!TrackValue(-4) || !TrackValue(4)) break;
                }
                MyRun(0, 0);
                delay(100);
                LedOff(LED_LEFT);
                LedOff(LED_RIGHT);
                //并且还原点坐标
                x2 = x1;
                y2 = y1;
                //因为重新回到以前的点，所以需要重新计算下一个点,跳出此while循环
                Serial.println("小车重新回到以前的点");
                break;
            }//如果是非路
        }
        Identifying_junctions();
        //如果前面没有障碍物，小车则向前继续前行，直到，小车达到下一个位置为止,小车底部的传感器
        //达到黑线，则表示点坐标到了
    }

    //保证小车可以到终点或者起点
    is_car_end();
    //保证小车如果此时的点坐标是边缘，但是实际上并没有到边缘，该怎么办
    check_border();
    car_status1 = 1;
}
