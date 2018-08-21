#define STRATEGY_API __declspec(dllexport)

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <windows.h>

///////////////////////////////////////////////////////////////////////
///////////////////////////////////常量////////////////////////////////
///////////////////////////////////////////////////////////////////////

/**********************************比赛数据***************************/

//游戏状态
const long FREE_BALL     = 1;                       // 争球
const long PLACE_KICK    = 2;                       // 开球
const long PENALTY_KICK  = 3;                       // 点球
const long FREE_KICK     = 4;                       // 任意球
const long GOAL_KICK     = 5;                       // 门球

//哪方发球
const long ANYONES_BALL  = 0;                       // 没有发球方
const long BLUE_BALL     = 1;                       // 蓝队发球
const long YELLOW_BALL   = 2;                       // 黄队发球


const long   CAR = 3;	                            //小车边长 2.95						
const double ROBOTWITH = 3.14 ;
const double BALLWITH = 1.5 ;

/*****************************场地数据***********************************/
//球场范围
const double PG_TOP       = 77.2392;                 // 场地上边界
const double PG_BOT       = 6.3730;                  // 场地下边界
const double PG_LEFT      = 6.8118;                   // 场地左边界
const double PG_RIGHT     = 93.4259;                  // 场地右边界

//球门区
const double BD_TOP       = 49.6801;                 // 球门上边界
const double BD_BOT       = 33.9320;                 // 球门下边界
/*左半场*/const double BD_LEFT       = 2.8748;       // 球门左边界//
/*右半场*/const double BD_RIGHT      = 97.3632;      // 球门右边界//

//小禁区
const double SRG_TOP = 51.6486;                      //小禁区上边界 
const double SRG_BOT = 31.9634;                      //小禁区下边界
/*左半场*/const double SRG_LEFT = 13.5555;           //小禁区左线
/*右半场*/const double SRG_RIGHT = 85.3632;          //小禁区右线

//不进入小禁区
const double GTOP = 50.0786;                         //小禁区上线 = - 车半径				
const double GBOT = 33.5334;                         //小禁区下线 = + 车半径	
const double GLEFT = 15.1255;                        //小禁区左线 = + 车半径
const double GRIGHT = 83.7932;                       //小禁区右线 = - 车半径

//大禁区
const double BRG_TOP = 57.5541;                      //大禁上线  
const double BRG_BOT = 26.0581;                      //大禁下线 
/*左半场*/const double BRG_LEFT = 21.2994;           //大禁左线 
/*右半场*/const double BRG_RIGHT = 79.3394 ;         //大禁右线



////能到达的边长范围
const double RTOP = 75.59 ;
const double RBOT = 8.04 ;
const double RLEFT = 8.44 ;
const double RRIGHT = 91.78;

//球网
const double GBLEFT = 2.8748;//能到达的 =  + 车半径
const double GBRIGHT = 97.3632;	//能到达的 =  - 车半径		

const double CORNER=5;

/*************************点的坐标******************************/

//争球
const double FBLEFT = 29.262907;			
const double FBRIGHT = 71.798508;
const double FREETOP = 64.428193;
const double FREEBOT = 18.184305;

//点球
const double PK_X = 22.216028;
const double PK_Y = 79.3394;

//门球
const double GK_X = 50.1189;
const double GK_Y = 41.8061;

/*************************修正数据***********************************/
//坐标转换时
const double CORRECTX = 0.8902;			
const double CORRECTY = 0.8957;

//速度
const double SPEED_ODD=0.662;	    //0.338;左右轮速为0时的减速参数
const double SPEED_ZERO = 0.1896;	// 0 减速度 和 125减速度的临界值

const double SPEED_TANGENT = 0.81;
const double SPEED_NORMAL = 0.27;

const double SPEED_A=0.060;
const double SPEED_B=0.015222305;

//角度
const double ANGLE_A=0.273575;
const double ANGLE_B=0.534262;
const double ANGLE_K=0.000294678;

/**********************************************************************/
const double PI = 3.1415926;

///////////////////////////////////////////////////////////////////////
/////////////////////////////////数据结构//////////////////////////////
///////////////////////////////////////////////////////////////////////




typedef struct
{
	double x, y, z;
} Vector3D;

typedef struct
{
	int x, y;
} Block;

typedef struct
{
	long left, right, top, bottom;
} Bounds;

typedef struct
{
	Vector3D pos;
	double rotation;
	double velocityLeft, velocityRight;
} Robot;

typedef struct
{
	Vector3D pos;
	double rotation;
} OpponentRobot;

typedef struct
{
	Vector3D pos;
} Ball;

typedef struct
{
	Robot home[5];
	OpponentRobot opponent[5];
	Ball currentBall, lastBall, predictedBall;
	Bounds fieldBounds, goalBounds;
	long gameState;	//0,1,2,3,4,5
	long whoseBall; //0,1,2
	void *userData;
} Environment;

typedef struct
{
	Vector3D my_old_pos[5];//我方队员上次的坐标
	Vector3D my_speed[5];//我方队员的速度
	Vector3D my_old_velocity[5];//我方队员上次的轮速

	Vector3D op_old_pos[5];//对方队员上次的坐标
	Vector3D op_speed[5];//对方队员的速度

	Robot robot[5];//我方球员
	OpponentRobot opp[5];//对方球员

	Vector3D ball_old;//球上次的坐标
	Vector3D ball_cur;//球现在的坐标
	Vector3D ball_pre;//球预测的坐标
	Vector3D ball_speed;//球速度的坐标


    long time[2];//time[1]取样周期//time[0]取样次数
	bool mygrand;//是 = 黄队//否 = 兰队
	bool locked;//是否已经判断场地	
	int WIB;//分区

	Vector3D block[100][100];//当前块的中心坐标
	int block_my[100][100];//我方队员分区
	int block_op[100][100];//对方队员分区
	int block_ball[100][100];//球分区
	Block block_min;//最小块x值
	Block block_max;//最大块x值
	Block my_block_pos[5];//我方队员的块坐标 //-1 = 不在栅格区域内 //值 >= 0 = 存在
	Block op_block_pos[5];//对方队员的块坐标 //-1 = 不在栅格区域内 //值 >= 0 = 存在
	Block ball_block_pos;//球的块坐标

	int bgoalball;//球门球
	int nfreeball;//自由球
	int nplaceball;
	int npenaltyball;//点对方球
	int chooserobot;//选择点球队员
		
	int ActiveAttacker;//队员
	int NegativeAttacker;
	int Attacker;
	int Defender;
	int Keeper;

	long gameState;	//0,1,2,3,4,5
	long whoseBall; //0,1,2

	long n;
	bool B_N;				//these two veriaty is for the test funtion!
	//	Bounds field, goal;

	bool debug;//是否是调试
	FILE * debugfile; 

}Mydata;


extern "C" STRATEGY_API void Create ( Environment *env ); 
extern "C" STRATEGY_API void Strategy ( Environment *env );
extern "C" STRATEGY_API void Destroy ( Environment *env ); 

///////////////////////////////////////////////////////////////////////
/////////////////////////////////初始化值//////////////////////////////
///////////////////////////////////////////////////////////////////////

const double InlitializeMyPosition_X[5]={10.6663,19.9199,19.7433,39.4618,39.8876};
const double InlitializeMyPosition_Y[5]={42.3077,60.3647,22.9943,60.3031,23.1065};
const double InlitializeMyPosition_Z[5]={90,0,0,0,0};
const double InlitializeOppPosition_X[5]={90.4616,82.0921,81.2890,61.8525,61.4469};
const double InlitializeOppPosition_Y[5]={42.2002,22.9046,60.4876,23.1527,60.3599};
const double InlitializeOppPosition_Z[5]={-90,180,180,180,180};


/*****************************几个特殊点*******************************/
const Vector3D CONSTGATE={PG_RIGHT,(PG_TOP+PG_BOT)/2,0};		
const Vector3D TOPGATE={93,72,0};
const Vector3D BOTGATE={93,9,0};
//const Vector3D TOPGATE={93,BD_TOP-1,0};
//const Vector3D BOTGATE={93,BD_BOT+1,0};