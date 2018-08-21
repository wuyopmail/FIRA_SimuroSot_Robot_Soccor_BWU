#define STRATEGY_API __declspec(dllexport)

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <windows.h>

///////////////////////////////////////////////////////////////////////
///////////////////////////////////����////////////////////////////////
///////////////////////////////////////////////////////////////////////

/**********************************��������***************************/

//��Ϸ״̬
const long FREE_BALL     = 1;                       // ����
const long PLACE_KICK    = 2;                       // ����
const long PENALTY_KICK  = 3;                       // ����
const long FREE_KICK     = 4;                       // ������
const long GOAL_KICK     = 5;                       // ����

//�ķ�����
const long ANYONES_BALL  = 0;                       // û�з���
const long BLUE_BALL     = 1;                       // ���ӷ���
const long YELLOW_BALL   = 2;                       // �ƶӷ���


const long   CAR = 3;	                            //С���߳� 2.95						
const double ROBOTWITH = 3.14 ;
const double BALLWITH = 1.5 ;

/*****************************��������***********************************/
//�򳡷�Χ
const double PG_TOP       = 77.2392;                 // �����ϱ߽�
const double PG_BOT       = 6.3730;                  // �����±߽�
const double PG_LEFT      = 6.8118;                   // ������߽�
const double PG_RIGHT     = 93.4259;                  // �����ұ߽�

//������
const double BD_TOP       = 49.6801;                 // �����ϱ߽�
const double BD_BOT       = 33.9320;                 // �����±߽�
/*��볡*/const double BD_LEFT       = 2.8748;       // ������߽�//
/*�Ұ볡*/const double BD_RIGHT      = 97.3632;      // �����ұ߽�//

//С����
const double SRG_TOP = 51.6486;                      //С�����ϱ߽� 
const double SRG_BOT = 31.9634;                      //С�����±߽�
/*��볡*/const double SRG_LEFT = 13.5555;           //С��������
/*�Ұ볡*/const double SRG_RIGHT = 85.3632;          //С��������

//������С����
const double GTOP = 50.0786;                         //С�������� = - ���뾶				
const double GBOT = 33.5334;                         //С�������� = + ���뾶	
const double GLEFT = 15.1255;                        //С�������� = + ���뾶
const double GRIGHT = 83.7932;                       //С�������� = - ���뾶

//�����
const double BRG_TOP = 57.5541;                      //�������  
const double BRG_BOT = 26.0581;                      //������� 
/*��볡*/const double BRG_LEFT = 21.2994;           //������� 
/*�Ұ볡*/const double BRG_RIGHT = 79.3394 ;         //�������



////�ܵ���ı߳���Χ
const double RTOP = 75.59 ;
const double RBOT = 8.04 ;
const double RLEFT = 8.44 ;
const double RRIGHT = 91.78;

//����
const double GBLEFT = 2.8748;//�ܵ���� =  + ���뾶
const double GBRIGHT = 97.3632;	//�ܵ���� =  - ���뾶		

const double CORNER=5;

/*************************�������******************************/

//����
const double FBLEFT = 29.262907;			
const double FBRIGHT = 71.798508;
const double FREETOP = 64.428193;
const double FREEBOT = 18.184305;

//����
const double PK_X = 22.216028;
const double PK_Y = 79.3394;

//����
const double GK_X = 50.1189;
const double GK_Y = 41.8061;

/*************************��������***********************************/
//����ת��ʱ
const double CORRECTX = 0.8902;			
const double CORRECTY = 0.8957;

//�ٶ�
const double SPEED_ODD=0.662;	    //0.338;��������Ϊ0ʱ�ļ��ٲ���
const double SPEED_ZERO = 0.1896;	// 0 ���ٶ� �� 125���ٶȵ��ٽ�ֵ

const double SPEED_TANGENT = 0.81;
const double SPEED_NORMAL = 0.27;

const double SPEED_A=0.060;
const double SPEED_B=0.015222305;

//�Ƕ�
const double ANGLE_A=0.273575;
const double ANGLE_B=0.534262;
const double ANGLE_K=0.000294678;

/**********************************************************************/
const double PI = 3.1415926;

///////////////////////////////////////////////////////////////////////
/////////////////////////////////���ݽṹ//////////////////////////////
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
	Vector3D my_old_pos[5];//�ҷ���Ա�ϴε�����
	Vector3D my_speed[5];//�ҷ���Ա���ٶ�
	Vector3D my_old_velocity[5];//�ҷ���Ա�ϴε�����

	Vector3D op_old_pos[5];//�Է���Ա�ϴε�����
	Vector3D op_speed[5];//�Է���Ա���ٶ�

	Robot robot[5];//�ҷ���Ա
	OpponentRobot opp[5];//�Է���Ա

	Vector3D ball_old;//���ϴε�����
	Vector3D ball_cur;//�����ڵ�����
	Vector3D ball_pre;//��Ԥ�������
	Vector3D ball_speed;//���ٶȵ�����


    long time[2];//time[1]ȡ������//time[0]ȡ������
	bool mygrand;//�� = �ƶ�//�� = ����
	bool locked;//�Ƿ��Ѿ��жϳ���	
	int WIB;//����

	Vector3D block[100][100];//��ǰ�����������
	int block_my[100][100];//�ҷ���Ա����
	int block_op[100][100];//�Է���Ա����
	int block_ball[100][100];//�����
	Block block_min;//��С��xֵ
	Block block_max;//����xֵ
	Block my_block_pos[5];//�ҷ���Ա�Ŀ����� //-1 = ����դ�������� //ֵ >= 0 = ����
	Block op_block_pos[5];//�Է���Ա�Ŀ����� //-1 = ����դ�������� //ֵ >= 0 = ����
	Block ball_block_pos;//��Ŀ�����

	int bgoalball;//������
	int nfreeball;//������
	int nplaceball;
	int npenaltyball;//��Է���
	int chooserobot;//ѡ������Ա
		
	int ActiveAttacker;//��Ա
	int NegativeAttacker;
	int Attacker;
	int Defender;
	int Keeper;

	long gameState;	//0,1,2,3,4,5
	long whoseBall; //0,1,2

	long n;
	bool B_N;				//these two veriaty is for the test funtion!
	//	Bounds field, goal;

	bool debug;//�Ƿ��ǵ���
	FILE * debugfile; 

}Mydata;


extern "C" STRATEGY_API void Create ( Environment *env ); 
extern "C" STRATEGY_API void Strategy ( Environment *env );
extern "C" STRATEGY_API void Destroy ( Environment *env ); 

///////////////////////////////////////////////////////////////////////
/////////////////////////////////��ʼ��ֵ//////////////////////////////
///////////////////////////////////////////////////////////////////////

const double InlitializeMyPosition_X[5]={10.6663,19.9199,19.7433,39.4618,39.8876};
const double InlitializeMyPosition_Y[5]={42.3077,60.3647,22.9943,60.3031,23.1065};
const double InlitializeMyPosition_Z[5]={90,0,0,0,0};
const double InlitializeOppPosition_X[5]={90.4616,82.0921,81.2890,61.8525,61.4469};
const double InlitializeOppPosition_Y[5]={42.2002,22.9046,60.4876,23.1527,60.3599};
const double InlitializeOppPosition_Z[5]={-90,180,180,180,180};


/*****************************���������*******************************/
const Vector3D CONSTGATE={PG_RIGHT,(PG_TOP+PG_BOT)/2,0};		
const Vector3D TOPGATE={93,72,0};
const Vector3D BOTGATE={93,9,0};
//const Vector3D TOPGATE={93,BD_TOP-1,0};
//const Vector3D BOTGATE={93,BD_BOT+1,0};