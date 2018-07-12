/**********************ϵͳ����**********************************/
void Init ( Environment *env );//��ʼ��
void See ( Environment *env ); //Ԥ����
void End ( Environment *env ); //���ڴ���
void Action ( Environment *env );

/************************���ݴ���*********************************/
void RegulateAngle(double &angle);                                //�淶angle�Ĵ�С�ڣ�-180��+180��֮�� 
double Atan(double y,double x);                                   //������ƽ����ˮƽ��ļн�
double Atan(Vector3D begin,Vector3D end);                         //��������֮��ļн�(-180~180)
double Distance (Vector3D pos1,Vector3D pos2);                    //������֮��ľ���

/************************�˶�����*********************************/
void Angle( Environment *env, int robot,double angle);//ת��angle
void Angle( Environment *env, int robot,Vector3D pos);//��ת��Ŀ���
void PAngle( Environment *env, int robot,double angle,double speed=0);//��robot��angle�ķ����ܣ�����speed���������������
void PositionAndStop(Environment *env,int  robot,Vector3D pos ,double bestangle=90,double limit=1);//(һ���Ա)��robot �ܵ�pos������ͣ������ bestangle ��ͣ����֮��ĳ���limit	����ͣ��pos�����ľ���
void GoaliePosition(Environment *env,int  robot,Vector3D pos ,double bestangle=90,double limit=1.5);//������Ա��ͬ��
void PositionAndStopX(Environment *env,int  robot,Vector3D pos ,double Xangle=90,double limit=2); //��robot �ܵ�pos������ͣ����ԭ����ת��Xangle	��ת�Ľ��ٶȣ�limit	����ͣ��pos�����ľ���
void PositionBallX(Environment *env,int  robot,Vector3D pos ,double Xangle=90,double limit=3.5);  //��robot �ܵ�pos������ͣ����ԭ����ת��Xangle	��ת�Ľ��ٶ�,limit	�������robot�ľ���,�����Ͷ�Ա�ľ������limit����ת
void PositionAndThrough(Environment *env,int robot,Vector3D pos ,double MAX=125);//��robot�����MAX ����pos���м�û�м��ٿ���

/************************������*********************************/
void Kick(Environment *env , int  robot , Vector3D ToPos );       //��robot�����ߵ�ToPos��λ��
void Kick(Environment *env , int  robot ,int robot1);             //��robot ����robot1�ĽǶ�������
void Kick(Environment *env,int robot,int steps,double limits);    //����steps�����ں����λ����
void shoot(Environment *env,int robot);                           //����

/************************��������*********************************/
double AngleOne(double omiga,double vl,double vr);                //�����ڵ�ǰ���ٶ�omiga�Ļ���������������vl,vr���ƣ���һ�����ڴﵽ�Ľ��ٶ�
double VelocityOne(double speed,double vl,double vr);             //�����ڵ�ǰ�ٶ�speed�Ļ���������������vl,vr���ƣ���һ�����ڴﵽ���ٶ� ������һ�����ڴﵽ���ٶ�
void Velocity(Environment *env, int robot,double vl,double vr);   //�޸Ļ�������������
int CheckBall(Environment *env);//���ط�����
void CheckBlockInfo(Environment *env);//��ȡ��д���״̬
void PredictBall(Environment *env,int steps=1);                    //Ԥ�⾭�� steps ������֮�����λ��
Vector3D Meetball_p( Environment *env, int robot);                 //���robot׷�����λ��
void Order(Environment *env);                                      //��ɫ����

/************************��ֹ����*********************************/
bool Within(Environment* env,int robot,double LENGTH);            //�ж�robot��Ա����ľ����Ƿ���LENGTH�涨�ķ�Χ�ڷ���true  or false
/**************************����*********************************/
void Keeper ( Environment *env, int robot );                       //����Ա����
/**************************����*********************************/
void NormalGame ( Environment *env );
void FreeBallGame(Environment *env);
void PlaceBallGame(Environment *env);
void PenaltyBallGame(Environment *env);
void FreeKickGame(Environment *env);
void GoalKickGame(Environment *env);

/****************************����**********************************/
void Sweep(Environment*env,int robot);
int  WhoseBall(Environment *env);
void GAMESTATE(Environment *env);