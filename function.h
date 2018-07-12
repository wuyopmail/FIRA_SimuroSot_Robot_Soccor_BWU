/**********************系统函数**********************************/
void Init ( Environment *env );//初始化
void See ( Environment *env ); //预处理
void End ( Environment *env ); //后期处理
void Action ( Environment *env );

/************************数据处理*********************************/
void RegulateAngle(double &angle);                                //规范angle的大小在（-180，+180）之间 
double Atan(double y,double x);                                   //在坐标平面与水平面的夹角
double Atan(Vector3D begin,Vector3D end);                         //求两个点之间的夹角(-180~180)
double Distance (Vector3D pos1,Vector3D pos2);                    //求两点之间的距离

/************************运动处理*********************************/
void Angle( Environment *env, int robot,double angle);//转向angle
void Angle( Environment *env, int robot,Vector3D pos);//车转向目标点
void PAngle( Environment *env, int robot,double angle,double speed=0);//让robot朝angle的方向跑，并且speed控制它的最大轮速
void PositionAndStop(Environment *env,int  robot,Vector3D pos ,double bestangle=90,double limit=1);//(一般队员)让robot 跑到pos，并且停下来， bestangle 是停下来之后的朝向，limit	控制停在pos附近的距离
void GoaliePosition(Environment *env,int  robot,Vector3D pos ,double bestangle=90,double limit=1.5);//（守门员）同上
void PositionAndStopX(Environment *env,int  robot,Vector3D pos ,double Xangle=90,double limit=2); //让robot 跑到pos，并且停下来原地旋转，Xangle	旋转的角速度，limit	控制停在pos附近的距离
void PositionBallX(Environment *env,int  robot,Vector3D pos ,double Xangle=90,double limit=3.5);  //让robot 跑到pos，并且停下来原地旋转，Xangle	旋转的角速度,limit	控制球和robot的距离,如果球和队员的距离大于limit则不旋转
void PositionAndThrough(Environment *env,int robot,Vector3D pos ,double MAX=125);//让robot以最快MAX 冲向pos，中间没有减速控制

/************************踢球动作*********************************/
void Kick(Environment *env , int  robot , Vector3D ToPos );       //让robot把球踢到ToPos的位置
void Kick(Environment *env , int  robot ,int robot1);             //让robot 以与robot1的角度跑向它
void Kick(Environment *env,int robot,int steps,double limits);    //向着steps个周期后球的位置踢
void shoot(Environment *env,int robot);                           //射门

/************************辅助函数*********************************/
double AngleOne(double omiga,double vl,double vr);                //计算在当前角速度omiga的基础上以左右轮速vl,vr控制，下一个周期达到的角速度
double VelocityOne(double speed,double vl,double vr);             //计算在当前速度speed的基础上以左右轮速vl,vr控制，下一个周期达到的速度 返回下一个周期达到的速度
void Velocity(Environment *env, int robot,double vl,double vr);   //修改机器人左右轮速
int CheckBall(Environment *env);//返回分区号
void CheckBlockInfo(Environment *env);//读取并写入块状态
void PredictBall(Environment *env,int steps=1);                    //预测经过 steps 个周期之后球的位置
Vector3D Meetball_p( Environment *env, int robot);                 //求出robot追到球的位置
void Order(Environment *env);                                      //角色分配

/************************防止犯规*********************************/
bool Within(Environment* env,int robot,double LENGTH);            //判断robot队员和球的距离是否再LENGTH规定的范围内返回true  or false
/**************************策略*********************************/
void Keeper ( Environment *env, int robot );                       //守门员守门
/**************************比赛*********************************/
void NormalGame ( Environment *env );
void FreeBallGame(Environment *env);
void PlaceBallGame(Environment *env);
void PenaltyBallGame(Environment *env);
void FreeKickGame(Environment *env);
void GoalKickGame(Environment *env);

/****************************补充**********************************/
void Sweep(Environment*env,int robot);
int  WhoseBall(Environment *env);
void GAMESTATE(Environment *env);