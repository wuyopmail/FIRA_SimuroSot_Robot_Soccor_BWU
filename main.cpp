#include "constants.h"
#include "function.h"
/****************************************������***************************************/
/*DllMain,Create,Destroy���������������Ҳ�����ᣬֻ��������ڲ���֮�������������Ҫ����ֵ������Create �н���*/
//DllMain
//1.Create
//����Init  ��ʼ��
//2.Destroy
//3.Stategy ���Ե���ڣ��ú�������ʽ�����޸ģ���������,ͨ���޸� Environment.home.velocityLeft, Environment.home.velocityRight��ֵ�����ƻ����˵��˶�
//(1)See Ԥ����    ��������任�����һЩ�����о����õ���������ÿ����Ա�����ʺͽ��ٶȵ�
//(2)Action ����
//(3)End ���ڴ���  �ύ�Զ�Ա���ٵ��޸ģ����汾���ڵ�״̬���´μ���ʹ��
/**************************************************************************************/

BOOL APIENTRY DllMain( HANDLE hModule, 
                       DWORD  ul_reason_for_call, 
                       LPVOID lpReserved
					 )
{
    switch (ul_reason_for_call)
	{
		case DLL_PROCESS_ATTACH:
		case DLL_THREAD_ATTACH:
		case DLL_THREAD_DETACH:
		case DLL_PROCESS_DETACH:
			break;
    }
    return TRUE;
}

extern "C" STRATEGY_API void Create ( Environment *env )
{
	//��ʼ������
	env->userData=(void* ) new Mydata;

	Mydata* p;
	p=(Mydata*)env->userData;//???

	Init(env);
}

extern "C" STRATEGY_API void Destroy ( Environment *env )
{
	Mydata * p;
	p=(Mydata *)env->userData;

//	if(p->debug)
		fclose(p->debugfile);
	
	if ( env->userData != NULL )	delete ( Mydata * ) env->userData;
}

extern "C" STRATEGY_API void Strategy ( Environment *env )
//////////////�����ڻƶ� 
{
	Mydata * p;
	p=(Mydata *)env->userData;

	if(!p->locked)		// �� �жϳ����� ??
	{//ȷ������,blue or yellow
		if( env->home[0].pos.x < 50.0 )
			p->mygrand=true; /// �� = �ƶ�??
		else
			p->mygrand=false;
		p->locked=true;
	}
	
	See(env);			// Ԥ����
	Action(env);		//����
	End ( env );		//���ڴ���

}

void Init ( Environment *env ){
	env->userData=(void* ) new Mydata;
	Mydata* p;
	p=(Mydata*)env->userData;
	
	p->n=125;			//these two veriaty is for the test funtion!
	p->B_N=true;		//

	p->debug = true;//�� = ����
	
	p->time[1]=0;//��ʼ��ʱ��
	p->time[0]=125;//��ʼ��ʱ��

	p->bgoalball=0;//��ʼ����������
	p->nfreeball=0;//��ʼ����������
	p->nplaceball=0;

	p->ActiveAttacker = -1;
	p->NegativeAttacker = -1;
	p->Attacker = -1;
	p->chooserobot = 0;

	for(int i =0 ;i < 5;i++)
	{//�ҷ�
		p->robot[i].pos.x=InlitializeMyPosition_X[i];
		p->robot[i].pos.y=InlitializeMyPosition_Y[i];
		p->robot[i].pos.z=InlitializeMyPosition_Z[i];

		p->my_old_pos[i].x=InlitializeMyPosition_X[i];
		p->my_old_pos[i].y=InlitializeMyPosition_Y[i];
		p->my_old_pos[i].z=InlitializeMyPosition_Z[i];

		p->my_speed[i].x=0;
		p->my_speed[i].y=0;
		p->my_speed[i].z=0;

		p->my_old_velocity[i].x=0;
		p->my_old_velocity[i].y=0;
		p->my_old_velocity[i].z=0;
//�Է�
		p->opp[i].pos.x = InlitializeOppPosition_X[i];
		p->opp[i].pos.y = InlitializeOppPosition_Y[i];			
		p->opp[i].pos.z = InlitializeOppPosition_Z[i];

		p->op_old_pos[i].x = InlitializeOppPosition_X[i];	
		p->op_old_pos[i].y = InlitializeOppPosition_Y[i];
		p->op_old_pos[i].z = InlitializeOppPosition_Z[i];

		p->op_speed[0].x = 0;
		p->op_speed[0].y = 0;
		p->op_speed[0].z = 0;

	}

	p->locked=false;// �Ƿ��ж��˳��� ??
	p->mygrand=true;// �� = �ƶ�//�� = ���� ??

	p->ball_old.x = (PG_LEFT + PG_RIGHT) / 2.0;
	p->ball_old.y = (PG_TOP + PG_BOT) / 2.0;
	p->ball_old.z = 0;

	p->ball_cur.x = (PG_LEFT + PG_RIGHT) / 2.0;
	p->ball_cur.y = (PG_TOP + PG_BOT) / 2.0;
	p->ball_cur.z = 0;

	p->ball_pre.x = (PG_LEFT + PG_RIGHT) / 2.0;
	p->ball_pre.y = (PG_TOP + PG_BOT) / 2.0;
	p->ball_pre.z = 0;

	p->ball_speed.x =0;
	p->ball_speed.y =0;
	p->ball_speed.z =0;

	if(p->debug)
	{
		p->debugfile = fopen("c:\\strategy\\SCU_introduction.txt","w"); 
		if( !p->debugfile )
			p->debugfile = fopen("c:\\strategy\\SCU_introduction.doc","w");
		if( !p->debugfile )
			p->debug = false;
//		fclose(p->debugfile); 
	}

}

void See ( Environment *env ){
	Mydata * p;
	p=(Mydata *)env->userData;

	int i=0;

	if(p->mygrand)//������ҷ��ǻƶӵĻ�
	{///���ر任��������任
		//�ҷ��ǻƶ�

		p->gameState = env->gameState ;

		p->ball_cur.x = env->currentBall.pos.x;		//������仯
		p->ball_cur.y = env->currentBall.pos.y;


		for(i=0;i<5;i++)
		{
			p->robot[i].pos.x = env->home[i].pos.x ;	//�ҷ���Ա����任
			p->robot[i].pos.y = env->home[i].pos.y ;
			p->robot[i].rotation= env->home[i].rotation;
			
			p->opp[i].pos.x =env->opponent[i].pos.x;	//�Է�����任
			p->opp[i].pos.y =env->opponent[i].pos.y;
			p->opp[i].rotation =env->opponent[i].rotation;
			RegulateAngle(p->opp[i].rotation);

		}
	}
	else
	{
		p->gameState = env->gameState ;

		p->ball_cur.x =PG_LEFT+PG_RIGHT + CORRECTX - env->currentBall.pos.x;		//������仯
		p->ball_cur.y =PG_BOT+PG_TOP + CORRECTY - env->currentBall.pos.y;

		for(i=0;i<5;i++)
		{
			p->robot[i].pos.x =PG_LEFT+PG_RIGHT + CORRECTX - env->home[i].pos.x ;	//�ҷ���Ա����任
			p->robot[i].pos.y =PG_BOT+PG_TOP + CORRECTY - env->home[i].pos.y ;
			p->robot[i].rotation= 180.0 + env->home[i].rotation;
			RegulateAngle(p->robot[i].rotation);
			
			p->opp[i].pos.x = PG_LEFT+PG_RIGHT + CORRECTX- env->opponent[i].pos.x;	//�Է�����任
			p->opp[i].pos.y = PG_BOT+PG_TOP + CORRECTY - env->opponent[i].pos.y;
			p->opp[i].rotation =  180 + env->opponent[i].rotation;
			RegulateAngle(p->robot[i].rotation);
		}
	}

////��һ�δ����ٶ�  (�ϴ�)
	for(i=0;i<5;i++)
	{///speed
		p->my_speed[i].x = ( p->robot[i].pos.x - p->my_old_pos[i].x);//70Ϊ����ϵ�����д�����
		p->my_speed[i].y = ( p->robot[i].pos.y - p->my_old_pos[i].y);
		p->my_speed[i].z = Atan(p->my_speed[i].y,p->my_speed[i].x);//�õ��ҷ������˵� �˶����� ��ת���ٶ�,������

		p->op_speed[i].x = ( p->opp[i].pos.x - p->op_old_pos[i].x);
		p->op_speed[i].y = ( p->opp[i].pos.y - p->op_old_pos[i].y);
		p->op_speed[i].z = Atan(p->op_speed[i].y,p->op_speed[i].x);//�õ��з�������
	}

	p->ball_speed.x = p->ball_cur.x - p->ball_old.x;
	p->ball_speed.y = p->ball_cur.y - p->ball_old.y;
	p->ball_speed.z = Atan( p->ball_speed.y , p->ball_speed.x ); //�õ������Ϣ��
////���ϲ�����ֱ����Ϊ��ǰ����
///���濪ʼ����ǰ����ʵ����
//////����robot����
	double v,a,b,c,omiga,angle;
	for(i=0;i<5;i++)
	{
		omiga = p->robot[i].rotation - p->my_old_pos[i].z;
		RegulateAngle(omiga);
		omiga = AngleOne(omiga,p->my_old_velocity[i].x , p->my_old_velocity[i].y);
		c = p->robot[i].rotation;	
		p->robot[i].rotation+=omiga;
		RegulateAngle(p->robot[i].rotation);

		v = sqrt((p->my_speed[i].x * p->my_speed[i].x) + (p->my_speed[i].y * p->my_speed[i].y));
		angle = p->robot[i].rotation - p->my_speed[i].z;
		RegulateAngle(angle);
		if(angle >-90 && angle < 90 )		
			v=v;
		else
			v=-v;

		v=VelocityOne(v,p->my_old_velocity[i].x , p->my_old_velocity[i].y);
		a=p->robot[i].pos.x;
		b=p->robot[i].pos.y;

		p->robot[i].pos.x += v*cos( p->robot[i].rotation * PI / 180) ;
		p->robot[i].pos.y += v*sin( p->robot[i].rotation * PI / 180) ;
///����ײǽ
//���������

////����ײǽ		
		p->my_old_pos[i].x =a;
		p->my_old_pos[i].y =b;
		p->my_old_pos[i].z =c;

		p->my_speed[i].x = ( p->robot[i].pos.x - p->my_old_pos[i].x );	//70Ϊ����ϵ�����д�����
		p->my_speed[i].y = ( p->robot[i].pos.y - p->my_old_pos[i].y );
		p->my_speed[i].z = Atan( p->my_speed[i].y , p->my_speed[i].x );
	}

/////////	 Ԥ���������

	double x,y;
	x = p->ball_cur.x ;
	y = p->ball_cur.y ;
	
	PredictBall(env);		//���������λ��
	p->ball_cur = p->ball_pre;

	p->ball_old.x = x;
	p->ball_old.y = y;

	PredictBall(env);		//Ԥ����һ�����λ��

	p->ball_speed.x = p->ball_cur.x - p->ball_old.x;
	p->ball_speed.y = p->ball_cur.y - p->ball_old.y;
	p->ball_speed.z = Atan( p->ball_speed.y , p->ball_speed.x );
/////////	 Ԥ���������

	p->WIB = CheckBall(env);
	CheckBlockInfo(env);
	///��ʱ
	///�����ô���
	p->time[1]++;
	if(p->time[1]==60){
		p->time[1]=0;
		p->time[0]++;
	}
	if(p->ball_cur.y>42){
	  p->ActiveAttacker=3;
	  p->Attacker=1;
	  p->NegativeAttacker=4;
	  p->Defender=2;
	}
	else{
		p->ActiveAttacker=4;
		p->Attacker=2;
		p->NegativeAttacker=3;
		p->Defender=1;
	}

	if(p->debug)
	{
		fprintf(p->debugfile, "%d,",p->time[1]);
	}
}

void End ( Environment *env ){
	//��һЩ��ɨ�Ĺ���
	//��һЩ��¼������
	
	Mydata * p;
	p=(Mydata *)env->userData;
	
	int i=0;

	for(i=0;i<5;i++){//�ٶ�
		env->home[i].velocityLeft = p->robot[i].velocityLeft;
		env->home[i].velocityRight = p->robot[i].velocityRight;
		
		p->my_old_velocity[i].x=p->robot[i].velocityLeft;
		p->my_old_velocity[i].y=p->robot[i].velocityRight;
	}

	if(p->mygrand){///���ر任��������任
		p->ball_old.x = env->currentBall.pos.x;		//������仯
		p->ball_old.y = env->currentBall.pos.y;
		//p->ball_cur.z = env->currentBall.pos.z;

		for(i=0;i<5;i++){
			p->my_old_pos[i].x = env->home[i].pos.x ;	//�ҷ���Ա����任
			p->my_old_pos[i].y = env->home[i].pos.y ;
			//p->robot[i].pos.z = env->home[i].pos.z ;
			p->my_old_pos[i].z= env->home[i].rotation;
			
			p->op_old_pos[i].x =env->opponent[i].pos.x;	//�Է�����任
			p->op_old_pos[i].y =env->opponent[i].pos.y;
			//p->opp[i].pos.z = env->opponent[i].pos.z;
			p->op_old_pos[i].z =env->opponent[i].rotation;
			RegulateAngle(p->op_old_pos[i].z);
		}
	}
	else{
		p->ball_old.x =PG_LEFT+PG_RIGHT + CORRECTX - env->currentBall.pos.x;		//������仯
		p->ball_old.y =PG_BOT+PG_TOP + CORRECTY - env->currentBall.pos.y;
		//p->ball_cur.z = env->currentBall.pos.z;

		for(i=0;i<5;i++){
			p->my_old_pos[i].x =PG_LEFT+PG_RIGHT + CORRECTX - env->home[i].pos.x ;	//�ҷ���Ա����任
			p->my_old_pos[i].y =PG_BOT+PG_TOP + CORRECTY - env->home[i].pos.y ;
			//p->robot[i].pos.z = env->home[i].pos.z ;
			p->my_old_pos[i].z = 180.0 + env->home[i].rotation;
			RegulateAngle(p->my_old_pos[i].z);
			
			p->op_old_pos[i].x = PG_LEFT+PG_RIGHT + CORRECTX- env->opponent[i].pos.x;	//�Է�����任
			p->op_old_pos[i].y = PG_BOT+PG_TOP + CORRECTY - env->opponent[i].pos.y;
			//p->opp[i].pos.z = env->opponent[i].pos.z;
			p->op_old_pos[i].z =  180 + env->opponent[i].rotation;
			RegulateAngle(p->op_old_pos[i].z);
		}
	}

	if(p->debug){
		fprintf(p->debugfile, "\n");
	}
}

void Action ( Environment *env ){
    GAMESTATE(env);
	Mydata * p;
	p=(Mydata *)env->userData;
	switch(p->gameState){
		case 0:
			NormalGame(env);
			break;
		case 1:
			FreeBallGame(env);
			break;
		case 2:
			PlaceBallGame(env);
			break;
		case 3:
			PenaltyBallGame(env);
			break;
		case 4:
			FreeKickGame(env);
			break;
		case 5:
			GoalKickGame(env);
			break;
	}
}