#include "constants.h"
#include "function.h"
/************************角度处理*********************************/
//1.RegulateAngle   规范angle的大小在（-180，+180）之间     
//2.Atan 在坐标平面与水平面的夹角
//3.Atan 求两个点之间的夹角(-180~180)
/****************************************************************/
void RegulateAngle(double &angle){
	while(angle >= 180.0 )angle-=360.0;
	while(angle < -180.0 )angle+=360.0;
}

double Atan(double y,double x){
	if(x != 0.0 || y != 0.0) 
	return 180*atan2(y,x)/PI;
	else return 0.0;
}

double Atan(Vector3D begin,Vector3D end){
	double y,x;
	y=end.y - begin.y ;
	x=end.x - begin.x ;
	return Atan(y,x);
}

/****************************直线处理*********************************/
//1.Distance  两点之间距离
/*******************************************************************/
double Distance (Vector3D pos1,Vector3D pos2){
	return sqrt( (pos1.x-pos2.x)*(pos1.x-pos2.x) + (pos1.y-pos2.y)*(pos1.y-pos2.y) );
}

/****************************基本动作*********************************/
//1.Angle 转向angle
//2.Angle让robot转到正对pos的方向
//3.PAngle 让robot朝angle的方向跑，并且speed控制它的最大轮速
//4.PositionAndStop 让robot 跑到pos，并且停下来， bestangle 是停下来之后的朝向，limit	控制停在pos附近的距离
//5.GoaliePosition（守门员）同上
//6.PositionAndStopX 让robot 跑到pos，并且停下来原地旋转，Xangle	旋转的角速度，limit	控制停在pos附近的距离
//7.PositionBallX 让robot 跑到pos，并且停下来原地旋转，Xangle	旋转的角速度,limit	控制球和robot的距离,如果球和队员的距离大于limit则不旋转
//8.PositionAndThrough 让robot以最快MAX 冲向pos，中间没有减速控制
/*******************************************************************/

void Angle( Environment *env, int robot,double angle){
	Mydata * p;
	p=(Mydata *)env->userData;

	double speed = 0;		//和pangle接轨
	double accuracy=1;
	double turnangle=0,nextangle=0;
	double FF=125;		//最大减速度

	turnangle = angle -p->robot[robot].rotation;
	RegulateAngle(turnangle);
	if(turnangle < 1 && turnangle >-1){
		Velocity(env,robot,0,0);
		return ;
	}	
	else if(turnangle < 2 && turnangle >-2)
		FF=10;
	else if( turnangle >-3 && turnangle < 3)
		FF=15;
	else if( turnangle >-5 && turnangle < 5)
		FF=30;

	double v=p->robot[robot].rotation - p->my_old_pos[robot].z ;
	RegulateAngle(v);

	double v1=v;
	double f=0;	//相当于减速时,右轮速度，
//	int n=0;
	bool turnleft=true;			//判断小车是否是该向左转
	double a=ANGLE_A;
	double b=ANGLE_B;

	if(turnangle > 90){
		turnleft=false;
		turnangle-=180;
	}
	else if(turnangle >0){
		turnleft=true;	
	}
	else if(turnangle > -90){
		turnleft=false;	
	}
	else{ //<-90时
		turnleft=true;
		turnangle+=180;	
	}

	if(turnleft){//
		f=-FF;
		v1=AngleOne(v1,speed+f,speed-f);		//v1+=a *( -b *f-v1);
		nextangle+=v1;
		do{//whether to reduce
			//收敛!!
			v1 =AngleOne(v1,speed-f,speed+f);//+= a *( b *f-v1);		// v1   
			nextangle+=v1;
		}while( v1 > 0  );		
		nextangle-=v1;
		if(nextangle < turnangle){//不满足减速条件 所以 f 取相反数
			Velocity(env,robot,speed+f,speed-f);
		}
		else{//reduce
			v1 = AngleOne(v,speed-f,speed+f);  //v + a *( b *f-v);
			if( v1 < 0 ){
				do{//该降低功率了
					f++;
					v1 = AngleOne(v,speed-f,speed+f);  //v + a *( b *f-v);
				}while( v1 < turnangle && f <FF);
			}
			Velocity(env,robot,speed-f,speed+f);
		}		
	}
	else{//
		f=FF;
		v1=AngleOne(v1,speed+f,speed-f);		//v1+=a *( -b *f-v1);
		nextangle+=v1;		
		do{//whether to reduce
			v1 =AngleOne(v1,speed-f,speed+f);//+= a *( b *f-v1);		// v1   
			nextangle+=v1;
		}while( v1 < 0 );		
		nextangle-=v1;
		if(nextangle > turnangle){//不满足减速条件 所以 f 取相反数
			Velocity(env,robot,speed+f,speed-f);
		}
		else{//reduce
			v1 = AngleOne(v,speed-f,speed+f);  //v + a *( b *f-v);
			if( v1 > 0 ){
				do{//该降低功率了
					f--;
					v1 = AngleOne(v,speed-f,speed+f);  //v + a *( b *f-v);
				}while( v1 > turnangle && f >-FF);
			}
			Velocity(env,robot,speed-f,speed+f);
		}		
	}

}

void Angle( Environment *env, int robot,Vector3D pos){
	Mydata * p;
	p=(Mydata *)env->userData;

	double speed = 0;		//和pangle接轨
	double accuracy=1;
	double turnangle=0,nextangle=0;
	double FF=125;		//最大减速度
	double angle=0;
	angle = Atan(p->robot[robot].pos , pos);

	turnangle = angle -p->robot[robot].rotation;
	RegulateAngle(turnangle);
	
	if(turnangle < 1 && turnangle >-1){
		Velocity(env,robot,0,0);
		return ;
	}
	else if(turnangle < 2 && turnangle >-2)
		FF=10;
	else if( turnangle >-3 && turnangle < 3)
		FF=15;
	else if( turnangle >-5 && turnangle < 5)
		FF=30;

	double v=p->robot[robot].rotation - p->my_old_pos[robot].z ;
	RegulateAngle(v);
	double v1=v;
	double f=0;	//相当于减速时,右轮速度，
//	int n=0;
	bool turnleft=true;			//判断小车是否是该向左转
	double a=ANGLE_A;
	double b=ANGLE_B;

	if(turnangle > 90){
		turnleft=false;
		turnangle-=180;
	}
	else if(turnangle >0){
		turnleft=true;	
	}
	else if(turnangle > -90){
		turnleft=false;	
	}
	else{
		turnleft=true;
		turnangle+=180;		
	}

	if(turnleft){//
		f=-FF;
		v1=AngleOne(v1,speed+f,speed-f);		//v1+=a *( -b *f-v1);
		nextangle+=v1;
		do{//whether to reduce
			//收敛!!
			v1 =AngleOne(v1,speed-f,speed+f);//+= a *( b *f-v1);		// v1   
			nextangle+=v1;
		}while( v1 > 0  );		
		nextangle-=v1;
		if(nextangle < turnangle){//不满足减速条件 所以 f 取相反数
			Velocity(env,robot,speed+f,speed-f);
		}
		else{//reduce	
			v1 = AngleOne(v,speed-f,speed+f);  //v + a *( b *f-v);
			if( v1 < 0 ){
				do{//该降低功率了
					f++;
					v1 = AngleOne(v,speed-f,speed+f);  //v + a *( b *f-v);
				}while( v1 < turnangle && f <FF);
			}
			Velocity(env,robot,speed-f,speed+f);
		}		
	}
	else{//
		f=FF;
		v1=AngleOne(v1,speed+f,speed-f);		//v1+=a *( -b *f-v1);
		nextangle+=v1;		
		do{//whether to reduce
			v1 =AngleOne(v1,speed-f,speed+f);//+= a *( b *f-v1);		// v1   
			nextangle+=v1;
		}while( v1 < 0 );		
		nextangle-=v1;
		if(nextangle > turnangle){//不满足减速条件 所以 f 取相反数
			Velocity(env,robot,speed+f,speed-f);
		}
		else{//reduce
			v1 = AngleOne(v,speed-f,speed+f);  //v + a *( b *f-v);
			if( v1 > 0 ){
				do{//该降低功率了
					f--;
					v1 = AngleOne(v,speed-f,speed+f);  //v + a *( b *f-v);
				}while( v1 > turnangle && f >-FF);
			}
			Velocity(env,robot,speed-f,speed+f);
		}		
	}
}

void PAngle( Environment *env, int robot,double angle,double speed)
{
	Mydata * p;
	p=(Mydata *)env->userData;
	
	double accuracy=1;
	double turnangle=0,nextangle=0;
	turnangle = angle -p->robot[robot].rotation;
	RegulateAngle(turnangle);
	double v=p->robot[robot].rotation - p->my_old_pos[robot].z ;
	RegulateAngle(v);
	double v1=v;
	double FF=125;		//最大减速度
	double f=0;	//相当于减速时,右轮速度，
//	int n=0;
	bool turnleft=true;			//判断小车是否是该向左转
	double a=ANGLE_A;
	double b=ANGLE_B;
	
	bool face ;
	if(  turnangle < 90 && turnangle > -90 ){//检查是否正面跑位	
		face = true;
		speed = speed;
	}
	else{
		face = false;
		speed = -speed;
	}
	if(turnangle > 90){
		turnleft=false;
		turnangle-=180;
	}
	else if(turnangle >0){
		turnleft=true;	
	}
	else if(turnangle > -90){
		turnleft=false;	
	}
	else{
		turnleft=true;
		turnangle+=180;		
	}

	if(turnleft)
	{//
		f=-FF;
		v1=AngleOne(v1,speed+f,speed-f);		//v1+=a *( -b *f-v1);
		nextangle+=v1;
		do{//whether to reduce
			//收敛!!
			v1 =AngleOne(v1,speed-f,speed+f);//+= a *( b *f-v1);		// v1   
			nextangle+=v1;
		}while( v1 > 0  );		
		nextangle-=v1;
		if(nextangle < turnangle)
		{//不满足减速条件 所以 f 取相反数
			Velocity(env,robot,speed+f,speed-f);
		}
		else{//reduce	
			v1 = AngleOne(v,speed-f,speed+f);  //v + a *( b *f-v);
			if( v1 < 0 ){
				do{//该降低功率了
					f++;
					v1 = AngleOne(v,speed-f,speed+f);  //v + a *( b *f-v);
				}while( v1 < turnangle && f <125);
			}
			Velocity(env,robot,speed-f,speed+f);
		}		
	}
	else{//
		f=FF;
		v1=AngleOne(v1,speed+f,speed-f);		//v1+=a *( -b *f-v1);
		nextangle+=v1;		
		do{//whether to reduce
			v1 =AngleOne(v1,speed-f,speed+f);//+= a *( b *f-v1);		// v1   
			nextangle+=v1;
		}while( v1 < 0 );		
		nextangle-=v1;
		if(nextangle > turnangle)
		{//不满足减速条件 所以 f 取相反数
			Velocity(env,robot,speed+f,speed-f);
		}
		else{//reduce
			v1 = AngleOne(v,speed-f,speed+f);  //v + a *( b *f-v);
			if( v1 > 0 ){
				do{//该降低功率了
					f--;
					v1 = AngleOne(v,speed-f,speed+f);  //v + a *( b *f-v);
				}while( v1 > turnangle && f >-125);
			}
			Velocity(env,robot,speed-f,speed+f);
		}		
	}
}

void PositionAndStop(Environment *env,int  robot,Vector3D pos ,double bestangle,double limit){
	Mydata * p;
	p=(Mydata *)env->userData;
	
	double anglespeedmax=0;//控制转交速度的变量
	double vmax=125;//默认的跑位加速度
	double Limitedangle=2;//默认减速范围

	if( limit < 0.5 )
		limit =0.5;
	double Limiteddis=limit;//减速范围有一个下限，保证不会来回跑动
	
	double  distance;//robot和目标点的距离
	double turnangle,posangle,vangle;//转动角度 ，目标点相对robot的角度，速度的绝对角度
	double dx,dy;//pos  和robot的坐标差
	double a=SPEED_A;//参数
	double b=SPEED_B;
	double v,v1;//临时保存速度的大小!!!
	double f=vmax;//加速度变量
	double s=0;	//预测的减速位移(路程)
	int n=0;//跑位的步数
	bool face=true;//判断小车是否是正面前进

	v= sqrt(p->my_speed[robot].x * p->my_speed[robot].x + p->my_speed[robot].y*p->my_speed[robot].y);
	//临时保存速度的大小!!!
	dx = pos.x - p->robot[robot].pos.x ;		//pos  和robot的坐标差
	dy = pos.y - p->robot[robot].pos.y ;
	
	distance = Distance(p->robot[robot].pos , pos);
	posangle = Atan(dy,dx);

	turnangle = p->robot[robot].rotation - posangle;		//转动角度 
	RegulateAngle(turnangle);

	if(turnangle > 90){//判断小车是否是正面前进
		face=false;
		turnangle-=180;
	}
	else if(turnangle < -90){
		face=false;
		turnangle+=180;
	}
	else{
		face=true;
	}

	vangle = p->my_speed[robot].z - p->robot[robot].rotation;		//速度的方向和robot正面的夹角
	RegulateAngle(vangle);					//主要用于最后控制减速度的大小
	if( vangle <-90 || vangle > 90 )//同时判断v的正负
		v=-v;

	if(face){//forward	跑位，如果后退的话  就v=0
		//设vl,vr=0 还是vl,vr=125 有一个条件有一个临界条件那就是 
		//v = SPEED_ZERO
		if(v < -SPEED_ZERO){
			Velocity(env,robot,0,0);
			return ;
		}
	}
	else if(v > SPEED_ZERO){//back	跑位，如果后退的话  就v=0
		Velocity(env,robot,0,0);
		return ;
	}

	v1=v;	//v1 is changing while program running 
			//whlie, v is not

	if(distance > Limiteddis ){//it is too early to count the steps
		//but the Limiteddis should be tested!!	to do...
		if(turnangle > Limitedangle || turnangle < -Limitedangle){//adjust angle
			/////////////////测试这一段
			//对于goalie这一段应该特别注意
			//发生变向	1.knock the robot,especially the opponent
			//	2.knock the wall
			// so the anglespeedmax is allowed ++ more!!
			if(turnangle > 20  || turnangle < -20)
				anglespeedmax = 0;
			else if(turnangle > 10  || turnangle < -10)
				anglespeedmax = 125;
			else if(turnangle > 5  || turnangle < -5)
				anglespeedmax = 180;
			else 
				anglespeedmax = 200;
			///////////////测试这一段
			PAngle(env,robot,posangle,anglespeedmax);
		}
		else{
			if(face)
				Velocity(env,robot,f,f);
			else
				Velocity(env,robot,-f,-f);
		}//it is time to rush
	}
	else{
		if(distance > 1){		//调整角度	return!!!!!!
			//radious of robot is about 1.5 ,so the distance is very short
			if(turnangle > Limitedangle || turnangle < -Limitedangle){	
				Angle(env,robot,posangle);
				return ;
			}
		}

		if(distance < 0.4){	//停止并转向		return!!!!!!
			//radious of robot is about 1.5 ,so the distance is very short
			if( v<0.1 && v>-0.1){	//the range of v shoud be tested 
				if(bestangle == 0)
					Velocity(env,robot,0,0);
				else
					Angle(env,robot,bestangle);
				return	;
			}
		}

		if(true){
			vmax=125;
			if(face){
				f=-vmax;		//减速度  为  0000000
				v1=VelocityOne(v1,-f,-f);		//加速一步
				s=v1;
				do{//whether to reduce
					if(v1 > SPEED_ZERO)	//as i said,this is limited
						v1=VelocityOne(v1,0,0);
					else 
						v1=VelocityOne(v1,f,f);
					s+=v1;
				}while( v1 > 0 );

				s-=v1;

				if(s < distance){//不满足减速条件加速
					Velocity(env,robot,-f,-f);
				}
				else{
					if(v > SPEED_ZERO)
						Velocity(env,robot,0,0);
					else{
						v1=VelocityOne(v,f,f);		//减速一步
						if( v1 < 0 )
						{
							do{//该降低功率了
								f++;		//f=-vmax;
								v1 = VelocityOne(v,f,f);
							}while( v1 < distance && f < vmax);	
						}
						Velocity(env,robot,f,f);
					}
				}
			}			
			else{
				f=vmax;		//减速度!!!!!
				v1=VelocityOne(v1,-f,-f);
				s=v1;
				do{//whether to reduce
					if(v1 < -SPEED_ZERO)	//as i said,this is limited
						v1=VelocityOne(v1,0,0);
					else 
						v1=VelocityOne(v1,f,f);
					s+=v1;
				}while( v1 < -0.1 );

				s-=v1;

				if(s > -distance){//不满足减速条件加速
					Velocity(env,robot,-f,-f);
				}
				else{
					if(v < -SPEED_ZERO)
						Velocity(env,robot,0,0);
					else{
						v1=VelocityOne(v,f,f);		//减速一步
						if( v1 > 0 ){
							do{//该降低功率了
								f--;		//f=-vmax;
								v1 = VelocityOne(v,f,f);
							}while( v1 > -distance && f > -vmax);	
						}
						Velocity(env,robot,f,f);
					}
				}
			}	
		}
	}
}

void GoaliePosition(Environment *env,int  robot,Vector3D pos ,double bestangle,double limit){	//考虑到可能的	急停和 急快速加速
	//特别作了优化
	//还有就是 被碰转后的转角过程 不能耽搁时间!!!
	//转角是最危险的过程

	Mydata * p;
	p=(Mydata *)env->userData;
	
	double anglespeedmax=0;		//控制转交速度的变量
	double vmax=125;			//默认的跑位加速度
	double Limitedangle=2;		//默认减速范围

	if( limit < 0.5 )
		limit =0.5;
	double Limiteddis=limit;	//减速范围有一个下限，保证不会来回跑动
	
	double  distance;			//robot和目标点的距离
	double turnangle,posangle,vangle;	//转动角度 ，目标点相对robot的角度，速度的绝对角度
	double dx,dy;				//pos  和robot的坐标差
	double a=SPEED_A;			//参数
	double b=SPEED_B;
	double v,v1;				//临时保存速度的大小!!!
	double f=vmax;				//加速度变量
	double s=0;					//预测的减速位移(路程)
	int n=0;					//跑位的步数
	bool face=true;			//判断小车是否是正面前进

	v= sqrt(p->my_speed[robot].x * p->my_speed[robot].x + p->my_speed[robot].y*p->my_speed[robot].y);
	//临时保存速度的大小!!!
	dx = pos.x - p->robot[robot].pos.x ;		//pos  和robot的坐标差
	dy = pos.y - p->robot[robot].pos.y ;
	
	distance = Distance(p->robot[robot].pos , pos);
	posangle = Atan(dy,dx);

	turnangle = p->robot[robot].rotation - posangle;		//转动角度 
	RegulateAngle(turnangle);

	if(turnangle > 90){//判断小车是否是正面前进
		face=false;
		turnangle-=180;
	}
	else if(turnangle < -90){
		face=false;
		turnangle+=180;
	}
	else{
		face=true;
	}

	vangle = p->my_speed[robot].z - p->robot[robot].rotation;		//速度的方向和robot正面的夹角
	RegulateAngle(vangle);					//主要用于最后控制减速度的大小
	if( vangle <-90 || vangle > 90 )		//同时判断v的正负
		v=-v;

	if(face){//forward	跑位，如果后退的话  就v=0
		//设vl,vr=0 还是vl,vr=125 有一个条件有一个临界条件那就是 
		//v = SPEED_ZERO
		if(v < -SPEED_ZERO){
			Velocity(env,robot,0,0);
			return ;
		}
	}
	else if(v > SPEED_ZERO){//back		跑位，如果后退的话  就v=0
		Velocity(env,robot,0,0);
		return ;
	}

	v1=v;	//v1 is changing while program running 
			//whlie, v is not

	if(distance > Limiteddis ){//it is too early to count the steps
		//but the Limiteddis should be tested!!	to do...
		if(turnangle > Limitedangle || turnangle < -Limitedangle){//adjust angle
			/////////////////测试这一段
			//对于goalie这一段应该特别注意
			//发生变向	1.knock the robot,especially the opponent
			//	2.knock the wall
			// so the anglespeedmax is allowed ++ more!!
			if(turnangle > 50  || turnangle < -50)
				anglespeedmax = 0;
			else if(turnangle > 30  || turnangle < -30)
				anglespeedmax = 80;
			else if(turnangle > 10  || turnangle < -10)
				anglespeedmax = 125;
			else if(turnangle > 5  || turnangle < -5)
				anglespeedmax = 180;
			else 
				anglespeedmax = 200;
			///////////////测试这一段

			PAngle(env,robot,posangle,anglespeedmax);
		}
		else{
			if(face)
				Velocity(env,robot,f,f);
			else
				Velocity(env,robot,-f,-f);
		}//it is time to rush
	}
	else{
		if(distance > 1){		//调整角度	return!!!!!!
			//radious of robot is about 1.5 ,so the distance is very short
			if(turnangle > Limitedangle || turnangle < -Limitedangle){	
				Angle(env,robot,posangle);
				return ;
			}
		}
		if(distance < 0.4){	//停止并转向		return!!!!!!
			//radious of robot is about 1.5 ,so the distance is very short
			if( v<0.1 && v>-0.1){	//the range of v shoud be tested 
				Angle(env,robot,bestangle);
				return	;
			}
		}
		if(true){
			vmax=125;
			if(face){
				f=-vmax;		//减速度  为  0000000
				v1=VelocityOne(v1,-f,-f);		//加速一步
				s=v1;
				do{//whether to reduce
					if(v1 > SPEED_ZERO)	//as i said,this is limited
						v1=VelocityOne(v1,0,0);
					else 
						v1=VelocityOne(v1,f,f);
					s+=v1;
				}while( v1 > 0 );

				s-=v1;

				if(s < distance){//不满足减速条件加速
					Velocity(env,robot,-f,-f);
				}
				else{
					if(v > SPEED_ZERO)
						Velocity(env,robot,0,0);
					else{
						v1=VelocityOne(v,f,f);		//减速一步
						if( v1 < 0 ){
							do{//该降低功率了
								f++;		//f=-vmax;
								v1 = VelocityOne(v,f,f);
							}while( v1 < distance && f < vmax);	
						}
						Velocity(env,robot,f,f);
					}
				}
			}			
			else{
				f=vmax;		//减速度!!!!!
				v1=VelocityOne(v1,-f,-f);
				s=v1;
				do{//whether to reduce
					if(v1 < -SPEED_ZERO)	//as i said,this is limited
						v1=VelocityOne(v1,0,0);
					else 
						v1=VelocityOne(v1,f,f);
					s+=v1;
				}while( v1 < -0.1 );

				s-=v1;

				if(s > -distance){//不满足减速条件加速
					Velocity(env,robot,-f,-f);
				}
				else{
					if(v < -SPEED_ZERO)
						Velocity(env,robot,0,0);
					else{
						v1=VelocityOne(v,f,f);		//减速一步
						if( v1 > 0 ){
							do{//该降低功率了
								f--;		//f=-vmax;
								v1 = VelocityOne(v,f,f);
							}while( v1 > -distance && f > -vmax);	
						}
						Velocity(env,robot,f,f);
					}
				}
			}	
		}
	}
}

void PositionAndStopX(Environment *env,int  robot,Vector3D pos ,double Xangle,double limit){
	Mydata * p;
	p=(Mydata *)env->userData;
	
	double anglespeedmax=0;		//控制转交速度的变量
	double vmax=125;
	double Limitedangle=2;

	if( limit < 2 )
		limit =2;
	double Limiteddis=limit;
	
	double  distance;
	double turnangle,posangle,vangle;
	double dx,dy;
	double a=SPEED_A;
	double b=SPEED_B;
	double v,v1;
	double f=vmax;
	double s=0;
	int n=0;
	bool face=true;			//判断小车是否是正面前进

	v= sqrt(p->my_speed[robot].x * p->my_speed[robot].x + p->my_speed[robot].y*p->my_speed[robot].y);
	
	dx = pos.x - p->robot[robot].pos.x ;
	dy = pos.y - p->robot[robot].pos.y ;
	
	distance = Distance(p->robot[robot].pos , pos);
	posangle = Atan(dy,dx);

	turnangle = p->robot[robot].rotation - posangle;		//think more!!
	RegulateAngle(turnangle);

	if(turnangle > 90){
		face=false;
		turnangle-=180;
	}
	else if(turnangle < -90){
		face=false;
		turnangle+=180;
	}
	else{
		face=true;
	}

	vangle = p->my_speed[robot].z - p->robot[robot].rotation;
	RegulateAngle(vangle);
	if( vangle <-90 || vangle > 90 )
		v=-v;
	v1=v;

	if(distance > Limiteddis ){//it is too early to count the steps
		if(turnangle > Limitedangle || turnangle < -Limitedangle){//adjust angle
			/////////////////测试这一段
			if(turnangle > 20  || turnangle < -20)
				anglespeedmax = 0;
			else if(turnangle > 10  || turnangle < -10)
				anglespeedmax = 125;
			else if(turnangle > 5  || turnangle < -5)
				anglespeedmax = 180;
			else 
				anglespeedmax = 200;
			///////////////测试这一段
			PAngle(env,robot,posangle,anglespeedmax);
		}
		else{
			if(face)
				Velocity(env,robot,f,f);
			else
				Velocity(env,robot,-f,-f);
		}//it is time to rush
	}
	else{
		if(distance > 1){		//调整角度	return!!!!!!
			if(turnangle > Limitedangle || turnangle < -Limitedangle){	
				Angle(env,robot,posangle);
				return ;
			}
		}
		if(distance < 1){	//停止并转向		return!!!!!!
			if( v<0.5 && v>-0.5){
				Velocity(env,robot,-Xangle,Xangle);
				return	;
			}
		}
		if(true){
			vmax=125;
			if(face){
				f=-vmax;		//减速度  为  0000000
				v1=VelocityOne(v1,-f,-f);		//加速一步
				s=v1;
				do{//whether to reduce
					if(v1 > SPEED_ZERO)	//as i said,this is limited
						v1=VelocityOne(v1,0,0);
					else 
						v1=VelocityOne(v1,f,f);
					s+=v1;
				}while( v1 > 0 );

				s-=v1;

				if(s < distance){//不满足减速条件加速
					Velocity(env,robot,-f,-f);
				}
				else{
					if(v > SPEED_ZERO)
						Velocity(env,robot,0,0);
					else{
						v1=VelocityOne(v,f,f);		//减速一步
						if( v1 < 0 ){
							do{//该降低功率了
								f++;		//f=-vmax;
								v1 = VelocityOne(v,f,f);
							}while( v1 < distance && f < vmax);	
						}
						Velocity(env,robot,f,f);
					}
				}
			}			
			else{
				f=vmax;		//减速度!!!!!
				v1=VelocityOne(v1,-f,-f);
				s=v1;
				do{//whether to reduce
					if(v1 < -SPEED_ZERO)	//as i said,this is limited
						v1=VelocityOne(v1,0,0);
					else 
						v1=VelocityOne(v1,f,f);
					s+=v1;
				}while( v1 < -0.1 );

				s-=v1;

				if(s > -distance){//不满足减速条件加速
					Velocity(env,robot,-f,-f);
				}
				else{
					if(v < -SPEED_ZERO)
						Velocity(env,robot,0,0);
					else{
						v1=VelocityOne(v,f,f);		//减速一步
						if( v1 > 0 ){
							do{//该降低功率了
								f--;		//f=-vmax;
								v1 = VelocityOne(v,f,f);
							}while( v1 > -distance && f > -vmax);	
						}
						Velocity(env,robot,f,f);
					}
				}
			}	
		}
	}
}

void PositionBallX(Environment *env,int  robot,Vector3D pos ,double Xangle,double limit) {
	Mydata * p;
	p=(Mydata *)env->userData;
	
	double anglespeedmax=0;		//控制转交速度的变量
	double vmax=125;
	double Limitedangle=2;

	if( limit <2.8 )
		limit =2.8;
	double Limiteddis=limit;
	
	double  distance;
	double turnangle,posangle,vangle;
	double dx,dy;
	double a=SPEED_A;
	double b=SPEED_B;
	double v;
	double f=vmax;
	bool face=true;			//判断小车是否是正面前进
	bool turnornot=false ;	//是否旋转,临时变量

	v= sqrt(p->my_speed[robot].x * p->my_speed[robot].x + p->my_speed[robot].y*p->my_speed[robot].y);
	
	dx = pos.x - p->robot[robot].pos.x ;
	dy = pos.y - p->robot[robot].pos.y ;
	
	distance = Distance(p->robot[robot].pos , pos);
	posangle = Atan(dy,dx);

	turnangle = p->robot[robot].rotation - posangle;		//think more!!
	RegulateAngle(turnangle);

	if(turnangle > 90){
		face=false;
		turnangle-=180;
	}
	else if(turnangle < -90){
		face=false;
		turnangle+=180;
	}
	else{
		face=true;
	}

	vangle = p->my_speed[robot].z - p->robot[robot].rotation;
	RegulateAngle(vangle);


	if(distance <  3.2)
		turnornot = true;
	else if(distance < 3.5 && v > 0.5 )
		turnornot = true ;
	else if(distance < 4.5 && v > 0.8)
		turnornot = true ;

	if(distance > Limiteddis  )	//不在旋转范围内  则不转
		turnornot = false ;
	
	if(turnornot){//满足条件 转!!!
		Velocity(env,robot,-Xangle,Xangle);
	}//否则跑位
	else if(turnangle > Limitedangle || turnangle < -Limitedangle){//adjust angle
		/////////////////测试这一段
		if(turnangle > 60  || turnangle < -60)
			anglespeedmax = 0;
		else if(turnangle > 30  || turnangle < -30)
			anglespeedmax = 100;
		else if(turnangle > 10  || turnangle < -10)
			anglespeedmax = 150;
		else 
			anglespeedmax = 200;
		///////////////测试这一段
		PAngle(env,robot,posangle,anglespeedmax);

	}
	else{
		if(face)
			Velocity(env,robot,f,f);
		else
			Velocity(env,robot,-f,-f);
	}//it is time to rush
}

void PositionAndThrough(Environment *env,int robot,Vector3D pos ,double MAX)
{
	Mydata * p;
	p=(Mydata *)env->userData;
	
	double anglespeedmax=0;		//控制转交速度的变量
	double max=MAX;
	double Limitedangle=2;
	double Limiteddis=0;
	double  distance;
	double turnangle,posangle,vangle;
	double dx,dy;
	double a=SPEED_A;
	double b=SPEED_B;
	double v,v1;
	double f;
	double s=0;
	int n=0;
	bool face=true;			//判断小车是否是正面前进

	v= sqrt(p->my_speed[robot].x * p->my_speed[robot].x + p->my_speed[robot].y*p->my_speed[robot].y);
	
	dx = pos.x - p->robot[robot].pos.x ;
	dy = pos.y - p->robot[robot].pos.y ;
	
	distance = Distance(p->robot[robot].pos , pos);
	posangle = Atan(dy,dx);

	turnangle = posangle - p->robot[robot].rotation;		//think more!!
	RegulateAngle(turnangle);

	if(turnangle > 90){
		face=false;
		turnangle-=180;
	}
	else if(turnangle < -90){
		face=false;
		turnangle+=180;
	}
	else{
		face=true;
	}

	vangle = p->my_speed[robot].z - posangle;
	RegulateAngle(vangle);
	if( vangle <-90 || vangle > 90 )
		v=-v;
	v1=v;

	if(distance > Limiteddis){//it is too early to count the steps
		if(turnangle > Limitedangle || turnangle < -Limitedangle){//adjust angle
			/////////////////测试这一段
			if(turnangle > 20  || turnangle < -20)
				anglespeedmax = 0;
			else if(turnangle > 10  || turnangle < -10)
				anglespeedmax = 125;
			else if(turnangle > 5  || turnangle < -5)
				anglespeedmax = 180;
			else 
				anglespeedmax = 200;
			///////////////测试这一段
			PAngle(env,robot,posangle,anglespeedmax);
		}
		else{
			f=max;
			if(face)
				Velocity(env,robot,f,f);
			else
				Velocity(env,robot,-f,-f);

		}//it is time to rush
	}
	else
	{


	}//abserlutely count
}

/************************踢球动作*********************************/
//1.Kick 让robot把球踢到ToPos的位置
//2.Kick 让robot 以与robot1的角度跑向它 
//3.Kick 向着steps个周期后球的位置踢
//4.shoot 射门
/*****************************************************************/


void Kick(Environment *env , int  robot , Vector3D ToPos )
{
	Mydata * p;
	p=(Mydata *)env->userData;

	double LimitedCircle = 3;
	Vector3D ball = Meetball_p(env,robot); //use the predictball position

	Vector3D RobotToBall; //人和球的相对位置
	RobotToBall.x = ball.x - p->robot[robot].pos.x ;
	RobotToBall.y = ball.y - p->robot[robot].pos.y ;
	RobotToBall.z = Atan(p->robot[robot].pos , ball);

	Vector3D BallToGate ; //球和球门的相对位置
	BallToGate.x = ToPos.x - ball.x ;
	BallToGate.y = ToPos.y - ball.y ;
	BallToGate.z = Atan(ball , ToPos);

	double gateangle=BallToGate.z;

	double RunAngle ;
	RunAngle = RobotToBall.z - BallToGate.z;
	RegulateAngle(RunAngle);

	double dis= Distance(ball,p->robot[robot].pos);

	if(dis > 3*LimitedCircle){
		Vector3D Center;
		if(RunAngle >0){
			BallToGate.z -=90;
		}
		else{
			BallToGate.z +=90;
		}

		RegulateAngle(BallToGate.z);
		Center.x = ball.x + LimitedCircle*cos(BallToGate.z /180.0);
		Center.y = ball.y + LimitedCircle*sin(BallToGate.z /180.0);
		Center.z = 0;
		double distance = Distance(Center,p->robot[robot].pos);

		if(distance < 2*LimitedCircle)
		{
			RunAngle = RobotToBall.z + RunAngle /2; // 可以调整  2 
		}
		else{
			double CenAngle= Atan(p->robot[robot].pos,Center);
			if(RunAngle <0){
				RunAngle = CenAngle-180*LimitedCircle*asin(LimitedCircle/distance)/3.142;
				RegulateAngle(RunAngle);
			}
			else{
				RunAngle = CenAngle+180*LimitedCircle*asin(LimitedCircle/distance)/3.142;
				RegulateAngle(RunAngle);
			}
		}

	}
	else{


		RunAngle = RobotToBall.z + RunAngle /2; // 可以调整  2 
		RegulateAngle(RunAngle);
	}
	double paraA=gateangle - p->robot[robot].rotation;
	if(paraA<0){
		paraA=-paraA;
	}
	if(paraA>90){
		paraA=180-paraA;
	}
	if(0.1>paraA)
	{
		paraA=0.1;
	}
	double paraB=125*dis/3*LimitedCircle*10/paraA;
	if(paraB>125){
		paraB=125;
	}
	PAngle(env,robot,RunAngle,paraB);
}

void Kick(Environment *env , int  robot ,int robot1){//踢人
	Mydata * p;
	p=(Mydata *)env->userData;
	Vector3D RobotToBall;		//人和球的相对位置
	RobotToBall.x = p->robot[robot1].pos.x- p->robot[robot].pos.x ;
	RobotToBall.y = p->robot[robot1].pos.y- p->robot[robot].pos.y ;
	RobotToBall.z = Atan(p->robot[robot].pos , p->robot[robot1].pos);
	
	Vector3D BallToGate ;		//球和球门的相对位置
	BallToGate.x = CONSTGATE.x- p->robot[robot1].pos.x;
	BallToGate.y = CONSTGATE.y- p->robot[robot1].pos.y;
	BallToGate.z = Atan(p->robot[robot1].pos,CONSTGATE);
	
	double RunAngle ;
	RunAngle = RobotToBall.z - BallToGate.z;
	RegulateAngle(RunAngle);
	
	RunAngle = RobotToBall.z + RunAngle /2 ;	// 可以调整  2 
	RegulateAngle(RunAngle);

	PAngle(env,robot,RunAngle,125);
}

void Kick(Environment *env,int robot,int steps,double limits){
	Mydata *p=(Mydata*)env->userData;
	double dx,dy,angle;
    
	dx=p->ball_cur.x-p->robot[robot].pos.x;
	dy=p->ball_cur.y-p->robot[robot].pos.y;
	angle=Atan(dy,dx);
    PredictBall(env,steps);

	if(angle<90&&angle>-90){
		if(p->ball_cur.y>41.8)
	      PositionBallX(env,robot,p->ball_pre,-125,3);
		else
		  PositionBallX(env,robot,p->ball_pre,125,3);
	}
	else 
      shoot(env,robot);
}

void shoot(Environment *env,int robot){
   Mydata *p=(Mydata *)env->userData;
   double w1,w2,alfa;
   double dx,dy;
   /*改过*/
   if(p->ball_cur.y>GBOT&&p->ball_cur.y<=(GTOP+GBOT)/2){
	   if(p->ball_speed.z>85 && p->ball_speed.z<95)
		  PositionBallX(env,robot,p->ball_cur,-90,4);
	   else if(p->ball_speed.z<-85&&p->ball_speed.z>-95)
		  PositionBallX(env,robot,p->ball_cur,90,4);
   }
   else if(p->ball_cur.y>=(GTOP+GBOT)/2&&p->ball_cur.y<=GTOP){
       if(p->ball_speed.z>85 && p->ball_speed.z<95)
		   PositionBallX(env,robot,p->ball_cur,-90,4);
	   else if(p->ball_speed.z<-85&&p->ball_speed.z>-95)
		   PositionBallX(env,robot,p->ball_cur,90,4);
   }
   
   if(p->robot[robot].pos.x<=p->ball_cur.x){     
	 PredictBall(env,2);  
     dx=GRIGHT-p->robot[robot].pos.x;
     dy=GTOP-p->robot[robot].pos.y;
     w1=Atan(dy,dx);

	 dx=GRIGHT-p->robot[robot].pos.x;
	 dy=GBOT-p->robot[robot].pos.y;
	 w2=Atan(dy,dx);

     dx=p->ball_pre.x-p->robot[robot].pos.x;
	 dy=p->ball_pre.y-p->robot[robot].pos.y;
     alfa=Atan(dy,dx);

	 if((w1-alfa)*(w2-alfa)<=0)
         PAngle(env,robot,alfa,125);
     else if(p->ball_cur.y<PG_BOT+4&&p->robot[robot].pos.y<PG_BOT+4)
		 Kick(env,robot,BOTGATE);
	 else if(p->ball_cur.y>PG_TOP-4&&p->robot[robot].pos.y>PG_BOT-4)
		 Kick(env,robot,TOPGATE);
	 else 
		 Kick(env,robot,CONSTGATE);
   }
   else 
	   Kick(env,robot,CONSTGATE);
}

/************************防止犯规*********************************/
//1.判断robot队员和球的距离是否再LENGTH规定的范围内返回true  or false
/*****************************************************************/

bool Within(Environment* env,int robot,double LENGTH){
	Mydata* p;
	p=(Mydata*)env->userData;

	const double steps=50;
	int who=robot;
	double dis;

	Vector3D ballgo={0,0,0};
	Vector3D robotgo={0,0,0};
	Vector3D ball=p->ball_cur;	
	
	ballgo.x = ball.x + steps * p->my_speed[who].x;
	ballgo.y = ball.y + steps * p->my_speed[who].y;
	
	dis=Distance(ballgo,p->robot[robot].pos);

	if(dis < LENGTH){
		return true;
	}
	return false;
}

/************************辅助函数*********************************/
//1.Velocity 修改左右轮速
//2.CheckBall返回分区号
//3.PredictBall 预测经过 steps 个周期之后球的位置
//4.Meetball_p 求出robot追到球的位置
//5.order 角色分配
//6.AngleOne计算在当前角速度omiga的基础上以左右轮速vl,vr控制，下一个周期达到的角速度
//7.VelocityOne 计算在当前速度speed的基础上以左右轮速vl,vr控制，下一个周期达到的速度 返回下一个周期达到的速度
/*****************************************************************/
void Velocity(Environment *env, int robot,double vl,double vr)
{
	Mydata * p;
	p=(Mydata *)env->userData;

	//vl,vr都有取值范围的!!!
	if(vl>125)vl=125;
	if(vl<-125)vl=-125;
	if(vr>125)vr=125;
	if(vr<-125)vr=-125;
	
	if(true){//速度的特别控制//重要，否则小车是颤抖的
		if(vl==0 && vr!=0)
			vl=0.00001;
		if(vr==0 && vl!=0)
			vr=0.00001;
	}
	p->robot[robot].velocityLeft = vl;
	p->robot[robot].velocityRight= vr;
}

int CheckBall(Environment *env)
{
	Mydata * p;
	p=(Mydata *)env->userData;
	int k;
    int WIB;

	double x1=21.5;
	double x2=50.1;
	double x3=78.6;
	
	double y1=25.1;
	double y2=41.8;
    double y3=58.6;
    
	Vector3D ball;
	ball.x=p->ball_cur.x;
	ball.y=p->ball_cur.y;

	if(ball.x<=x1)
		k=0;
	else if(ball.x>x1&&ball.x<=x2)
		k=4;
	else if(ball.x>x2&&ball.x<=x3)
		k=8;
	else if(ball.x>x3)
		k=12;
	if(ball.y<=y1)
		WIB=1+k;
	else if(ball.y>y1&&ball.y<=y2)
		WIB=2+k;
	else if(ball.y>y2&&ball.y<=y3)
		WIB=3+k;
	else if(ball.y>y3)
		WIB=4+k;
	return WIB;
}

void CheckBlockInfo(Environment * env)
{
	Mydata * p;
	p=(Mydata *)env->userData;
	int i,j,k;
	double x=0;//相对块x值
	double x_min=0;//当前块x最小值
	double x_max=0;//当前块x最大值 
	double x1=6.8118;//区域x最小值
	double x2=78.6;//区域x最大值

	double y=0;//相对块y值
	double y_min=0;//当前块y最小值
	double y_max=0;//当前块y最大值
	double y1=6.3730;//区域y最小值
    double y2=77.2392;//区域y最大值
	int block_size=2;//块大小
	int block_x_num=0;//x块总数量
	int block_y_num=0;//y块总数量
	int block_x_num_judge=0;//判断x用
	int block_y_num_judge=0;//判断y用

	block_x_num = (x2 - x1) / block_size;
	block_x_num++;//算入边界块
	block_y_num = (y2 - y1) / block_size;
	block_y_num++;//算入边界块

	for (i = 0;i < block_x_num;i++){
		for (j = 0;j < block_y_num;j++){
			x = i * block_size;
			x_min = x + x1;
			x_max = x_min + block_size;

			y = j * block_size;
			y_min = y + y1;
			y_max = y_min + block_size;

			//将当前块的中心x和y坐标，存入block当中
			p->block[i][j].x = x_min + (block_size / 2);
			if(block_y_num != 0){
				block_y_num_judge = block_y_num;//fix bug
				block_y_num_judge--;//fix bug
				if(j == block_y_num_judge){//fix bug
				//if(j == (block_y_num--)){//bug
					p->block[i][j].y = y2;
				} else {
					p->block[i][j].y = y_min + (block_size / 2);
				}
			} else {
				p->block[i][j].y = y2;
			}

			//循环读取队员的块编号
			for (k = 0;k < 5;k++){
				if ((p->robot[k].pos.x >= x_min) && (p->robot[k].pos.y >= y_min) && (p->robot[k].pos.x < x_max) && (p->robot[k].pos.y < y_max)){
					p->block_my[i][j] = 1;
					p->my_block_pos[k].x = i;
					p->my_block_pos[k].y = j;
				} else {
					p->block_my[i][j] = 0;
				}
				if ((p->opp[k].pos.x >= x_min) && (p->opp[k].pos.y >= y_min) && (p->opp[k].pos.x < x_max) && (p->opp[k].pos.y < y_max)){
					p->block_op[i][j] = 1;
					p->op_block_pos[k].x = i;
					p->op_block_pos[k].y = j;
				} else {
					p->block_op[i][j] = 0;
				}
			}

			//读取球的块坐标
			if ((p->ball_cur.x >= x_min) && (p->ball_cur.y >= y_min) && (p->ball_cur.x < x_max) && (p->ball_cur.y < y_max)){
				p->block_ball[i][j] = 1;
				p->ball_block_pos.x = i;
				p->ball_block_pos.y = j;
			} else {
				p->block_ball[i][j] = 0;
			}
		}
	}

	//存入块信息
	p->block_min.x = 0;
	p->block_min.y = 0;
	p->block_max.x = block_x_num;
	p->block_max.y = block_y_num;
}

void PredictBall(Environment *env,int steps)
{
	Mydata * p;
	p=(Mydata *)env->userData;
	
	Vector3D predictball;
	Vector3D ball_speed;
	int i=0;

	predictball.x = p->ball_cur.x;			//赋初值
	predictball.y = p->ball_cur.y;
	ball_speed.x = p->ball_speed.x ;
	ball_speed.y = p->ball_speed.y ;
	ball_speed.z = p->ball_speed.z ;
	
	for(i=0;i<steps;i++){
		predictball.x += ball_speed.x ;
		predictball.y += ball_speed.y ;
//处理撞墙
		if( predictball.x > PG_RIGHT ){
			predictball.x -= ball_speed.x ;	//retern
			predictball.y -= ball_speed.y ;
			ball_speed.x *=-SPEED_NORMAL;	//loose 
			ball_speed.y *= SPEED_TANGENT;
			predictball.x += ball_speed.x ;	//go on
			predictball.y += ball_speed.y ;
		}
		else if( predictball.x < PG_LEFT ){
			predictball.x -= ball_speed.x ;	//retern
			predictball.y -= ball_speed.y ;
			ball_speed.x *=-SPEED_NORMAL;	//loose 
			ball_speed.y *= SPEED_TANGENT;
			predictball.x += ball_speed.x ;	//go on
			predictball.y += ball_speed.y ;
		}
		else if( predictball.y < PG_BOT )		{
			predictball.x -= ball_speed.x ;	//retern
			predictball.y -= ball_speed.y ;
			ball_speed.x *= SPEED_TANGENT;	//loose 
			ball_speed.y *=-SPEED_NORMAL;
			predictball.x += ball_speed.x ;	//go on
			predictball.y += ball_speed.y ;
		}
		else if( predictball.y > PG_TOP ){
			predictball.x -= ball_speed.x ;	//retern
			predictball.y -= ball_speed.y ;
			ball_speed.x *= SPEED_TANGENT;	//loose 
			ball_speed.y *=-SPEED_NORMAL;
			predictball.x += ball_speed.x ;	//go on
			predictball.y += ball_speed.y ;
		}
		/////////////////对于边界时的设置
		if( predictball.x + predictball.y > PG_RIGHT +PG_TOP -CORNER){//右上
			double vx,vy;	
			vy=0.7071*ball_speed.y + 0.7071*ball_speed.x;	//变换1
			vx=-0.7071*ball_speed.y + 0.7071*ball_speed.x;

			predictball.x -= ball_speed.x ;	//retern
			predictball.y -= ball_speed.y ;
			vx *= SPEED_TANGENT;	//loose 
			vy *=-SPEED_NORMAL;
			ball_speed.y = 0.7071 * vy - 0.7071 * vx;	//变换2
			ball_speed.x = 0.7071 * vy + 0.7071 * vx;
			predictball.x += ball_speed.x ;	//go on
			predictball.y += ball_speed.y ;

		}
		else if( predictball.x + predictball.y < PG_LEFT +PG_BOT+CORNER){//左下
			double vx,vy;	
			vy=0.7071*ball_speed.y + 0.7071*ball_speed.x;	//变换1
			vx=-0.7071*ball_speed.y + 0.7071*ball_speed.x;
			predictball.x -= ball_speed.x ;	//retern
			predictball.y -= ball_speed.y ;
			vx *= SPEED_TANGENT;	//loose 
			vy *=-SPEED_NORMAL;
			ball_speed.y = 0.7071 * vy - 0.7071 * vx;	//变换2
			ball_speed.x = 0.7071 * vy + 0.7071 * vx;
			predictball.x += ball_speed.x ;	//go on
			predictball.y += ball_speed.y ;	
		}
		else if( predictball.x - predictball.y > PG_RIGHT -PG_BOT -CORNER){//右下
			double vx,vy;	
			vy=0.7071*ball_speed.y - 0.7071*ball_speed.x;	//变换1
			vx=0.7071*ball_speed.y + 0.7071*ball_speed.x;
			predictball.x -= ball_speed.x ;	//retern
			predictball.y -= ball_speed.y ;
			vx *= SPEED_TANGENT;	//loose 
			vy *=-SPEED_NORMAL;
			ball_speed.y = 0.7071 * vy + 0.7071 * vx;	//变换2
			ball_speed.x = -0.7071 * vy + 0.7071 * vx;
			predictball.x += ball_speed.x ;	//go on
			predictball.y += ball_speed.y ;			
		}
		else if( predictball.y - predictball.x > PG_TOP - PG_LEFT-CORNER){//左上
			double vx,vy;	
			vy=0.7071*ball_speed.y - 0.7071*ball_speed.x;	//变换1
			vx=0.7071*ball_speed.y + 0.7071*ball_speed.x;
			predictball.x -= ball_speed.x ;	//retern
			predictball.y -= ball_speed.y ;
			vx *= SPEED_TANGENT;	//loose 
			vy *=-SPEED_NORMAL;
			ball_speed.y = 0.7071 * vy + 0.7071 * vx;	//变换2
			ball_speed.x = -0.7071 * vy + 0.7071 * vx;
			predictball.x += ball_speed.x ;	//go on
			predictball.y += ball_speed.y ;			
		}
//处理四角		
	}
	p->ball_pre.x = predictball.x ;
	p->ball_pre.y = predictball.y ;
	p->ball_pre.z = Atan( ball_speed.y ,ball_speed.x );
}

Vector3D Meetball_p( Environment *env, int robot)
{//求出robot追到球的位置
	Mydata * p;
	p=(Mydata *)env->userData;

	Vector3D meetpoint={0,0,-1};
	double dis=Distance(p->ball_cur,p->robot[robot].pos);

	double t =0 ;
	double vb=0;
	double v=1.9;		//按照最大速度计算
	double pos_angle,b_sp_angle;

	pos_angle = Atan(p->ball_cur.y - p->robot[robot].pos.y , p->ball_cur.x - p->robot[robot].pos.x);
	b_sp_angle = p->ball_speed.z ;
	vb = (p->ball_speed.y * p->ball_speed.y + p->ball_speed.x * p->ball_speed.x);
	t = sin((b_sp_angle - pos_angle) * PI /180);
	t = vb* t*t;
	v=v*v;
	if( v > t )
	{
		v = sqrt( v - t) + sqrt( vb ) * cos((b_sp_angle - pos_angle) * PI /180);
		if( v > 0.1)
		{
			t = dis /v;	//得到步数
			meetpoint.x = p->ball_speed.x *t + p->ball_cur.x ;
			meetpoint.y = p->ball_speed.y *t + p->ball_cur.y ;
			meetpoint.z = t;
		}
	}
	return meetpoint;
}


void Order(Environment *env)
{
	Mydata * p;
	p=(Mydata *)env->userData;
    int i,j,k,a[4];
    double dis[4],b[4],temple;
    double dy=0;
	static record=2;
   
	b[0]=dis[0]=Distance(p->robot[1].pos,p->ball_cur);
	b[1]=dis[1]=Distance(p->robot[2].pos,p->ball_cur);
	b[2]=dis[2]=Distance(p->robot[3].pos,p->ball_cur);
	b[3]=dis[3]=Distance(p->robot[4].pos,p->ball_cur);

	switch(p->WIB){
	  case 1: 
      case 4:
	  case 5:
      case 6: 
	  case 7:
      case 8:
	  for(i=0;i<3;i++)
		  for(j=i+1;j<4;j++){
			  if(b[i]>b[j]){
				 temple=b[i];
				 b[i]=b[j];
				 b[j]=temple;
			  }
		  }
      for(i=0;i<4;i++)
		for(j=0;j<4;j++)
           if(dis[j]==b[i])
			   a[i]=j+1;

    p->ActiveAttacker=a[0];
	p->Attacker=a[1];
	p->NegativeAttacker=a[2];
	p->Defender=a[3];
        break;
	  case 2: 
	  case 3:
		  if(p->ball_cur.y>p->robot[0].pos.y){
			  i=p->ActiveAttacker;
		      p->ActiveAttacker=p->NegativeAttacker;
			  p->NegativeAttacker=i;
		  }
		  break;
      case 9:
      case 10:
	  case 11:
	  case 12:
	  case 13:

	  if(dis[1]<dis[2]){
         
		if(dis[2]<=dis[3]){
			i=2;
			j=3;
			k=4;
		}
		else if(dis[1]<=dis[3]){
			i=2;
			j=4;
			k=3;
		}
		else{
			i=4;
			j=2;
			k=3;
		}
	}
	else{

		if(dis[1]<=dis[3]){
			i=3;
			j=2;
			k=4;
		}
		else if(dis[2]<=dis[3]){
			i=3;
			j=4;
			k=2;
		}
		else{
			i=4;
			j=3;
			k=2;
		}
	}

	p->ActiveAttacker=i;
	p->Attacker=j;
	p->NegativeAttacker=k;
	record=p->NegativeAttacker;
        break;
	  case 14: 
    if(!Within(env,record,1))
		p->NegativeAttacker=record;

	else{
		if(p->ball_speed.z<0){
           dy=100;
		for(i=2;i<5;i++)
			if(p->robot[i].pos.y<dy){
				dy=p->robot[i].pos.y;
				record=i;
			}
			p->ActiveAttacker=p->NegativeAttacker;
			p->NegativeAttacker=record;
		}
		else{
			dy=0;
         for(i=2;i<5;i++)
			 if(p->robot[i].pos.y>dy){
				 dy=p->robot[i].pos.y;
				 record=i;
			 }
			 p->ActiveAttacker=p->NegativeAttacker;
			 p->NegativeAttacker=record;
		}    
	  }

	 for(i=2;i<5;i++)
       if(i!=p->NegativeAttacker&&i!=p->ActiveAttacker)
		   p->Attacker=i;

		  break;
	  case 15:
	   
	  if(!Within(env,record,1))
		p->NegativeAttacker=record;

	else{
		if(p->ball_speed.z<0){
           dy=100;
		for(i=2;i<5;i++)
			if(p->robot[i].pos.y<dy){
				dy=p->robot[i].pos.y;
				record=i;
			}
			p->ActiveAttacker=p->NegativeAttacker;
			p->NegativeAttacker=record;
		}
		else{
			dy=0;
         for(i=2;i<5;i++)
			 if(p->robot[i].pos.y>dy){
				 dy=p->robot[i].pos.y;
				 record=i;
			 }
			 p->ActiveAttacker=p->NegativeAttacker;
			 p->NegativeAttacker=record;
		}    
	}	
     for(i=2;i<5;i++)
       if(i!=p->NegativeAttacker&&i!=p->ActiveAttacker)
		   p->Attacker=i;

		  break;
	  case 16:

	  if(dis[1]<dis[2]){
         
		if(dis[2]<=dis[3]){
			i=2;
			j=3;
			k=4;
		}
		else if(dis[1]<=dis[3]){
			i=2;
			j=4;
			k=3;
		}
		else{
			i=4;
			j=2;
			k=3;
		}
	}
	else{

		if(dis[1]<=dis[3]){
			i=3;
			j=2;
			k=4;
		}
		else if(dis[2]<=dis[3]){
			i=3;
			j=4;
			k=2;
		}
		else{
			i=4;
			j=3;
			k=2;
		}
	}

	p->ActiveAttacker=i;
	p->Attacker=j;
	p->NegativeAttacker=k;
	record=p->NegativeAttacker;
	break;
	}
}


double AngleOne(double omiga,double vl,double vr)
{
//		omiga = p->robot[i].rotation - p->my_old_pos[i].z ;
//		RegulateAngle(omiga);
	if(vl>125)vl=125;
	if(vl<-125)vl=-125;
	if(vr>125)vr=125;
	if(vr<-125)vr=-125;
	double angle = (vr - vl)/2;

	RegulateAngle(omiga);
	omiga += ANGLE_A*(ANGLE_B* angle-omiga);
	if( vr > vl ){	
		if( vl >= 0 || vr <=0 ){
			omiga -= 4 * ANGLE_K * angle * angle;
		}
	}
	else if( vr < vl ){	
		if( vr >= 0 || vl <=0 ){
			omiga += 4 * ANGLE_K * angle * angle;
		}
	}
	RegulateAngle(omiga);		//应该没有大于180 的角速度罢
	return omiga;
}

double VelocityOne(double speed,double vl,double vr){
	if(vl>125)vl=125;
	if(vl<-125)vl=-125;
	if(vr>125)vr=125;
	if(vr<-125)vr=-125;

	if(speed > 3 || speed < -3)
		speed =0;
	if( vl==0 && vr==0 )
		speed += -SPEED_ODD * speed;
	else
		speed += SPEED_A*( SPEED_B*(vl +vr)/2-speed);
	return speed;
}

/**************************策略*********************************/
//keeper 守门员守门
//
//点球
//球门球
//争球
//任意球
/***************************************************************/

void Keeper ( Environment *env, int robot )
{//先校正姿态，再去拦球
	Mydata * p;
	p=(Mydata *)env->userData;
	Vector3D go;
		double OX=	PG_LEFT - (GTOP - GBOT);	// 该点为球门中心 向后移动半个球门
		double OY=	(PG_TOP + PG_BOT)/2;			//球门中心	
		double ballx=p->ball_cur.x;
		double bally=p->ball_cur.y	;
		double gx = p->robot[robot].pos.x;
		double gx_outline = PG_LEFT + 2.2;		//对x坐标的限定，防止离球门线太远了
		double gx_inline = PG_LEFT -1;
		double gy = p->robot[robot].pos.y;		//跑位点,初值为当前位置
		double goalline = PG_LEFT + 3;
		bool notout=true;
		bool   standby = true;	//限制x 坐标
		bool   XX=false;	//是否旋转

		if(ballx<PG_LEFT+0.7 && bally<BD_BOT+0.75 && bally>BD_BOT+0.7 && p->robot[robot].pos.y<BD_BOT+2 && p->robot[robot].pos.y>BD_BOT+1.4 && Distance(p->ball_cur,p->robot[robot].pos)<2.4){
		  Velocity(env,robot,-125,125);
		} 
		else if (ballx<PG_LEFT+0.7 && bally<BD_TOP+0.75 && bally<BD_TOP-0.7 && p->robot[robot].pos.y>BD_TOP-2 &&  p->robot[robot].pos.y>BD_TOP-1.4 && Distance(p->ball_cur,p->robot[robot].pos)<2.4 ){
            Velocity(env,robot,125,-125);
		}
			gy = OY + ( goalline - OX ) * (bally - OY)/(ballx - OX);
			if(notout){
				if(gy > GTOP+3)
					gy = GTOP+3;
				else if(gy < GBOT-3)
					gy = GBOT-3;
			}
			if(standby){
				if(gx > gx_outline )
					gx = gx_outline;
				else if(gx < gx_inline)
					gx = gx_inline;
			}
			if(fabs(p->ball_speed.y)<10 && p->ball_cur.x<15 && p){
				if(p->ball_cur.y>BD_TOP-3){
							gx=PG_LEFT-0.5;
							gy=BD_TOP-0.7;
				}
				else if(p->ball_cur.y<BD_BOT+3){
				        gx=PG_LEFT-0.5;
						gy=BD_BOT+0.7;
				}
				else  gx=PG_LEFT + 2.5;
			}
			else gx=PG_LEFT + 2.5; 
			go.x = gx;
			go.y = gy ;
			GoaliePosition(env,robot,go,90,1.5);
}

/****************************补充**********************************/



void Sweep(Environment * env,int robot){
	Mydata * p; 
	p=(Mydata *)env->userData;
    Vector3D pos;
    int add_time=0;
	pos.x=p->ball_cur.x;
	pos.y=p->ball_cur.y;
	add_time=(int)(fabs(Distance(p->robot[robot].pos,pos)-3.5)/1.2);
	if(add_time>18){
        PredictBall(env,20);
		pos.x=p->ball_pre.x;
	    pos.y=p->ball_pre.y;
	 	PositionAndStop(env,robot,pos);
	}
	else if(add_time==0){
        pos.x=p->ball_cur.x;
	    pos.y=p->ball_cur.y;
		PositionAndStop(env,robot,pos);
	}
	else
	{   
	    PredictBall(env,add_time-1);
		pos.x=p->ball_pre.x;
	    pos.y=p->ball_pre.y;
	 	PositionAndStop(env,robot,pos);
	}
}

int WhoseBall(Environment *env){
	Mydata * p;
	p=(Mydata *)env->userData;
	double temp1,temp2,dis;
	int k1,k2;

	temp1=Distance(p->ball_cur,p->robot[1].pos);
	k1=1;
	for(int i=2;i<5;i++){
	    dis=Distance(p->ball_cur,p->robot[i].pos);
		if(dis<temp1){
		   temp1=dis;
           k1=i;
		}
	}
    
	temp2=Distance(p->ball_cur,p->opp[1].pos);
	k1=1;
	for(i=2;i<5;i++){
	    dis=Distance(p->ball_cur,p->opp[i].pos);
		if(dis<temp2){
		   temp2=dis;
           k2=i;
		}
	}
    if(temp1<=temp2)    {p->whoseBall=1; return k1;}
	else {p->whoseBall=2; return k2;}
}

