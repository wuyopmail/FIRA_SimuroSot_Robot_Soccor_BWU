#include "constants.h"
#include "function.h"

void NormalGame(Environment *env){
  Mydata * p;
	p=(Mydata *)env->userData;
	PredictBall(env,2);
	if(p->ball_speed.x<0.5 && p->ball_speed.y<0,5) Kick(env,3,p->ball_pre);
	Vector3D pos,begin,end;
    static int flag=1;
	int i,count,x;
    double alfa;
    
	if(flag==1){
		PredictBall(env,2);
		Kick(env,3,p->ball_pre);
		Kick(env,4,p->ball_pre);
	    flag++;
	}
	Keeper(env,0);
    
	if(p->ball_cur.x>=50&&p->ball_cur.x<61)
		Kick(env,1,CONSTGATE);
	else if(p->ball_cur.x>=61&&p->ball_cur.x<78.6){
		   if(p->ball_cur.y<27.8){
			   pos.x=37.2;
			   pos.y=20;
		   }
		   else if(p->ball_cur.y<=58.5){
			   PredictBall(env,2);
			   pos.x=37.2;
			   pos.y=p->ball_pre.y;
		   }
		   else{
			   pos.x=37.2;
			   pos.y=66.2;
		   }
		   PositionAndStop(env,1,pos);
	}
	else  if(p->ball_cur.x>78.6&&((p->ball_cur.y<34&&p->ball_cur.y>6.3)||(p->ball_cur.y>48&&p->ball_cur.y<77.2))){
		       Order(env);
		       count=120;			 
			   while(count>0){
				  Kick(env,1,p->Attacker);
				  count--;
			   }
	}  		
	switch(p->WIB){
	case 1:
		Order(env);
    //ActiveAttacker,Attacker
		pos.x=50;
		pos.y=9;
		Kick(env,p->ActiveAttacker,pos);
		Kick(env,p->Attacker,pos);	
	//Defender
		pos.x=19;
		pos.y=58;
		for(i=1;i<5;i++){
			if(fabs(p->opp[i].pos.x-pos.x)<15&&fabs(p->opp[i].pos.y-pos.y)<15)
				break;
		}
		if(i<5)
			PositionAndThrough(env,p->Defender,p->opp[i].pos,125);
		else{ 
			pos.x=20;
			pos.y=58;
			PositionAndStop(env,p->Defender,pos,-145);
		}
	//NegativeAttacker
		pos.x=19;
		pos.y=27;
		for(i=1;i<5;i++){
			if(fabs(p->opp[i].pos.x-pos.x)<15&&fabs(p->opp[i].pos.y-pos.y)<15)
				break;
		}
		if(i<5)
			PositionAndThrough(env,p->NegativeAttacker,p->opp[i].pos,125);
		else{ 
			pos.x=20;
			pos.y=27;
			PositionAndStop(env,p->NegativeAttacker,pos,-145);
		}
		break;
	case 2:
		Order(env);
		//Defender
        pos.x=19;
		pos.y=58;
		for(i=1;i<5;i++){
			if(fabs(p->opp[i].pos.x-pos.x)<15&&fabs(p->opp[i].pos.y-pos.y)<15)
				break;
		}
		if(i<5)
			PositionAndThrough(env,p->Defender,p->opp[i].pos,125);
		else{ 
			pos.x=20;
			pos.y=58;
			PositionAndStop(env,p->Defender,pos,-145);
		}

//小禁区加上下两小块
		if(p->ball_cur.x<SRG_LEFT+0.8 && p->ball_cur.y<BRG_TOP+0.8 && p->ball_cur.y>BRG_BOT-0.8){
		//ActiveAttacker
			pos.x=8.9;
			pos.y=58;
			for(i=1;i<5;i++){
				if(fabs(p->opp[i].pos.x-pos.x)<2 && fabs(p->opp[i].pos.y-pos.y)<8) break;
			}
			if(i<5)
				PositionAndThrough(env,p->ActiveAttacker,p->opp[i].pos,125);
			else{ 
				pos.x=BRG_LEFT;
				pos.y=SRG_TOP;
				PositionAndStop(env,p->ActiveAttacker,pos,-145);
			}
         //Attacker
            pos.x=8.9;
			pos.y=27;
			for(i=1;i<5;i++){
				if(fabs(p->opp[i].pos.x-pos.x)<2 && fabs(p->opp[i].pos.y-pos.y)<8) break;
			}
			if(i<5)
				PositionAndThrough(env,p->Attacker,p->opp[i].pos,125);
			else{ 
				pos.x=BRG_LEFT;
				pos.y=BD_BOT;
				PositionAndStop(env,p->Attacker,pos,145);
			}
          //NegativeAttacker
            pos.x=19;
			pos.y=27;
			for(i=1;i<5;i++){
				if(fabs(p->opp[i].pos.x-pos.x)<15&&fabs(p->opp[i].pos.y-pos.y)<15)
					break;
			}
			if(i<5)
				PositionAndThrough(env,p->NegativeAttacker,p->opp[i].pos,125);
			else{ 
				pos.x=20;
				pos.y=27;
				PositionAndStop(env,p->NegativeAttacker,pos,145);
			}
		}
	//其他
        else{
		//ActiveAttacker,Attacker
			pos.x=50;
			pos.y=9;
			Kick(env,p->ActiveAttacker,pos);
			Kick(env,p->Attacker,pos);
	
		//NegativeAttacker
            pos.x=SRG_LEFT;
			pos.y=p->ball_cur.y;
			PositionAndStop(env,p->NegativeAttacker,pos);
		}		
		break;

	case 3:
		Order(env);
	//Defender
        pos.x=19;
		pos.y=27;
		for(i=1;i<5;i++){
			if(fabs(p->opp[i].pos.x-pos.x)<15&&fabs(p->opp[i].pos.y-pos.y)<15)
				break;
		}
		if(i<5)	PositionAndThrough(env,p->Defender,p->opp[i].pos,125);
		else{
			pos.x=20;
			pos.y=27;
			PositionAndStop(env,p->Defender,pos,145);
		}
	//小禁区
		if(p->ball_cur.x<SRG_LEFT+0.8 && p->ball_cur.y<BRG_TOP+0.8 && p->ball_cur.y>BRG_BOT-0.8){
		//ActiveAttacker
			pos.x=8.9;
			pos.y=27;
			for(i=1;i<5;i++){
				if(fabs(p->opp[i].pos.x-pos.x)<2 && fabs(p->opp[i].pos.y-pos.y)<8) break;
			}
			if(i<5)
				PositionAndThrough(env,p->ActiveAttacker,p->opp[i].pos,125);
			else{ 
				pos.x=BRG_LEFT;
				pos.y=BD_BOT;
				PositionAndStop(env,p->ActiveAttacker,pos,145);
			}
         //Attacker
            pos.x=8.9;
			pos.y=58;
			for(i=1;i<5;i++){
				if(fabs(p->opp[i].pos.x-pos.x)<2 && fabs(p->opp[i].pos.y-pos.y)<8) break;
			}
			if(i<5)
				PositionAndThrough(env,p->Attacker,p->opp[i].pos,125);
			else{ 
				pos.x=BRG_LEFT;
				pos.y=BD_TOP;
				PositionAndStop(env,p->Attacker,pos,-145);
			}
			pos.x=19;
			pos.y=58;
			for(i=1;i<5;i++){
				if(fabs(p->opp[i].pos.x-pos.x)<15&&fabs(p->opp[i].pos.y-pos.y)<15)
					break;
			}
			if(i<5)
				PositionAndThrough(env,p->NegativeAttacker,p->opp[i].pos,125);
			else{ 
				pos.x=20;
				pos.y=27;
				PositionAndStop(env,p->NegativeAttacker,pos,-145);
			}

		}
	//其他
        else{
		//ActiveAttacker,Attacker
			pos.x=50;
			pos.y=74;
			Kick(env,p->ActiveAttacker,pos);
			Kick(env,p->Attacker,pos);
		//NegativeAttacker
            pos.x=SRG_LEFT;
			pos.y=p->ball_cur.y;
			PositionAndStop(env,p->NegativeAttacker,pos);

			if(p->ball_cur.x<12&&p->ball_cur.y<59)
				PositionAndThrough(env,p->NegativeAttacker,p->robot[0].pos);
			else if(p->ball_cur.x>PG_LEFT+4&&p->ball_cur.x<17){
				pos.x=9;
				pos.y=77;
				Kick(env,p->NegativeAttacker,pos);
			}
			else{
				pos.x=20;
				pos.y=p->robot[0].pos.y;
				PositionAndStop(env,p->NegativeAttacker,pos);
			}
		}
		break;
	case 4:
		Order(env);
	//ActiveAttacker,Attacker
		pos.x=50;
		pos.y=74;
		Kick(env,p->ActiveAttacker,pos);
		Kick(env,p->Attacker,pos);
	//Defender		
		pos.x=19;
		pos.y=27;
		for(i=1;i<5;i++){
			if(fabs(p->opp[i].pos.x-pos.x)<15&&fabs(p->opp[i].pos.y-pos.y)<15)
				break;
		}
		if(i<5)
			PositionAndThrough(env,p->Defender,p->opp[i].pos,125);
		else{ 
			pos.x=20;
			pos.y=27;
			PositionAndStop(env,p->Defender,pos,145);
		}
	//NegativeAttacker
		pos.x=19;
		pos.y=58;
		for(i=1;i<5;i++){
			if(fabs(p->opp[i].pos.x-pos.x)<15&&fabs(p->opp[i].pos.y-pos.y)<15)
				break;
		}
		if(i<5)
			PositionAndThrough(env,p->NegativeAttacker,p->opp[i].pos,125);
		else{ 
			pos.x=20;
			pos.y=27;
			PositionAndStop(env,p->NegativeAttacker,pos,-145);
		}
		break;
	case 5: 
	case 6:
		Order(env);
	/*改过7.18
		x=WhoseBall(env);
		if(p->whoseBall==2){
             Sweep(env,p->Attacker);
			 for(i=0;i<5;i++) if(Distance(p->robot[p->Attacker].pos,p->opp[i].pos)<5) break;
			 if(i<5) PositionAndThrough(env,p->ActiveAttacker,p->opp[i].pos);
			 else Kick(env,p->ActiveAttacker,p->Attacker);
		}
		else{
		   	Kick(env,p->ActiveAttacker,1,1.5);
			Kick(env,p->Attacker,3,1.5);
		}
    改过7.18*/
		Kick(env,p->ActiveAttacker,1,1.5);
		Kick(env,p->Attacker,3,1.5);
		pos.x=19;
		pos.y=58;
		PositionAndStop(env,p->Defender,pos,-135);
		/*改过2011.7.17*/
	//NegativeAttacker
        PredictBall(env,10);
		for(i=1;i<5;i++){
			if(fabs(p->opp[i].pos.x-p->ball_pre.x)<15 && fabs(p->opp[i].pos.y-p->ball_pre.y)<15)
				break;
		}
        PositionAndThrough(env,p->NegativeAttacker,pos,135);
		break;
	case 7:
	case 8: 
		Order(env);
	/*改过7.18
		x=WhoseBall(env);
		if(p->whoseBall==2){
             Sweep(env,p->Attacker);
			 for(i=0;i<5;i++) if(Distance(p->robot[p->Attacker].pos,p->opp[i].pos)<5) break;
			 if(i<5) PositionAndThrough(env,p->ActiveAttacker,p->opp[i].pos);
			 else Kick(env,p->ActiveAttacker,p->Attacker);
		}
		else{
		   	Kick(env,p->ActiveAttacker,1,1.5);
			Kick(env,p->Attacker,3,1.5);
		}
    改过7.18*/
		Kick(env,p->ActiveAttacker,1,1.5);
		Kick(env,p->Attacker,3,1.5);
		pos.x=20;
		pos.y=27;
		PositionAndStop(env,p->Defender,pos,135);
		/*改过2011.7.17*/
	//NegativeAttacker
        PredictBall(env,10);
		for(i=1;i<5;i++){
			if(fabs(p->opp[i].pos.x-p->ball_pre.x)<15 && fabs(p->opp[i].pos.y-p->ball_pre.y)<15)
				break;
		}
        PositionAndThrough(env,p->Defender,pos,135);
		break;
	case 9: 
		Order(env);	
		//ActiveAttacker
		shoot(env,p->ActiveAttacker);
		//Attacker
		if(p->ball_cur.x>p->robot[p->ActiveAttacker].pos.x&&p->ball_cur.y<10){
			count=120;
			while(count>0){
				Kick(env,p->Attacker,p->ActiveAttacker);
				count--;	
			}
		}
		else{
			PredictBall(env,2);
			PositionAndStop(env,p->Attacker,p->ball_pre);
			pos.x=77;
			pos.y=57.5;
			PositionAndStop(env,p->NegativeAttacker,pos,-45);
		}
	/*改过*/
        for(i=1;i<5;i++){
			if(i!=p->ActiveAttacker && i!=p->Attacker && i!=p->NegativeAttacker) break;
		}
		PredictBall(env,4);
        pos.x=50;
		pos.y=p->ball_pre.y;
        PositionAndStop(env,i,pos);
		break;
	case 10: 
		Order(env);
		shoot(env,p->ActiveAttacker);
		PredictBall(env,2);
		PositionAndStop(env,p->Attacker,p->ball_pre);
		pos.x=77;
		pos.y=57.5;
		PositionAndStop(env,p->NegativeAttacker,pos,-45);
	/*改过*/
		for(i=1;i<5;i++){
			if(i!=p->ActiveAttacker && i!=p->Attacker && i!=p->NegativeAttacker) break;
		}
		PredictBall(env,4);
        pos.x=50;
		pos.y=p->ball_pre.y;
        PositionAndStop(env,i,pos);
		break;
	case 11: 
		Order(env);
		shoot(env,p->ActiveAttacker);
		PredictBall(env,2);
		PositionAndStop(env,p->Attacker,p->ball_pre);
		pos.x=77;
		pos.y=26;
		PositionAndStop(env,p->NegativeAttacker,pos,-45);
	/*改过*/
		for(i=1;i<5;i++){
			if(i!=p->ActiveAttacker && i!=p->Attacker && i!=p->NegativeAttacker) break;
		}
		PredictBall(env,4);
        pos.x=50;
		pos.y=p->ball_pre.y;
        PositionAndStop(env,i,pos);
		break; 
	case 12:
		Order(env);	
		shoot(env,p->ActiveAttacker);
		if(p->ball_cur.x>p->robot[p->ActiveAttacker].pos.x && p->ball_cur.y>74.2){
			count=120;
			while(count>0){	
				Kick(env,p->Attacker,p->ActiveAttacker);
				count--;
			}	
		}
		else{	
			PredictBall(env,2);
			PositionAndStop(env,p->Attacker,p->ball_pre);
			pos.x=77;
			pos.y=26;
			PositionAndStop(env,p->NegativeAttacker,pos,-45);
		}
	/*改过*/
		for(i=1;i<5;i++){
			if(i!=p->ActiveAttacker && i!=p->Attacker && i!=p->NegativeAttacker) break;
		}
		PredictBall(env,4);
        pos.x=50;
		pos.y=p->ball_pre.y;
        PositionAndStop(env,i,pos);
		break;
	case 13:
		Order(env);	
	//ActiveAttacke
		shoot(env,p->ActiveAttacker);
	//Attacker
		if(p->ball_cur.x<92){
			count=120;
			while(count>0){	
				Kick(env,p->Attacker,p->ActiveAttacker);
				count--;
			}	  
		}
		else
			shoot(env,p->Attacker);
     //NegativeAttacker
		if(p->ball_cur.x<87||p->ball_cur.y<20){  
			count=120;
			while(count>0){
				Kick(env,p->NegativeAttacker,p->Attacker);
				count--;
			}
		}
		else{
			pos.x=77;
			pos.y=57.5;
			PositionAndStop(env,p->NegativeAttacker,pos,-45);
		}
	/*改过*/
		for(i=1;i<5;i++){
			if(i!=p->ActiveAttacker && i!=p->Attacker && i!=p->NegativeAttacker) break;
		}
		PredictBall(env,4);
        pos.x=50;
		pos.y=p->ball_pre.y;
        PositionAndStop(env,i,pos);
		break;
	case 14:
		Order(env);
    //NegativeAttacker
		if(p->ball_cur.y>GBOT){
			PredictBall(env,2);
			begin=p->robot[p->NegativeAttacker].pos;
			end=p->ball_pre;
			alfa=Atan(begin,end);
            PAngle(env,p->NegativeAttacker,alfa,125); 
		}
		else{
			pos.x=77;
			pos.y=57.5;
		    PositionAndStop(env,p->NegativeAttacker,pos,-45);
		}
     //ActiveAttacker
		shoot(env,p->ActiveAttacker);
	 //Attacker
		if(p->ball_cur.y<34){
			count=120;
			while(count>0){	
				Kick(env,p->Attacker,p->ActiveAttacker);
				count--;
			}
		}
		else{
			if(p->ball_speed.y>1){ 
				//PositionBallX(env,p->ActiveAttacker,p->robot[p->ActiveAttacker].pos,-100,1);
                  PositionBallX(env,p->Attacker,p->robot[p->ActiveAttacker].pos,-100,1);
			}
			else{
				//PositionBallX(env,p->ActiveAttacker,p->robot[p->ActiveAttacker].pos,-70,1);
				PositionBallX(env,p->Attacker,p->robot[p->ActiveAttacker].pos,-70,1);

			}
			Kick(env,p->Attacker,CONSTGATE);
		}
		/*改过*/
	 //"Defender"
		for(i=1;i<5;i++){
			if(i!=p->ActiveAttacker && i!=p->Attacker && i!=p->NegativeAttacker) break;
		}
		PredictBall(env,4);
        pos.x=50;
		pos.y=p->ball_pre.y;
        PositionAndStop(env,i,pos);
		break;
	case 15:
		Order(env);
		if(p->ball_cur.y<GTOP){
			PredictBall(env,2);
		    begin=p->robot[p->NegativeAttacker].pos,end=p->ball_pre;
		    alfa=Atan(begin,end);
            PAngle(env,p->NegativeAttacker,alfa,125);
		} 
		else{
            pos.x=77;pos.y=26;
		    PositionAndStop(env,p->NegativeAttacker,pos,-45);
		}
		shoot(env,p->ActiveAttacker);
		if(p->ball_cur.y>48){
			count=120;
			while(count>0){	
				Kick(env,p->Attacker,p->ActiveAttacker);
				count--;
			}
		}
		else{
			if(p->ball_speed.y>1){
				//PositionBallX(env,p->ActiveAttacker,p->robot[p->ActiveAttacker].pos,100,1);
				PositionBallX(env,p->Attacker,p->robot[p->ActiveAttacker].pos,100,1);
			}
			else{
				//PositionBallX(env,p->ActiveAttacker,p->robot[p->ActiveAttacker].pos,70,1);
				PositionBallX(env,p->Attacker,p->robot[p->ActiveAttacker].pos,70,1);
			}
			Kick(env,p->Attacker,CONSTGATE);
		}
	/*改过*/
		for(i=1;i<5;i++){
			if(i!=p->ActiveAttacker && i!=p->Attacker && i!=p->NegativeAttacker) break;
		}
		PredictBall(env,4);
        pos.x=50;
		pos.y=p->ball_pre.y;
        PositionAndStop(env,i,pos);
		break;
	case 16:
		Order(env);	
		shoot(env,p->ActiveAttacker);
		if(p->ball_cur.x<92){
			count=120;
			while(count>0){
				Kick(env,p->Attacker,p->ActiveAttacker);
				count--;
			}
		}
		else
			shoot(env,p->Attacker);

		if(p->ball_cur.x<87||p->ball_cur.y>66){
            count=100;
			while(count>0){
				Kick(env,p->NegativeAttacker,p->Attacker);
				count--;
			}
		}
		else{
			pos.x=77;pos.y=26;
			PositionAndStop(env,p->NegativeAttacker,pos,45);
		}
		/*改过*/
		for(i=1;i<5;i++){
			if(i!=p->ActiveAttacker && i!=p->Attacker && i!=p->NegativeAttacker) break;
		}
		PredictBall(env,4);
        pos.x=50;
		pos.y=p->ball_pre.y;
        PositionAndStop(env,i,pos);
		break;
	}
}
void GAMESTATE(Environment *env){
        Mydata * p;
	    p=(Mydata *)env->userData;
		int i,j,k=0;
		p->gameState= 0;
        Vector3D pos;
/*判断是否点球,任意球*/  
		pos.x=78.6621;
		pos.y=41.8060;
		for(i=1;i<4;i++)   if(p->robot[i].pos.x >50.1189)   break;
		for(j=1;j<5;j++)   if(p->robot[j].pos.x> 50.1189) break;
		if(Distance(pos,p->ball_cur)<2 )	{
			if(i>=4 && j>=5){
				p->gameState= PENALTY_KICK; 
				return;
			}
			else{
			    p->gameState= FREE_KICK;
				return;
			}
		}

/*判断是否门球*/  
		for(j=0;j<5;j++)  if(p->opp[j].pos.x<50.1 )     break;
		if(j>=5 && p->ball_cur.x<BRG_LEFT && p->ball_cur.y<BRG_TOP && p->ball_cur.y>BRG_BOT) {p->gameState= GOAL_KICK; return;}

/*判断是否开球*/
        pos.x=50.1189;
		pos.y=41.8061;
		for(i=1;i<5;i++)	if(p->robot[i].pos.x> 50.1){
			if(k==0){k=1;continue;}
			else break;
		}
		for(j=0;j<5;j++)    if(p->opp[j].pos.x<50.1 )     break;
        if(Distance(pos,p->ball_cur)<19 && i>=5 && j>=5)  {
			if(p->robot[3].pos.x<50.1189) {p->gameState =0; return;}
			else {p->gameState = PLACE_KICK; return;}
		}

/*判断是否争球*/
        
}

void FreeBallGame(Environment *env){//争球
    Mydata * p;
	p=(Mydata *)env->userData;

}
void PlaceBallGame(Environment *env){//开中场球
    Mydata * p;
	p=(Mydata *)env->userData;
	Vector3D pos;
	pos.x=(PG_LEFT+PG_RIGHT)/2-10;
	pos.y=41.8060;
	PredictBall(env,10);
    PositionAndStop(env,3,pos);
	Kick(env,4,CONSTGATE);
	PositionAndThrough(env,1,p->ball_pre);
	Keeper(env,0);
}

void PenaltyBallGame(Environment *env){//点球
    Mydata * p;
	p=(Mydata *)env->userData;
	shoot(env,4);
}
void FreeKickGame(Environment *env){//任意球
	 Mydata * p;
	 p=(Mydata *)env->userData;
	 shoot(env,4);

}
void GoalKickGame(Environment *env){//门球
     Mydata * p;
	 p=(Mydata *)env->userData;
     shoot(env,0);
}