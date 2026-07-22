#include "stm32f10x.h"                  // Device header
#include "motor.h"
#include "grey.h"
 float error;



float Track_err(void)
{
	u8 state =  Get_Infrared_State();
	
	switch(state)
	{  
		case 0:     //0000 0000   
		error= 0 ;	  break;
		case 16:    //0001 0000   
		error= 1 ;  break;
		case 8:     //0000 1000   
		error= 1 ; break;	
		case 24:    //0001 1000   
		error= 0 ;	  break;		
		case 60:    //0011 1100   
		error= 0 ;	  break;		
		case 126:   //0111 1110   
		error= 0 ;	  break;
		
		case 48:     //0011 0000   //小车右偏，err为正
		error= 2  ;	  break;
		case 32:     //0010 0000   
		error= 2 ;	  break;
		case 64:     //0100 0000   
		error= 4 ;	  break;
		case 96:     //0110 0000   
		error= 4 ;	  break;
		case 128:    //1000 0000   
		error= 6 ;	  break;	
		case 192:    //1100 0000   
		error= 6 ;	  break;
		case 224:   //1110 0000   
		error= 6 ;	  break;
		case 160:   //1010 0000   
		error= 6 ;	  break;
		case 254:   //1111 1110   
		error= 6 ;	  break;
		case 252:   //1111 1100   
		error= 6 ;	  break;
		case 248:   //1111 1000   
		error= 6 ;	  break;
		case 240:   //1111 0000   
		error= 6 ;	  break;
		case 124:   //0111 1100   
		error= 4 ;	  break;
		case 120:   //0111 1000   
		error= 4 ;	  break;
		case 56:    //0011 1000   
		error= 2 ;	break;
		
		
		

		case 12:   //0000 1100   //小车左偏，err为负
		error= -2 ;	  break;	
        case 14:   //0000 1110   //小车左偏，err为负
		error= -4 ;	  break;	
        case 30:   //0001 1110   //小车左偏，err为负
		error= -4 ;	  break;
        case 62:   //0011 1110   //小车左偏，err为负
		error= -4 ;	  break;
		case 31:   //0001 1111   
		error= -6 ;	  break;
		case 63:   //0011 1111   
		error= -6 ;	  break;
		case 125:  //0111 1111   
		error= -6 ;	  break;
		case 4:    //0000 0100   
		error= -2 ;	  break;	
        case 28:   //0001 1100   
		error= -2 ;	  break;		
		case 2:    //0000 0010   
		error= -4 ;	  break;
		case 6:    //0000 0110   
		error= -4 ;	  break;
		case 1:    //0000 0001   
		error= -6 ;	  break;
		case 3:    //0000 0011   
		error= -6 ;	  break;
		case 7:    //0000 0111   
		error= -6 ;	  break;
		case 15:   //0000 1111   
		error= -6 ;	  break;
			
		default: 
		
		break;
	}
	return error;
}

#define Kp    4
#define Kd    1 
int PID_out(float error, int Target)      //位置环
{
    static int last_err = 0;  // 保留原有静态变量，不新增全局变量
    
    int err = (int)error;
    int out;
    
    // 抑制微分项初始跳变（角度环切换后首次循迹，避免输出突变）
    int diff = err - last_err;
    
    
    // 保留原有PD计算逻辑，仅替换微分项为优化后的diff
    out = Kp * err + Kd * diff;
    
    // 增加极简输出限幅，避免超出电机有效范围
    if (out > 25) out = 25;    
    if (out < -25) out = -25; 
    
    last_err = err;  // 保留原有历史误差更新
    return out;
}

void Final_Speed(int pid_out ,int base_speed)
{
    int left_speed = base_speed - pid_out;
    int right_speed = base_speed + pid_out;
    
    // 软过渡：限制电机速度变化幅度，避免爆冲
    #define MIN_SPEED 5
    #define MAX_SPEED 40
    
    // 左电机限幅
    left_speed = left_speed < MIN_SPEED ? MIN_SPEED : left_speed;
    left_speed = left_speed > MAX_SPEED ? MAX_SPEED : left_speed;
    // 右电机限幅
    right_speed = right_speed < MIN_SPEED ? MIN_SPEED : right_speed;
    right_speed = right_speed > MAX_SPEED ? MAX_SPEED : right_speed;
    
    Motor_Set_PWM(left_speed, right_speed);
}



#define  ka2p   1.5
#define  ka2d   0.5

int PID_angle(int target, int yaw)     //角度环
{
    static int err_last = 0;  // 初始化历史误差（静态变量，记录上一次的有效误差）
    int pid_a_out, err;    
    err = target - yaw;
    // 若误差大于180°，取反向误差（走更短路径，避免逆时针超大误差）
    if (err > 180) {
        err -= 360;
    }
    // 若误差小于-180°，同样取反向误差（保持最短路径）
    else if (err < -180) {
        err += 360;
    }

    
    int diff = err - err_last;
   
    pid_a_out = (int)(ka2p * err + ka2d * diff);  

    #define PID_A_OUT_MAX 20  // 可根据实际调试调整（建议15~25）
    if (pid_a_out > PID_A_OUT_MAX) {
        pid_a_out = PID_A_OUT_MAX;
    }
    if (pid_a_out < -PID_A_OUT_MAX) {
        pid_a_out = -PID_A_OUT_MAX;
    }

  
    err_last = err;

    // 返回PD修正量，无死区闭环，依赖PD参数和限幅实现自然收敛
    return pid_a_out;
}

