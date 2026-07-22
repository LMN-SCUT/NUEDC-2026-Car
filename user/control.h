#ifndef __CONTROL_H
#define __CONTROL_H

float Track_err(void);

int PID_out(float error,int Target);//poistion_pid:入口参数：1.循迹模块返回的误差;2.目标误差，也就是0，让小车在线的中间
int PID_angle(int target,int yaw);
void Final_Speed(int pid_out ,int base_speed);

#endif
