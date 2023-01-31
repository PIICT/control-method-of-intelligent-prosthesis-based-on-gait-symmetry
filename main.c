//
//  main.c
//  control
//
//  Created by gjy on 2022/4/18.
//
#include <stdlib.h>
#include "uart.h"
#include "input_signal.h"
#include "sensor.h"
#include "stdio.h"
#include "main.h"
#include "motor.h"

#define DELTA_T        0.01
#define MAX_STEP       10

float    vj1[3]={0.4436,-0.8818,-0.1602}, vj2[3]={0.8889,-0.4509,-0.0812};
float    imu_raw_data_online1[3];
float    imu_raw_data_online2[3];
float    vj1_times_g1,vj2_times_g2;
float    sum=0;
float    angle_gyr=0.0,pre_angle_gyr=0.0,pro_angle,pre_pro_angle=0.0;//需要给prothetic_angle赋值，假肢侧的角度变化
float    knee_angle;
float    prosthetic_angle,diff_angle;

int      test_angle_i=0;
float    electric_factor=1.0;

int      isPress;//isPress 识别是否支撑，2000是无压力，1000是有压力
int      isProstheticBegin=0,isBegin=0,pre_isProstheticBegin=0,pre_isBegin=0;//分别代表假肢是否开始走路，以及健侧腿是否开始走路，0代表未开始，1代表开始
float    online_pro_angle,near_pro_angle[MAX_STEP],near_leg_angle[MAX_STEP],online_leg_begin,online_leg_angle;
int      near_pro_len=0,near_leg_len=0;
float    max_angle,min_angle;

extern TAllSensorsType SensorsAll;
extern TSignal_Input signal_input;
extern int mode_detect;
extern int control_electric_factor;
int flag_gao=0;


void test_angle()
{
/////赋值////////////////////////////////////////////////////
pro_angle=signal_input.Knee_angle;
imu_raw_data_online1[0]=(float)SensorsAll.IMU[9].Gyro_X;
imu_raw_data_online1[1]=(float)SensorsAll.IMU[9].Gyro_Y;
imu_raw_data_online1[2]=(float)SensorsAll.IMU[9].Gyro_Z;
imu_raw_data_online2[0]=(float)SensorsAll.IMU[10].Gyro_X;
imu_raw_data_online2[1]=(float)SensorsAll.IMU[10].Gyro_Y;
imu_raw_data_online2[2]=(float)SensorsAll.IMU[10].Gyro_Z;
isPress=mode_detect;
////////////////////////////////////////////////////////////
    vj1_times_g1=imu_raw_data_online1[0]*vj1[0]+imu_raw_data_online1[1]*vj1[1]+imu_raw_data_online1[2]*vj1[2];
    vj2_times_g2=imu_raw_data_online2[0]*vj2[0]+imu_raw_data_online2[1]*vj2[1]+imu_raw_data_online2[2]*vj2[2];
    
    sum=sum+vj1_times_g1-vj2_times_g2;
    
    angle_gyr    = sum * DELTA_T;
    angle_gyr    = angle_gyr * 180.0 / 3.1415926;

    if ((isProstheticBegin==0)&&(pro_angle>10)){
        //假肢开始运动了
        isProstheticBegin=1;
        online_pro_angle=pro_angle;
        if(pro_angle>70){
            online_pro_angle=pre_pro_angle;
        }

    }
    if ((isProstheticBegin==1)&&(pro_angle<10)){
        //假肢停止运动
        isProstheticBegin=0;
    }
    if ((isProstheticBegin==1)&&(pro_angle>20)){
        //记录假肢摆动期最大夹角
        if(pro_angle>online_pro_angle){
            online_pro_angle=pro_angle;
        }
    }

    if ((isProstheticBegin==0)&&(pre_isProstheticBegin==1)){
        //假肢刚刚停止摆动，记录上个假肢摆动期的最大角度
        if(near_pro_len==MAX_STEP){
            for(test_angle_i=0;test_angle_i<MAX_STEP-1;test_angle_i++){
                near_pro_angle[test_angle_i]=near_pro_angle[test_angle_i+1];
            }
            near_pro_len=near_pro_len-1;
        }
        near_pro_angle[near_pro_len]=online_pro_angle;
        near_pro_len=near_pro_len+1;
    }

    //isPress 识别是否支撑，2000是无压力，1000是有压力
    if ((pro_angle>10)&&(pre_pro_angle<10)){
        online_leg_angle=max_angle-min_angle;

        //健侧腿刚刚停止摆动，记录上个健侧腿摆动期的最大角度
        if(near_leg_len==MAX_STEP){
            for(test_angle_i=0;test_angle_i<MAX_STEP-1;test_angle_i++){
                near_leg_angle[test_angle_i]=near_leg_angle[test_angle_i+1];
            }
            near_leg_len=near_leg_len-1;
        }
        near_leg_angle[near_leg_len]=online_leg_angle;
        near_leg_len=near_leg_len+1;
                    if ((near_leg_len>0)&&(near_pro_len>0)&&(control_electric_factor==1)){
        //两个腿均有步态周期
        knee_angle=0.0;
        prosthetic_angle=0.0;
        for (test_angle_i=0;test_angle_i<near_leg_len;test_angle_i++){
            knee_angle=knee_angle+near_leg_angle[test_angle_i];
        }
        for (test_angle_i=0;test_angle_i<near_pro_len;test_angle_i++){
            prosthetic_angle=prosthetic_angle+near_pro_angle[test_angle_i];
        }
        
        //knee_angle 和prosthetic_angle是MAX_STEP个步态周期里的平均值
        knee_angle=knee_angle/near_leg_len;
        prosthetic_angle=prosthetic_angle/near_pro_len;

        diff_angle=knee_angle- prosthetic_angle;

        //积分

        if ((5>diff_angle)&&(diff_angle >2)) {
            electric_factor=electric_factor+1;
        }
        if (diff_angle>5) {
            electric_factor=electric_factor+3;
        }
        if ((diff_angle<-2)&&(diff_angle>-5)) {
            electric_factor=electric_factor-1;
        }
        if (diff_angle<-5) {
            electric_factor=electric_factor-3;
        }

        if (electric_factor<0.5){
            electric_factor=0.5;
        }
        if (electric_factor>8){
            electric_factor=8;
        }

        
        }
        online_leg_begin=angle_gyr;
        max_angle=angle_gyr;
        min_angle=angle_gyr;
    }

    if(angle_gyr>max_angle){
        max_angle=angle_gyr;
        }
    if(angle_gyr<min_angle){
        min_angle=angle_gyr;
    }
        
    pre_isBegin=isBegin;
    pre_isProstheticBegin=isProstheticBegin;
    pre_pro_angle=pro_angle;
}
