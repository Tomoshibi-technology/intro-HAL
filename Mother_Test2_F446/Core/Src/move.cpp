
#include "move.h"

MOVE::MOVE(uint8_t motor_id, int16_t motor_degree){
    MOTOR_ID = motor_id;
    MOTOR_DEGREE = motor_degree;
}

void MOVE::calcurate(int16_t gyro_degree, int16_t goal_degree, int16_t goal_speed){
    int roll_speed;
    if(gyro_degree > 180){gyro_degree -= 360;}

    if (gyro_degree > 0){
        roll_speed = -10 + (-gyro_degree * 2);
        if (gyro_degree < 6){
            roll_speed = 0;
        }
        if (roll_speed < -150){
            roll_speed = -150;
        }
    }else if (gyro_degree < 0){
        roll_speed = 10 + (-gyro_degree * 2);
        if (gyro_degree > -6){
            roll_speed = 0;
        }
        if (roll_speed > 150){
            roll_speed = 150;
        }
    }else{
        roll_speed = 0;
    }

    motor_speed = (int)goal_speed*sin((PI/180)*(goal_degree - MOTOR_DEGREE));
    motor_speed = (int)(motor_speed * motor_rate) + (roll_speed * (1 - motor_rate));

}

void MOVE::set_array(uint8_t *send_array){
    send_array[MOTOR_ID*3]=250+MOTOR_ID;
    int16_t send_speed = motor_speed;
    send_speed += 1000;
	for(int i = 1; i <3 ;i++){
		send_array[MOTOR_ID*3+i] = send_speed%100;
		send_speed = (int)send_speed/100;
	}
}

