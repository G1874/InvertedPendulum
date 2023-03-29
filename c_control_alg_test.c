#include <stdio.h>
#include <stdint.h>

const int32_t LQR_K_MATRIX[4] = {-31623,-24031,
                                -952478,-206440}; // Times SCALE_FACTOR

/* Scale factor used for fixed point calculations */
#define SCALE_FACTOR 10000

/* Coefficients derived from physical parameters */
#define ACC_TO_SPD 2500 // Times SCALE_FACTOR
#define RADIUS_FACTOR 77

/* Coefficients used for sensor reading rescaling */
#define DISTANCE_NORMALIZATION 20
#define ANGLE_NORMALIZATION 20

/* Sample time in ms */
#define TP 20

/* Select electronic hardware parameters */
#define MAX_SPEED 100000
#define STEPS_PER_REVOLUTION 200
#define AS5600_RESOLUTION 4096

double ControlAlgFlt(double* x,double* K,double* r)
{
    /****
    u = -K*(x-r)
                              [x1]   [r1]
                              [x2]   [r2]
    u = [-k1 -k2 -k3 -k4] * ( [x3] - [r3] )
                              [x4]   [r4]

    u = -k1(x1-r1) -k2(x2-r2) -k3(x3-r3) -k4(x4-r4)
    ****/
    double u = 0;
    for(int i=0;i<4;i++) {
        u += -K[i]*(x[i]-r[i]);
    }

    return u;
}

void ControlAlg(uint16_t distance, int32_t angle, int32_t* p_angle, int32_t* speed, uint8_t mode)
{
	if(mode == 1) {
        char buffer[64];
		int64_t u = 0;
		int64_t x[4];
		x[0] = (distance - DISTANCE_NORMALIZATION) * (SCALE_FACTOR / 100); // distance in meters times SCALE_FACTOR
		x[1] = *speed;                                                     // previous speed
		x[2] = angle;                                                      // angle in radians times SCALE_FACTOR
		x[3] = (angle - *p_angle) * 1000 / TP;                             // angular speed in radians/second times SCALE_FACTOR

		for(int i=0;i<4;i++) {
		    u += -LQR_K_MATRIX[i] * x[i] / SCALE_FACTOR;
		}

        sprintf(buffer,"%lld\n",u);
        printf(buffer);
        /* sprintf(buffer,"%lld\n",x[0]);
        printf(buffer);
        sprintf(buffer,"%lld\n",x[1]);
        printf(buffer);
        sprintf(buffer,"%lld\n",x[2]);
        printf(buffer);
        sprintf(buffer,"%lld\n",x[3]);
        printf(buffer); */
        
        int64_t a = u * ACC_TO_SPD * TP /1000;

        sprintf(buffer,"%lld\n",a);
        printf(buffer);

		*speed = *speed + u;
		*p_angle = x[2];
	} else {
		*p_angle = (angle - ANGLE_NORMALIZATION) * 31415 / 180;
	}
}

int main()
{
    int32_t speed = 0;
    uint16_t distance = 15;
    int32_t angle = 6981;
    int32_t p_angle = 8726;
    uint8_t mode = 1;

    ControlAlg(distance,angle,&p_angle,&speed,mode);

    /* char buffer[64];
    sprintf(buffer,"%d\n",speed);
    printf(buffer); */
    
    return 0;
}