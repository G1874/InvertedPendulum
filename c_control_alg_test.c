#include <stdio.h>
#include <stdint.h>

const int32_t LQR_K_MATRIX[4] = {-248761,-1579388,
                                -4926922,-1099123}; // Times SCALE_FACTOR

/* Scale factor used for fixed point calculations */
#define SCALE_FACTOR 10000

/* Coefficients derived from physical parameters */
#define ACC_TO_SPD 25000 // Times SCALE_FACTOR
#define RADIUS_FACTOR 77

/* Coefficients used for sensor reading rescaling */
#define DISTANCE_NORMALIZATION 20

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

void ControlAlg(int32_t distance, int32_t angle, int32_t* p_angle, int32_t* speed)
{
	int64_t u = 0;
	int64_t x[4];
	x[0] = distance;                                                   // distance in meters times SCALE_FACTOR
	x[1] = *speed;                                                     // previous speed
	x[2] = angle;                                                      // angle in radians times SCALE_FACTOR
	x[3] = (angle - *p_angle) * 1000 / TP;                             // angular speed in radians/second times SCALE_FACTOR

	for(int i=0;i<4;i++) {
	    u += -LQR_K_MATRIX[i] * x[i] / SCALE_FACTOR;
	}

    char buffer[64];
    sprintf(buffer,"%lld\n",u);
    printf(buffer);

	*speed = *speed + u * ACC_TO_SPD * TP / 1000 / SCALE_FACTOR;
	*p_angle = angle;
}

void HCSR04_GetDistance(uint16_t* RechoTime, uint16_t* FechoTime, int32_t* distance)
{
	*distance = ((*FechoTime - *RechoTime) / 58 - DISTANCE_NORMALIZATION) * (SCALE_FACTOR / 100);
}

void AngleRescalingFlt(uint16_t raw_angle, double* angle_deg, double* angle)
{
    *angle_deg = (360.0 / (AS5600_RESOLUTION - 1) * raw_angle) - 180;
    *angle = *angle_deg * 3.1415 / 180;
}

void AngleRescaling(uint16_t raw_angle, int32_t* angle_deg, int32_t* angle)
{
    *angle_deg = (36 * (raw_angle * 10 * SCALE_FACTOR / (AS5600_RESOLUTION - 1))) - 180 * SCALE_FACTOR;
    *angle = *angle_deg / 180 * 31415 / SCALE_FACTOR;
}

void StepperNewPWM(int32_t speed, uint8_t* direction, uint16_t* period)
{
	int64_t p;
    p = 2 * 31415 * 130 / speed * 100 / STEPS_PER_REVOLUTION;

    *period = p;

    char buffer[64];
    sprintf(buffer,"%lld\n",*period);
    printf(buffer);
}

void StepperNewPWMFlt(double speed, double* period)
{
	double p;
	p = 2 * 3.1415 * 0.013 / speed / STEPS_PER_REVOLUTION;

    char buffer[64];
    sprintf(buffer,"%f\n",p);
    printf(buffer);
}

int main()
{
    uint16_t raw_angle = 2100;
    int32_t distance = -500;
    int32_t speed = 0;
    int32_t d_angle;
    int32_t angle;
    int32_t p_angle = 1000;

    AngleRescaling(raw_angle, &d_angle, &angle);
    ControlAlg(distance,angle,&p_angle,&speed);
    
    return 0;
}