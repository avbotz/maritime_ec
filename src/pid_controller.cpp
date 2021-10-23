/*
 * pid_controller.cpp
 *
 * Implements a PID controller, in "standard" form
 * that is, C(s) = Kp(1 + 1/sTi + sTd)
 *
 * @author Kalyan Sriram
 */

#include <mec/pid_controller.h>

void pid_set_gains(struct pid_controller *pid, float kp, float ti, float td)
{
	pid->kp = kp;
	pid->ti = ti;
	pid->td = td;
}

void pid_reset(struct pid_controller *pid)
{
	pid->prev_error = 0;
	pid->integral = 0;
}

/*
 * @param error the current error
 * @param dt amount of time passed (in seconds) since last calculation
 * @return output of the controller
 */
float pid_calculate(struct pid_controller *pid, float error, float dt)
{
	/* u(t) = Kp(e(t) + (1/Ti)*int e(t)dt + Td*de(t)/dt */

	/* calculate integral term */
	pid->integral += error * dt;
	float integral_term = (1.0f / pid->ti) * pid->integral;

	/* calculate the derivative term */
	float derivative_term = pid->td * ((error - pid->prev_error) / dt);
	pid->prev_error = error;

	/* add proportional term and scale entire output */
	float output = pid->kp * (error + integral_term - derivative_term);
}
