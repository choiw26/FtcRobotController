package org.firstinspires.ftc.teamcode.CompetitionCode.Mechanisms;

public class PIDController {

    /**
     * construct PID controller
     * @param Kp Proportional coefficient
     * @param Ki Integral coefficient
     * @param Kd Derivative coefficient
     */
    double Kp;
    double Ki;
    double Kd;

    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Kp;
        this.Kd = Kp;
    }

    /**
     * update the PID controller output
     * @param target where we would like to be, also called the reference
     * @param state where we currently are, I.E. motor position
     * @return the command to our motor, I.E. motor power
     */
    public double update(double target, double state) {
        // PID logic and then return the output
        double error = target - state;
        return 0;
    }


}
