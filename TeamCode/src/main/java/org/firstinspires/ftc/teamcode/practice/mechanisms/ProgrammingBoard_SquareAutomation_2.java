package org.firstinspires.ftc.teamcode.practice.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//For Distance sensor


public class ProgrammingBoard_SquareAutomation_2 {
    private DcMotor[] motors = new DcMotor[4];


    private DistanceSensor distanceSensor1;
    private DistanceSensor distanceSensor2;

    public void init(HardwareMap hwmap) {
        motors[0] = hwmap.get(DcMotor.class, "frontLeftMotor"); // frontLeftMotor
        motors[1] = hwmap.get(DcMotor.class, "backLeftMotor"); // backLeftMotor
        motors[2] = hwmap.get(DcMotor.class, "frontRightMotor"); //frontRightMotor
        motors[3] = hwmap.get(DcMotor.class, "backRightMotor"); //BackRightMotor

        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        // The robot motors are put weirdly. The left ones are inverted instead of the right one.s
        //Therefore inverted the left ones.

        motors[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[2].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[3].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        distanceSensor1 = hwmap.get(DistanceSensor.class, "sensorDistance1");
        distanceSensor2 = hwmap.get(DistanceSensor.class, "sensorDistance2");

    }
    public void setAllMotor (double speed) {
        for (DcMotor motor : motors) {
            motor.setPower(speed);
        }
    }

    public void setPower(double frontLeftPower, double frontRightPower, double backLeftPower
            , double backRightPower) {
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double maxSpeed = 1.0;
        maxSpeed = Math.max(maxSpeed, Math.abs(frontLeftPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(frontRightPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(backLeftPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(backRightPower));

        frontLeftPower /= maxSpeed;
        frontRightPower /= maxSpeed;
        backLeftPower /= maxSpeed;
        backRightPower /= maxSpeed;

        motors[0].setPower(frontLeftPower);
        motors[1].setPower(backLeftPower);
        motors[2].setPower(frontRightPower);
        motors[3].setPower(backRightPower);
    }
    public void drive(double forward, double right, double rotate) {

        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backLeftPower = forward - right + rotate;
        double backRightPower = forward + right - rotate;

        setPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    public double getDistance1(DistanceUnit cm) {
        return distanceSensor1.getDistance(cm);
    }
    public double getDistance2(DistanceUnit cm) {
        return distanceSensor2.getDistance(cm);
    }
}
