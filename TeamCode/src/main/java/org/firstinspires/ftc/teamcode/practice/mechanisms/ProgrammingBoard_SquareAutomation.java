package org.firstinspires.ftc.teamcode.practice.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
// DcMotor for the movement
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//Just for Lols
import com.qualcomm.robotcore.hardware.HardwareMap;
//HardwareMap for connecting the configuration between Robot and code
import com.qualcomm.robotcore.hardware.DistanceSensor;
//Distance sensor to measure distance between robot
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//For IMU
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//For Distance sensor


public class ProgrammingBoard_SquareAutomation {
    private DcMotor[] motors = new DcMotor[4];

    private DistanceSensor distanceSensor;

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

        distanceSensor = hwmap.get(DistanceSensor.class, "sensorDistance");

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

    public double getDistance(DistanceUnit cm) {
        return distanceSensor.getDistance(cm);
    }
}
