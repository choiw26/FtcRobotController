package org.firstinspires.ftc.teamcode.practice.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class ProgrammingBoard_mecanum_wheels {
    private DcMotor[] motors = new DcMotor[4];

    private DcMotor tiltMotor;

    private DcMotor viperslide;

    private DistanceSensor distanceSensor;

    private double ticksPerRotationTilt;
    private double ticksPerRotationViper;


    public void init(HardwareMap hwmap) {
        motors[0] = hwmap.get(DcMotor.class, "frontLeftMotor"); // frontLeftMotor
        motors[1] = hwmap.get(DcMotor.class, "backLeftMotor"); // backLeftMotor
        motors[2] = hwmap.get(DcMotor.class, "frontRightMotor"); //frontRightMotor
        motors[3] = hwmap.get(DcMotor.class, "backRightMotor"); //BackRightMotor



        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);

        motors[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[2].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[3].setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        tiltMotor = hwmap.get(DcMotor.class, "test motor");

        tiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tiltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        viperslide = hwmap.get(DcMotor.class, "viperSlide");

        viperslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ticksPerRotationViper = viperslide.getMotorType().getTicksPerRev();
        ticksPerRotationTilt = tiltMotor.getMotorType().getTicksPerRev();



        //distanceSensor = hwmap.get(DistanceSensor.class, "sensor_color_distance");

    }

    public void changeDirectionRight () {

    }

    public void changeDirectionLeft () {

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
        motors[1].setPower(frontRightPower);
        motors[2].setPower(backLeftPower);
        motors[3].setPower(backRightPower);
    }
    public void drive(double forward, double right, double rotate) {

        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backLeftPower = forward - right + rotate;
        double backRightPower = forward + right - rotate;
        setPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }



    public void setTiltMotor(double speed) {
        tiltMotor.setPower(speed);
    }

    public void viperSlideSpeed(double speed) {
        viperslide.setPower(speed);
    }


    public double getViperRotation() {
        return viperslide.getCurrentPosition() / ticksPerRotationViper;
    }
    public double getTiltRotation() {
        return tiltMotor.getCurrentPosition() / ticksPerRotationTilt;
    }
    /*public double getDistance(DistanceUnit cm) {
        return distanceSensor.getDistance(cm);
    }*/

}
