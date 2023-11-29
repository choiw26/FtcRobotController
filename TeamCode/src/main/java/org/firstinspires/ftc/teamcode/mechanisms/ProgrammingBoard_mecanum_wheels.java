package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ProgrammingBoard_mecanum_wheels {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor viperslide;

    private double ticksPerRotation;


    public void init(HardwareMap hwmap) {
        leftFront = hwmap.get(DcMotor.class, "frontLeftMotor");
        rightFront = hwmap.get(DcMotor.class, "backLeftMotor");
        leftBack = hwmap.get(DcMotor.class, "frontRightMotor");
        rightBack = hwmap.get(DcMotor.class, "backRightMotor");


        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        viperslide = hwmap.get(DcMotor.class, "viperslide");
        viperslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ticksPerRotation = viperslide.getMotorType().getTicksPerRev();

    }

    public void leftFrontSpeed (double speed) {
        leftFront.setPower(speed);
    }
    public void rightFrontSpeed (double speed) {
        rightFront.setPower(speed);
    }
    public void leftBackSpeed (double speed) {
        leftBack.setPower(speed);
    }
    public void rightBackSpeed (double speed) {
        rightBack.setPower(speed);
    }

    public void viperslideSpeed (double speed) {
        viperslide.setPower(speed);
    }
    public double getViperRotation () {
        return (double) viperslide.getCurrentPosition() / ticksPerRotation;
    }

}
