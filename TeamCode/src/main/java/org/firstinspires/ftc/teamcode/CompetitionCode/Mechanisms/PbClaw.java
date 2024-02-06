// Code created like like a month ago
//Code finished on 3rd of January
//This is a code that is imported by the Main Opmode code
//I distributed the codes in multiple documents so the main OpMode doesn't get too complicated

// Imported to AutonomousOpMode.java
// Only for Autonomous

// Copied Rain's Code

package org.firstinspires.ftc.teamcode.CompetitionCode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Config
public class PbClaw {

    //TODO CLAW CONSTANTS
    public static double OpenClawPosition = 0.4; //Claw Open
    public static double closeclawposition = 0.54; //Claw close
    public static double RotateGrabPosition = 0.299999999; // Rotate grabbing position
    public static double rotateplaceposition = 0.5; // Rotate placing position

    public double ServoPositionClaw;
    public double ServoPositionRotate;
    //

    public static double ActualClawSpeedIncrement = 0.007;
    public static double ActualRotateSpeedIncrement = 0.007;

    public double clawSpeedIncrement = 0;
    public double rotateSpeedIncrement = 0;

    //CLAW DEFINE
    public Servo ClawServo;  // LEFT
    public Servo RotateServo; //RIGHT
    //

    //TODO Viperslide
    //Type of Motor : 5301-2402-0019
    //312 RPM, cpr 537.7
    public DcMotorEx viperSlideLeft;
    public DcMotorEx viperSlideRight;
    boolean exit = false;

    public double cprviperSlide = 537.7; //ticks per rotation for viper slide motor;
    public double distancePerRotation = 12; //centimeters per rotation

    public double distancePerTick = distancePerRotation/cprviperSlide;
    public double maxRotation = 8.1; //Rotation

    //
    //


    public void init(HardwareMap hwmap) {
        ClawServo = hwmap.get(Servo.class, "left servo");
        RotateServo = hwmap.get(Servo.class, "right servo");
        ClawServo.setPosition(OpenClawPosition);
        RotateServo.setPosition(RotateGrabPosition);

        //

        viperSlideLeft = hwmap.get(DcMotorEx.class, "viperSlideLeft");
        viperSlideLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        viperSlideLeft.setDirection(DcMotorEx.Direction.REVERSE);
        viperSlideLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        viperSlideRight = hwmap.get(DcMotorEx.class, "viperSlideRight");
        viperSlideRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        viperSlideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Sets the starting position of the ViperSlide to the Zero position
        //
    }


    // Methods for Automation
    // If method is called, the claw is assigned a value "increment" which
    // Increases or Decreases the Position by certain value (0.007) every loop
    // The method ends when the value is over 1.0 or under 0.0
    public void ClawOpen() {
        ClawServo.setPosition(OpenClawPosition);
    }

    public void ClawClose() {
        ClawServo.setPosition(closeclawposition);
    }

    public void RotateGrab() {
        RotateServo.setPosition(RotateGrabPosition);
        return;
    }

    public void RotatePlace() {
        RotateServo.setPosition(rotateplaceposition);
        return;
    }

    // Methods to get the Position of each Servos
    public double getClawServoPosition() {
        return ClawServo.getPosition();
    }

    public double getRotateServoPosition() {
        return RotateServo.getPosition();
    }

    //
    //Viper Slide
    //

    public void ViperSlideSpeed(double speed) {
        viperSlideRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        viperSlideLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        viperSlideLeft.setPower(speed);
        viperSlideRight.setPower(speed);
    }

    public void ViperSlideSetPosition(double centimeters, double speed) {
        viperSlideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        viperSlideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //
        int goal = (int)(Math.round (centimeters / distancePerRotation * cprviperSlide));
        //
        viperSlideLeft.setTargetPosition(goal);
        viperSlideRight.setTargetPosition(goal);
        viperSlideLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        viperSlideRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //
        viperSlideLeft.setPower(speed);
        viperSlideRight.setPower(speed);
        //
        while (viperSlideLeft.isBusy() || viperSlideRight.isBusy()) {
        }
        viperSlideLeft.setPower(0.0);
        viperSlideRight.setPower(0.0);
    }


    public void ViperSlideAddPosition(double centimeters, double speed) {
        viperSlideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        viperSlideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //

        int move = (int)(Math.round(cprviperSlide * (centimeters / distancePerRotation)));
        //
        viperSlideLeft.setTargetPosition((viperSlideLeft.getCurrentPosition() + move));
        viperSlideRight.setTargetPosition((viperSlideRight.getCurrentPosition() + move));

        //
        viperSlideLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        viperSlideRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //
        viperSlideLeft.setPower(speed);
        viperSlideRight.setPower(speed);


        while (viperSlideLeft.isBusy() && viperSlideRight.isBusy()) {
        }
        viperSlideLeft.setPower(0.0);
        viperSlideRight.setPower(0.0);
        return;
    }



    public double getViperSlidePosition() {
        int viperSlidePosition = viperSlideLeft.getCurrentPosition();
        return viperSlidePosition * distancePerTick;
    }


}
