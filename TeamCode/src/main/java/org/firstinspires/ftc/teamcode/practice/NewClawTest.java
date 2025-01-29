package org.firstinspires.ftc.teamcode.practice;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class NewClawTest extends OpMode{
    public static double clawOpenPosition = 0.5;
    public static double closeclawposition = 0.65;
    public static double rotateGrabPosition = 0.28;
    public static double rotateplaceposition = 0.5;

    double ServoPositionClaw;
    double ServoPositionRotate;
    //

    public static double ActualClawSpeedIncrement = 0.007;
    public static double ActualRotateSpeedIncrement = 0.005;

    public double clawSpeedIncrement = 0;
    public double rotateSpeedIncrement = 0;

    Servo ClawServo;  // LEFT
    Servo RotateServo; //RIGHT
    //

    boolean leftTriggerPressed;
    boolean leftBumperPressed;
    boolean rightTriggerPressed;
    boolean rightBumperPressed;
    //





    @Override
    public void init() {
        ClawServo = hardwareMap.get(Servo.class, "left servo");//TODO CHANGEs MADE OPPOSITE
        RotateServo = hardwareMap.get(Servo.class, "right servo");
        ServoPositionRotate = RotateServo.getPosition();
        ServoPositionClaw = ClawServo.getPosition();
        ClawServo.setPosition(ServoPositionRotate);
        RotateServo.setPosition(ServoPositionClaw);

        leftTriggerPressed = false;
        leftBumperPressed = false;
        rightTriggerPressed = false;
        rightBumperPressed = false;
    }

    public void start() {
        ServoPositionRotate = RotateServo.getPosition();
        ServoPositionClaw = ClawServo.getPosition();
    }

    @Override
    public void loop() {

        //TODO Claw Controls : gamePad2(or1) triggers and bumpers
        //Claw Close - Left Trigger
        //Claw Open -  Left bumper
        //Rotate Up (to place) - Right Trigger
        //Rotate Down (to grab) - Right bumper


        //CLAW - LEFT
        if(gamepad1.left_trigger>0.5){
            if(!leftTriggerPressed){
                clawSpeedIncrement = 0.01;
            }
            leftTriggerPressed = true;
        }else{
            if(leftTriggerPressed){
                clawSpeedIncrement = 0.0;
            }
            leftTriggerPressed = false;
        }

        if(gamepad1.left_bumper){
            if(!leftBumperPressed){
                clawSpeedIncrement = -0.01;
            }
            leftBumperPressed = true;
        }else{
            if(leftBumperPressed){
                leftBumperPressed = false;
            }
        }

        ServoPositionClaw += clawSpeedIncrement;
        ServoPositionClaw = Math.min(closeclawposition //0.65 claw close bigger
                ,Math.max(clawOpenPosition, ServoPositionClaw)); //0.33333 claw open smaller
        ClawServo.setPosition(ServoPositionClaw);



        //Rotate - RIGHT

        if(gamepad1.right_trigger>0.5){
            if(!rightTriggerPressed){
                rotateSpeedIncrement = -0.01;
            }
            rightTriggerPressed = true;
        }else{
            if(rightTriggerPressed){
                rotateSpeedIncrement = 0.0;
            }
            rightTriggerPressed = false;
        }

        if(gamepad1.right_bumper){
            if(!rightBumperPressed){
                rotateSpeedIncrement = 0.01;
            }
            rightBumperPressed = true;
        }else{
            if(rightBumperPressed){
                rightBumperPressed = false;
            }
        }

        ServoPositionRotate += rotateSpeedIncrement;
        ServoPositionRotate = Math.min(rotateplaceposition //0.749 rotate place bigger
                ,Math.max(rotateGrabPosition, ServoPositionRotate)); //0.5 rotate grab smaller
        RotateServo.setPosition(ServoPositionRotate);
        ServoPositionRotate = RotateServo.getPosition();

        telemetry.addData("Rotate Servo position", ServoPositionRotate);
        telemetry.addData("Rotate Servo increment", rotateSpeedIncrement);
        telemetry.addData("\nClaw Servo position", ServoPositionClaw);
        telemetry.addData("Claw Servo increment", clawSpeedIncrement);



    }

}
