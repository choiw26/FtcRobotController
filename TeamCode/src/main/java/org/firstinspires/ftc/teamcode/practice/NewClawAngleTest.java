package org.firstinspires.ftc.teamcode.practice;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class NewClawAngleTest extends OpMode{
    public static double clawOpenPosition = 0; //
    public static double closeclawposition = 1.0;
    public static double rotateGrabPosition = 0;
    public static double rotateplaceposition = 1.0;

    static double ServoPositionClaw;
    static double ServoPositionRotate;
    //

    public static double ActualClawSpeedIncrement = 0.002;
    public static double ActualRotateSpeedIncrement = 0.002;

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


//    //TODO LINEAR ACTUATOR
//    DcMotorEx linearActuator;
//
//
//    //TODO VIPERSLIDE
//
//    public DcMotorEx viperSlideLeft;
//    public DcMotorEx viperSlideRight;



    @Override
    public void init() {
        ClawServo = hardwareMap.get(Servo.class, "left servo");
        RotateServo = hardwareMap.get(Servo.class, "right servo");
        ClawServo.setPosition(clawOpenPosition);
        RotateServo.setPosition(rotateGrabPosition);

        leftTriggerPressed = false;
        leftBumperPressed = false;
        rightTriggerPressed = false;
        rightBumperPressed = false;

        // LINEAR ACTUATOR

//        linearActuator = hardwareMap.get(DcMotorEx.class, "hang motor");
//        linearActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        //VIPER SLIDE
//        //LEFT is right
//        //RIGHT IS OPposite
//
//        viperSlideLeft = hardwareMap.get(DcMotorEx.class, "viperSlideLeft");
//        viperSlideLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        viperSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        viperSlideRight = hardwareMap.get(DcMotorEx.class, "viperSlideRight");
//        viperSlideRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        viperSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        viperSlideRight.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void start() {
        ClawServo.setPosition(clawOpenPosition); //0.3333333
        RotateServo.setPosition(rotateGrabPosition); //0.5
    }

    @Override
    public void loop() {

        //CLAW - LEFT
        if(gamepad1.left_trigger>0.5){ // Left trigger pressed
            if(!leftTriggerPressed){
                clawSpeedIncrement = ActualClawSpeedIncrement;
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
                clawSpeedIncrement = -ActualClawSpeedIncrement;
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
                rotateSpeedIncrement = -ActualRotateSpeedIncrement;
            }
            rightTriggerPressed=true;
        }else{
            if(rightTriggerPressed){
                rotateSpeedIncrement = 0.0;
            }
            rightTriggerPressed=false;
        }

        if(gamepad1.right_bumper){
            if(!rightBumperPressed){
                rotateSpeedIncrement = ActualRotateSpeedIncrement;
            }
            rightBumperPressed=true;
        }else{
            if(rightBumperPressed){
                rightBumperPressed = false;
            }
        }

        ServoPositionRotate += rotateSpeedIncrement;
        ServoPositionRotate = Math.min(rotateplaceposition //0.749 rotate place bigger
                ,Math.max(rotateGrabPosition, ServoPositionRotate)); //0.5 rotate grab smaller
        RotateServo.setPosition(ServoPositionRotate);


        //  LINEAR ACTUATOR

//        if (gamepad1.a) {
//            linearActuator.setPower(0.5);
//        } else if (gamepad1.b) {
//            linearActuator.setPower(-0.5);
//        } else {
//            linearActuator.setPower(0.0);
//        }
//
//        // VIPER SLIDE
//
//        if (gamepad1.x) {
//            viperSlideRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//            viperSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            viperSlideLeft.setPower(0.5);
//            viperSlideRight.setPower(0.5);
//        } else if (gamepad1.y) {
//            viperSlideRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//            viperSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            viperSlideLeft.setPower(-0.5);
//            viperSlideRight.setPower(-0.5);
//        } else {
//            viperSlideLeft.setPower(0.0);
//            viperSlideRight.setPower(0.0);
//        }

        telemetry.addData("Rotate Servo position", ServoPositionRotate);
        telemetry.addData("Claw Servo position", ServoPositionClaw);
//        telemetry.addData("Linear Actuator Speed", linearActuator.getVelocity());
//
//        telemetry.addData("VIPER SLIDE SPEED", viperSlideLeft.getVelocity());

    }

}
