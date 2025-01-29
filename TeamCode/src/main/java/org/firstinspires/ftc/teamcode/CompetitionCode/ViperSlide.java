package org.firstinspires.ftc.teamcode.CompetitionCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.CompetitionCode.Mechanisms.PbClaw;


@TeleOp (name = "Claw experiment TeleOp")
public class ViperSlide extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

//        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
//        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
//        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
//        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");



        PbClaw pbclaw = new PbClaw();

        pbclaw.init(hardwareMap);


        //TODO Servo

        boolean leftTriggerPressed;
        boolean leftBumperPressed;
        boolean rightTriggerPressed;
        boolean rightBumperPressed;

        leftTriggerPressed = false;
        leftBumperPressed = false;
        rightTriggerPressed = false;
        rightBumperPressed = false;

        double [] hello = {0, 100, 200};

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            //TODO ViperSlide

            if (gamepad1.x){
                pbclaw.ViperSlideSpeed(1.0);
            } else if (gamepad1.y) {
                pbclaw.ViperSlideSpeed(-1.0);
            } else {
                pbclaw.ViperSlideSpeed(0.0);
            }

            if(gamepad1.left_trigger>0.5){
                    pbclaw.setCRServoSpeed(2.0);
            }else if (gamepad1.left_bumper){
                pbclaw.setCRServoSpeed(-2.0);
            } else {
                pbclaw.setCRServoSpeed(0);
            }


//            //TODO Claw
//            //Claw
//
//            if(gamepad1.left_trigger>0.5){
//                if(!leftTriggerPressed){
//                    pbclaw.clawSpeedIncrement = 0.01;
//                }
//                leftTriggerPressed=true;
//            }else{
//                if(leftTriggerPressed){
//                    pbclaw.clawSpeedIncrement = 0.0;
//                }
//                leftTriggerPressed=false;
//            }
//
//            if(gamepad1.left_bumper){
//                if(!leftBumperPressed){
//                    pbclaw.clawSpeedIncrement = -0.01;
//                }
//                leftBumperPressed=true;
//            }else{
//                if(leftBumperPressed){
//                    leftBumperPressed = false;
//                }
//            }
//
//            pbclaw.ServoPositionClaw += pbclaw.clawSpeedIncrement;
//            pbclaw.ServoPositionClaw = Math.min(1.0,Math.max(0.0,pbclaw.ServoPositionClaw));
//            pbclaw.ClawServo.setPosition(pbclaw.ServoPositionClaw);
//
//            //Rotate
//
//
//            if(gamepad1.right_trigger>0.5){
//                if(!rightTriggerPressed){
//                    pbclaw.rotateSpeedIncrement = -0.01;
//                }
//                rightTriggerPressed=true;
//            }else{
//                if(rightTriggerPressed){
//                    pbclaw.rotateSpeedIncrement = 0.0;
//                }
//                rightTriggerPressed=false;
//            }
//
//            if(gamepad1.right_bumper){
//                if(!rightBumperPressed){
//                    pbclaw.rotateSpeedIncrement = 0.01;
//                }
//                rightBumperPressed=true;
//            }else{
//                if(rightBumperPressed){
//                    rightBumperPressed=false;
//                }
//            }
//
//            pbclaw.ServoPositionRotate += pbclaw.rotateSpeedIncrement;
//            pbclaw.ServoPositionRotate = Math.min(1.0,Math.max(0.0, pbclaw.ServoPositionRotate));
//            pbclaw.RotateServo.setPosition(pbclaw.ServoPositionRotate);
//
//            //
//
//
//            telemetry.addData("LeftServoPosition",pbclaw.ServoPositionClaw);
//            telemetry.addData("RightServoPosition",pbclaw.ServoPositionRotate);

            telemetry.addData("ViperSlidePosition",  pbclaw.getViperSlidePosition() + "Percent");
            telemetry.update();
        }
    }
}