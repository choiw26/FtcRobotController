package org.firstinspires.ftc.teamcode.CompetitionCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;


import org.firstinspires.ftc.teamcode.CompetitionCode.Mechanisms.PbClaw;


@TeleOp (name = "Driving Stage")
public class DriverOpMode extends LinearOpMode {

    private PbClaw pbclaw;

    //

    //TODO LINEAR ACTUATOR
    DcMotorEx linearActuator;


    //TODO Drone Launcher
    Servo DroneLauncher;


    @Override
    public void runOpMode() throws InterruptedException {

        //Declaring the Programming Board
        //Wheels
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //TODO INDENTED

        //
        //CLAW
        pbclaw = new PbClaw();
        pbclaw.init(hardwareMap);

        //


         boolean leftTriggerPressed;
         boolean leftBumperPressed;
         boolean rightTriggerPressed;
         boolean rightBumperPressed;

         leftTriggerPressed = false;
         leftBumperPressed = false;
         rightTriggerPressed = false;
         rightBumperPressed = false;

         //
         //Viper Slide

         double viperSpeed = 0;

         //


        //Drone Launcher
        DroneLauncher = hardwareMap.servo.get("Launcher Servo");
        DroneLauncher.setPosition(0.4);

        // Linear Actuator
        //linearActuator = hardwareMap.get(DcMotorEx.class, "hang motor");


        //


        waitForStart();
        //
        pbclaw.ClawServo.setPosition(pbclaw.OpenClawPosition);
        pbclaw.RotateServo.setPosition(pbclaw.RotateGrabPosition);
        //

        while (opModeIsActive()) {
            //

            //TODO Driving Part
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );




            //ViperSlides uses gamepad2 bumpers and triggers
            //TODO ViperSlide


            if (gamepad2.y){
                pbclaw.ViperSlideSpeed(0.5);
                viperSpeed = 0.5;
            } else if (gamepad2.x) {
                pbclaw.ViperSlideSpeed(-0.5);
                viperSpeed = -0.5;
            } else {
                pbclaw.ViperSlideSpeed(0);
                viperSpeed = 0;
            }



            //TODO Claw: gamePad2(or1) triggers and bumpers
            //Claw Close - Left Trigger
            //Claw Open -  Left bumper
            //Rotate Up (to place) - Right Trigger
            //Rotate Down (to grab) - Right bumper


            //TODO CLAW

            if(gamepad2.left_trigger > 0.5){
                if(!leftTriggerPressed){
                    pbclaw.clawSpeedIncrement = 0.005;
                }
                leftTriggerPressed = true;
            }else{
                if(leftTriggerPressed){
                    pbclaw.clawSpeedIncrement = 0.0;
                }
                leftTriggerPressed = false;
            }

            if(gamepad2.left_bumper){
                if(!leftBumperPressed){
                    pbclaw.clawSpeedIncrement = -0.005;
                }
                leftBumperPressed = true;
            }else{
                if(leftBumperPressed){
                    leftBumperPressed = false;
                }
            }

            //
            pbclaw.ServoPositionClaw += pbclaw.clawSpeedIncrement;
            //
            pbclaw.ServoPositionClaw = Math.min(pbclaw.closeclawposition    //0.65
                    ,Math.max(pbclaw.OpenClawPosition    //0.33333
                            , pbclaw.ServoPositionClaw));
            //
            pbclaw.ClawServo.setPosition(pbclaw.ServoPositionClaw);


            //TODO Rotate

            if(gamepad2.right_trigger>0.5){
                if(!rightTriggerPressed){
                    pbclaw.rotateSpeedIncrement = -0.01;
                }
                rightTriggerPressed=true;
            }else{
                if(rightTriggerPressed){
                    pbclaw.rotateSpeedIncrement = 0.0;
                }
                rightTriggerPressed=false;
            }

            if(gamepad2.right_bumper){
                if(!rightBumperPressed){
                    pbclaw.rotateSpeedIncrement = 0.01;
                }
                rightBumperPressed=true;
            }else{
                if(rightBumperPressed){
                    rightBumperPressed=false;
                }
            }

            //
            pbclaw.ServoPositionRotate += pbclaw.rotateSpeedIncrement;
            //
            pbclaw.ServoPositionRotate = Math.min(pbclaw.rotateplaceposition //0.5
                    ,Math.max(pbclaw.RotateGrabPosition    //0.2789
                            , pbclaw.ServoPositionRotate));
            //
            pbclaw.RotateServo.setPosition(pbclaw.ServoPositionRotate);
            //

            //TODO Drone Launcher

            if (gamepad1.back) {
                DroneLauncher.setPosition(1.0);
            } else {
                DroneLauncher.setPosition(0.4);
            }



            // TODO HANGING MECHANISM, linear actuator.
            /*
            if (gamepad2.a) {
                linearActuator.setPower(0.5);
            } else if (gamepad2.b) {
                linearActuator.setPower(-0.5);
            } else {
                linearActuator.setPower(0.0);
            }*/


            telemetry.addData("Claw Servo Position : ",pbclaw.getClawServoPosition());
            telemetry.addData("Rotate ServoPosition : ",pbclaw.getRotateServoPosition());
            //
            telemetry.addData("\nViper Slide Speed : ", viperSpeed);
            telemetry.addData("Viper Slide Distance : ",
                    pbclaw.getViperSlidePosition() + "cm");
            telemetry.update();
        }




    }





}
