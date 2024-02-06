package org.firstinspires.ftc.teamcode.practice.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class MecanumWheelsAutomationCalibrate extends LinearOpMode {
    //
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;

    //Calculate encoder conversion
    double cpr = 2150.8; //counts per rotation
    Double diameter = 3.780; //inches per rotation
    Double cpi = (cpr) / (Math.PI * diameter); //counts per inch -> counts per rotation / circumference
    Double bias = 0.8;
    //
    Double conversion = cpi * bias;
    //
    public void runOpMode() {
        //
        frontleft = hardwareMap.dcMotor.get("frontLeftMotor");
        frontright = hardwareMap.dcMotor.get("frontRightMotor");
        backleft = hardwareMap.dcMotor.get("backLeftMotor");
        backright = hardwareMap.dcMotor.get("backRightMotor");
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        //
        waitForStart();
        //
        moveToPosition(20, 0.2);//Don't change this line, unless you want to calibrate with different speeds
        //
        return;
    }
    //
    /*
    This function's purpose is simply to drive forward or backward.
    To drive backward, simply make the inches input negative.
     */
    public void moveToPosition(double inches, double speed) {
        //
            int move = (int) (Math.round(inches * conversion));
            //
            frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
            frontright.setTargetPosition(frontright.getCurrentPosition() + move);
            backleft.setTargetPosition(backleft.getCurrentPosition() + move);
            backright.setTargetPosition(backright.getCurrentPosition() + move);
            //
            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //
            frontleft.setPower(speed);
            frontright.setPower(speed);
            backleft.setPower(speed);
            backright.setPower(speed);


        //
            while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
            }
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);

    }
}
