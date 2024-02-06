package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.practice.mechanisms.ProgrammingBoard_mecanum_wheels;

@TeleOp()
public class OpMode_Field_Centric_TeleOp2 extends OpMode{
    ProgrammingBoard_mecanum_wheels pbMecanum = new ProgrammingBoard_mecanum_wheels();

    private IMU imu;
    @Override
    public void init() {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        DcMotor viperslide =  hardwareMap.dcMotor.get("viperslide");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        viperslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

    }
    @Override
    public void loop() {
        double y = - gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        pbMecanum.setPower(frontLeftPower,frontRightPower,backLeftPower,backRightPower);

        if (gamepad2.left_bumper && !gamepad2.right_bumper) {
            //pbMecanum.setViperSlide(-0.5);
        } else if (gamepad2.right_bumper && !gamepad2.left_bumper) {
            //pbMecanum.setViperSlide(0.5);
        } else {

        }


        /*if (gamepad1.left_bumper) {
            pbMecanum.setTiltMotor(-0.1);
        } else if (gamepad1.right_bumper) {
            pbMecanum.setTiltMotor(0.1);
        } else {
            pbMecanum.setTiltMotor(0);
        }*/
        /*
         */

/**/
        // ViperSlide
/*
        if (gamepad1.right_bumper && !gamepad1.left_bumper) {
            pbMecanum.viperslideSpeed(-0.5);
        } else if (gamepad1.left_bumper && !gamepad1.right_bumper) {
            pbMecanum.viperslideSpeed(0.5);
        } else {
            pbMecanum.viperslideSpeed(0.0);
        }
        telemetry.addData("Number of Rotations", pbMecanum.getViperRotation());
        if (gamepad1.a) {
            telemetry.addData("gamepad1 a", 1);
        }
        else {
            telemetry.addData("gamepad1 a", 0);
        }
        telemetry.update();*/
    }
}