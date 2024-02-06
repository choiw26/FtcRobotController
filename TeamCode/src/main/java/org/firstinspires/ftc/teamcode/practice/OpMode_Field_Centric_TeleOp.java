package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.practice.mechanisms.ProgrammingBoard_mecanum_wheels;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@TeleOp()
public class OpMode_Field_Centric_TeleOp extends OpMode{
    ProgrammingBoard_mecanum_wheels pbMecanum = new ProgrammingBoard_mecanum_wheels();

    private IMU imu;
    @Override
    public void init() {
        pbMecanum.init(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }
    @Override
    public void loop() {
        //SetUp for the Mecanum Motors
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
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

        pbMecanum.setPower(rotY + rotX + rx,rotY - rotX - rx
                ,rotY - rotX + rx,rotY + rotX - rx);





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