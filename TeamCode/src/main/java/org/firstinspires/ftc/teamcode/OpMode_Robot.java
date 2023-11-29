package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard_mecanum_wheels;

@TeleOp()
public class OpMode_Robot extends OpMode{
    ProgrammingBoard_mecanum_wheels pbMecanum = new ProgrammingBoard_mecanum_wheels();

    @Override
    public void init() {
        pbMecanum.init(hardwareMap);
    }
    @Override
    public void loop() {
        //SetUp for the Mecanum Motors
        /*double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double leftFront = r * Math.cos(robotAngle) + rightX;
        final double rightFront = r * Math.sin(robotAngle) - rightX;
        final double leftBack = r * Math.sin(robotAngle) + rightX;
        final double rightBack = r * Math.cos(robotAngle) - rightX;

        //setting the speed for the motors
        pbMecanum.leftFrontSpeed(leftFront);
        pbMecanum.rightFrontSpeed(rightFront);
        pbMecanum.leftBackSpeed(leftBack);
        pbMecanum.rightBackSpeed(rightBack);

        //
        //button for individual motors
        if (gamepad1.x) {
            pbMecanum.rightBackSpeed(0.39);
        }
        if (gamepad1.y) {
            pbMecanum.rightFrontSpeed(1.0);
        }
        if (gamepad1.a) {
            pbMecanum.leftBackSpeed(0.76);
        }
        if (gamepad1.b) {
            pbMecanum.leftFrontSpeed(0.23);
        }
        /*
         */

/**/
        // ViperSlide

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
        telemetry.update();
    }
}