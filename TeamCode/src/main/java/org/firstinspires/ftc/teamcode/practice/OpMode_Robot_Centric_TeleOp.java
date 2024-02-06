package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.practice.mechanisms.ProgrammingBoard_mecanum_wheels;

@TeleOp()
public class OpMode_Robot_Centric_TeleOp extends OpMode{
    ProgrammingBoard_mecanum_wheels pbMecanum = new ProgrammingBoard_mecanum_wheels();

    double RotationOfTiltMotor = 0;

    double RotationOfViperSlide = 0;
    double[] limitOfViperRot = {0, 5.8};

    @Override
    public void init() {
        pbMecanum.init(hardwareMap);
    }
    @Override
    public void loop() {
        //SetUp for the Mecanum Motors
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        //Actual Robot Movement
        if(gamepad1.left_stick_y > 0.15 || gamepad1.left_stick_y < -0.15 ||
                gamepad1.left_stick_x > 0.15 || gamepad1.left_stick_x < -0.15) {
            pbMecanum.drive(y, x, rx);
        }

        //Spins the robot right
        /*else if(gamepad1.right_bumper) {
            pbMecanum.setPower(0.5,-0.5,0.5,-0.5);
        }*/

        //Spins the robot left
        /*else if(gamepad1.left_bumper) {
            pbMecanum.setPower(-0.5,0.5,-0.5,0.5);

        }*/

        //Stops all movement
        /*else {
            pbMecanum.setPower(0,0,0,0);
        }*/


        //To check Wheel Spinning
        if (gamepad1.a) {
            pbMecanum.setPower(1,0,0,0);
        }
        else if (gamepad1.b) {
            pbMecanum.setPower(0,1,0,0);
        }
        else if (gamepad1.x) {
            pbMecanum.setPower(0,0,1,0);
        }
        else if (gamepad1.y) {
            pbMecanum.setPower(0,0,0,1);
        }





        //Tilt Motor Movement
        //Left bumper clicked = rotate left
        //right bumper clicked = rotate right
        //No click, break on
        if (gamepad1.left_bumper) {
            pbMecanum.setTiltMotor(-0.3);
            RotationOfTiltMotor = pbMecanum.getTiltRotation();
        } else if (gamepad1.right_bumper) {
            pbMecanum.setTiltMotor(0.3);
            RotationOfTiltMotor = pbMecanum.getTiltRotation();
        } else {
            // put the break detecting movement
            if (Math.abs(pbMecanum.getTiltRotation() - RotationOfTiltMotor) > 0.1){
                pbMecanum.viperSlideSpeed(-RotationOfTiltMotor/2);
            }
            pbMecanum.setTiltMotor(0);
        }





        // ViperSlide Movement
        //right trigger pressed - rotate right
        //left trigger pressed - rotate left
        RotationOfViperSlide = pbMecanum.getViperRotation();
        if ((gamepad1.left_trigger > 0) && RotationOfViperSlide >= limitOfViperRot[0] + 0.5) {
            pbMecanum.viperSlideSpeed(-0.5);
        } else if ((gamepad1.right_trigger > 0) && RotationOfViperSlide <= limitOfViperRot[1] - 0.5) {
            pbMecanum.viperSlideSpeed(0.5);
        } else {
            pbMecanum.viperSlideSpeed(0.0);
        }

        telemetry.addData("Number of Rotation : Viper Slide", pbMecanum.getViperRotation());
        telemetry.addData("Angle : Tilt Angle", pbMecanum.getTiltRotation());
    }
}