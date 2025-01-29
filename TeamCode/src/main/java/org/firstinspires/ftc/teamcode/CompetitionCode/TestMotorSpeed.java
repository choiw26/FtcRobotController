package org.firstinspires.ftc.teamcode.CompetitionCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "TestMotorSpeed", group = "TeleOp")
public class TestMotorSpeed extends OpMode {

    private DcMotorEx motor;
    public static double MOTOR_POWER = 0.5;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "test motor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int targetPosition;
        if (gamepad1.x && !gamepad1.y) {
            motor.setPower(MOTOR_POWER);
        } else if (gamepad1.y){
            motor.setPower(-MOTOR_POWER);
        } else  {
            motor.setPower(0.0);
        }

        telemetry.addData("Motor position", motor.getCurrentPosition());
        telemetry.update();
    }

}