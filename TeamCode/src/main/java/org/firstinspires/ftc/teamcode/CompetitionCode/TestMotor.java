package org.firstinspires.ftc.teamcode.CompetitionCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp(name = "TestMotor", group = "TeleOp")
public class TestMotor extends OpMode {

    private DcMotorEx motor;
    private static final double MOTOR_POWER = 0.5;

    public static int UpAngle = 90;

    public final double CPR = 537.7;
    public final int ratio = 28;

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
        if (gamepad1.x) {
            targetPosition = (int) (UpAngle /360 * CPR * ratio);
        } else {
            targetPosition = 0;
        }
        motor.setTargetPosition(targetPosition);
        motor.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        motor.setPower(.2);
        telemetry.addData("Motor Angle", motor.getCurrentPosition()/CPR * 360);
        telemetry.update();
    }

}