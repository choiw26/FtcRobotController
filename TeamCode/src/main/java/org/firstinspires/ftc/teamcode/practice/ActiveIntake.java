package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "ActiveIntake", group = "TeleOp")
public class ActiveIntake extends OpMode {

    private DcMotorEx motor;
    private static final double MOTOR_POWER = 0.5;

    public static int UpAngle = 90;

    public final double CPR = 537.7;
    public final int ratio = 28;


    private CRServo servo; // Continuous rotation servo

    private static final double SERVO_POWER = 1.0; // Full speed


    @Override
    public void init() {

        motor = hardwareMap.get(DcMotorEx.class, "test motor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Map the servo to the configuration name in the FTC app
        servo = hardwareMap.get(CRServo.class, "CRS");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
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


        if (gamepad1.a) {
            // Rotate in the right direction
            servo.setPower(SERVO_POWER);
        } else if (gamepad1.b) {
            // Rotate in the opposite direction
            servo.setPower(-SERVO_POWER);
        } else {
            // Stop the servo
            servo.setPower(0.0);
        }

        // Telemetry for debugging
        telemetry.addData("Servo Power", servo.getPower());
        telemetry.update();
    }

}
