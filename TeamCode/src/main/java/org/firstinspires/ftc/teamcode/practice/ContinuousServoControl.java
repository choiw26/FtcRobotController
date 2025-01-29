package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "ContinuousServoControl", group = "TeleOp")
public class ContinuousServoControl extends OpMode {

    private CRServo servo; // Continuous rotation servo

    private static final double SERVO_POWER = 1.0; // Full speed

    @Override
    public void init() {
        // Map the servo to the configuration name in the FTC app
        servo = hardwareMap.get(CRServo.class, "CRS");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            // Rotate in the right direction
            servo.setPower(SERVO_POWER);
        } else if (gamepad1.y) {
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