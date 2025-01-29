package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name = "Omni Chassis Robot", group = "TeleOp")
public class OmniChassisRobot extends OpMode {

    // Declare motors for the two back wheels
    private DcMotorEx leftBackMotor;
    int leftCPR = 0;
    private DcMotorEx rightBackMotor;
    int rightCPR = 0;

    // Constants for motor power
    private static final double MAX_POWER = 1.0;

    @Override
    public void init() {
        // Initialize the motors
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Set the motors to run with brake behavior when power is set to zero
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        // Get the joystick values
        double leftStickX = gamepad1.left_stick_x;  // Left joystick X (side-to-side movement)
        double leftStickY = -gamepad1.left_stick_y; // Left joystick Y (forward/backward movement)
        double rightStickX = gamepad1.right_stick_x; // Right joystick X (rotation)

        // Calculate motor powers for the back wheels
        // For omni wheels, the front wheels don't have motors, so the back wheels handle both translation and rotation.

        double leftPower = leftStickY + leftStickX + rightStickX;
        double rightPower = leftStickY - leftStickX - rightStickX;

        // Normalize powers to ensure they're within the motor power range (-1 to 1)
        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > MAX_POWER) {
            leftPower /= maxPower;
            rightPower /= maxPower;
        }

        // Set the motor powers
        leftBackMotor.setPower(leftPower);
        rightBackMotor.setPower(rightPower);

        // Telemetry for debugging
        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
        telemetry.update();
    }
}