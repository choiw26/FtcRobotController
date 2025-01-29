package org.firstinspires.ftc.teamcode.CompetitionCode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;

@Config
@TeleOp(name = "2025 Driver OpMode Copy", group = "TeleOp")
public class DriverOpMode2025_Copy extends LinearOpMode {

    // Constants for servo positions
    public static double SERVO_RETRACTED = 0.53;
    public static double SERVO_EXTENDED = 0.49;

    // Variables for controlling servo speeds and positions
    public static final double CLAW_SPEED_INCREMENT = 0.002;
    private double clawSpeedIncrement = 0;
    //
    public static double ROTATE_SERVO_INITIAL = 0.5;
    public static double ROTATE_SERVO_TILTED = 0.3;

    // Servo instances
    private Servo clawServo;  // LEFT
    private Servo rotateServo; // RIGHT

    // Flags for button press state
    private boolean leftTriggerPressed;
    private boolean leftBumperPressed;
    private boolean rightTriggerPressed;
    private boolean rightBumperPressed;

    // Motor instances
    private DcMotorEx motor;
    private static final double MOTOR_POWER = 1.0;
    private static final double COUNTS_PER_REV = 1150;  // Counts per revolution

    // Intake system (CRServo)
    private CRServo intakeServo; // Continuous rotation servo for intake
    private static final double INTAKE_SERVO_POWER = 1.0; // Full speed for intake servo

    // Intake motor control variables
    private static int intakeMotorPosition = 0;
    public static final int INTAKE_MOTOR_MIN_POSITION = -700;
    public static final int INTAKE_MOTOR_MAX_POSITION = 1100;
    public static final int INTAKE_MOTOR_SPEED = 10;

    // Viper Slide motor control
    private DcMotorEx viperSlideLeft;
    private DcMotorEx viperSlideRight;
    public static final double VIPER_SLIDE_SPEED = 0.5;
    private static int viperSlidePosition = 0;
    private static final double CPR_VIPER_SLIDE = 537.7; // ticks per rotation for viper slide motor
    private static final double MAX_ROTATION = 8.1; // Maximum rotation for viper slide

    // Damping for driving control
    private static final double DRIVE_DAMPENING = 3;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Roadrunner drive system
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Initialize servos and motors
        initializeServosAndMotors();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Main loop for controlling robot
        while (opModeIsActive()) {
            // Drive control using Roadrunner
            handleDriveControl(drive);

            // Motor rotation logic (active intake motor with encoder control)
            handleIntakeMotorControl();

            // Handle claw servo movement based on button input
            handleClawServoControl();

            // Handle rotate servo movement
            handleRotateServoControl();

            // Handle intake system (CRServo)
            handleIntakeServoControl();

            // Control Viper Slides (up/down)
            handleViperSlideControl();

            // Display telemetry data
            telemetry.update();
        }
    }

    private void initializeServosAndMotors() {
        clawServo = hardwareMap.get(Servo.class, "left servo");
        rotateServo = hardwareMap.get(Servo.class, "right servo");
        clawServo.setPosition(SERVO_RETRACTED);
        rotateServo.setPosition((ROTATE_SERVO_INITIAL));

        motor = hardwareMap.get(DcMotorEx.class, "test motor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorEx.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeServo = hardwareMap.get(CRServo.class, "CRS");

        viperSlideLeft = hardwareMap.get(DcMotorEx.class, "viperSlideLeft");
        viperSlideLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        viperSlideLeft.setDirection(DcMotorEx.Direction.REVERSE);
        viperSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        viperSlideRight = hardwareMap.get(DcMotorEx.class, "viperSlideRight");
        viperSlideRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        viperSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        viperSlidePosition = viperSlideRight.getCurrentPosition();
    }

    private void handleDriveControl(SampleMecanumDrive drive) {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y / DRIVE_DAMPENING,
                        -gamepad1.left_stick_x / DRIVE_DAMPENING,
                        -gamepad1.right_stick_x / DRIVE_DAMPENING
                )
        );
    }

    private void handleIntakeMotorControl() {
        if (gamepad1.y && !gamepad1.x) {
            intakeMotorPosition += INTAKE_MOTOR_SPEED;
        } else if (gamepad1.x && !gamepad1.y) {
            intakeMotorPosition -= 2 * INTAKE_MOTOR_SPEED;
        }

        if (intakeMotorPosition < 0 && !(gamepad1.x || gamepad1.y)) {
            intakeMotorPosition += INTAKE_MOTOR_SPEED;
        }

        intakeMotorPosition = Math.max(INTAKE_MOTOR_MIN_POSITION, Math.min(INTAKE_MOTOR_MAX_POSITION, intakeMotorPosition));

        motor.setTargetPosition(intakeMotorPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(MOTOR_POWER);
    }

    private void handleClawServoControl() {
        if (gamepad1.left_bumper) {
            if (!leftBumperPressed) {
                clawSpeedIncrement = -CLAW_SPEED_INCREMENT;
            }
            leftBumperPressed = true;
        } else {
            if (leftBumperPressed) {
                leftBumperPressed = false;
            }
        }

        double clawPosition = clawServo.getPosition() + clawSpeedIncrement;
        clawPosition = Math.min(SERVO_RETRACTED, Math.max(SERVO_EXTENDED, clawPosition));
        clawServo.setPosition(clawPosition);

        telemetry.addData("Claw Position", clawPosition);
    }

    private void handleRotateServoControl() {
        if (gamepad1.right_bumper) {
            rotateServo.setPosition(ROTATE_SERVO_TILTED); // Example value for rotated position
        } else {
            rotateServo.setPosition(ROTATE_SERVO_INITIAL); // Reset to initial position
        }

        telemetry.addData("Rotate Servo Position", rotateServo.getPosition());
    }

    private void handleIntakeServoControl() {
        if (gamepad1.a) {
            intakeServo.setPower(INTAKE_SERVO_POWER);  // Forward
        } else if (gamepad1.b) {
            intakeServo.setPower(-INTAKE_SERVO_POWER); // Reverse
        } else {
            intakeServo.setPower(0.0); // Stop
        }

        telemetry.addData("Intake Servo Power", intakeServo.getPower());
    }

    private void handleViperSlideControl() {
        if (gamepad1.right_bumper) {
            viperSlidePosition += 10;  // Move slides up
        } else if (gamepad1.right_trigger > 0.5) {
            viperSlidePosition -= 10;  // Move slides down
        }

        viperSlidePosition = Math.max(0, Math.min(viperSlidePosition, (int)(MAX_ROTATION * CPR_VIPER_SLIDE)));

        viperSlideLeft.setTargetPosition(viperSlidePosition);
        viperSlideRight.setTargetPosition(viperSlidePosition);

        viperSlideLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        viperSlideRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        viperSlideLeft.setPower(VIPER_SLIDE_SPEED);
        viperSlideRight.setPower(VIPER_SLIDE_SPEED);

        telemetry.addData("Viper Slide Left Position", viperSlideLeft.getCurrentPosition());
        telemetry.addData("Viper Slide Right Position", viperSlideRight.getCurrentPosition());
    }
}
