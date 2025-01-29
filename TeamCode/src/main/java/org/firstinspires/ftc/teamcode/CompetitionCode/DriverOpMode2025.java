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
@TeleOp(name = "2025 Driver OpMode", group = "TeleOp")
public class DriverOpMode2025 extends LinearOpMode {

    // Extension
    public double dampening = 3;
    public static double ServoRetracted = 0.53; //
    public static double ServoExtended = 0.49;
    double ServoPositionClaw;
    public static double ROTATE_SERVO_INITIAL = 0.46;
    public static double ROTATE_SERVO_TILTED = 0.55;

    public static double ActualClawSpeedIncrement = 0.002;
    public static double ActualRotateSpeedIncrement = 0.002;

    public double clawSpeedIncrement = 0;
    public double rotateSpeedIncrement = 0;

    Servo ClawServo;  // LEFT
    Servo RotateServo; // RIGHT

    boolean leftTriggerPressed;
    boolean leftBumperPressed;
    boolean rightTriggerPressed;
    boolean rightBumperPressed;

    // Rotation motor
    private DcMotorEx motor;
    private static final double MOTOR_POWER = 1.0;

    //Extension Motor:
    private DcMotorEx ExtensionMotor;
    private static int ExtensionMotorPosition = 0;  // Encoder position tracker for intake motor
    private static final double EXTENSION_MOTOR_POWER = 1.0;

    public static int ExtensionMotorMinPosition = 0;  // Minimum encoder position
    public static int ExtensionMotorMaxPosition = 273 * 3;   // Maximum encoder position
    public static int ExtensionMotorSpeed = 1;  // Speed for moving motor


    public static int UpAngle = 90;

    public static final double CPR = 1150;
    public static final int ratio = 2;

    // Intake
    private CRServo servo; // Continuous rotation servo

    private static double SERVO_POWER = 1.0; // Full speed

    // New Encoder-controlled motor for rotating intake
    private static int intakeMotorPosition = 0;  // Encoder position tracker for intake motor

    public static int intakeMotorMinPosition = -500;  // Minimum encoder position
    public static int intakeMotorMaxPosition = 1200;   // Maximum encoder position
    public static int intakeMotorSpeed = 10;  // Speed for moving motor

    // VIPER SLIDE
    public DcMotorEx viperSlideLeft;
    public DcMotorEx viperSlideRight;
    boolean exit = false;

    public double cprviperSlide = 537.7; // ticks per rotation for viper slide motor;
    public double distancePerRotation = 12; // centimeters per rotation

    public double distancePerTick = distancePerRotation / cprviperSlide;
    public final double maxRotation = 8.1; // Rotation

    public static double viperSlideSpeed = 0.5;

    public static int viperSlidePosition = 0;
    public static int viperSlideEncoder = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        ClawServo = hardwareMap.get(Servo.class, "left servo");
        ClawServo.setPosition(ServoRetracted);
        RotateServo = hardwareMap.get(Servo.class, "right servo");
        RotateServo.setPosition((ROTATE_SERVO_INITIAL));

        ExtensionMotor = hardwareMap.get(DcMotorEx.class, "extend motor");
        ExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ExtensionMotor.setDirection(DcMotorEx.Direction.REVERSE);
        ExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ExtensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor = hardwareMap.get(DcMotorEx.class, "test motor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorEx.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Map the servo to the configuration name in the FTC app
        servo = hardwareMap.get(CRServo.class, "CRS");
        telemetry.addData("Status", "Initialized");

        // VIPER SLIDE
        viperSlideLeft = hardwareMap.get(DcMotorEx.class, "viperSlideLeft");
        viperSlideLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        viperSlideLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        viperSlideRight = hardwareMap.get(DcMotorEx.class, "viperSlideRight");
        viperSlideRight.setDirection(DcMotorEx.Direction.REVERSE);
        viperSlideRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        viperSlideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        viperSlidePosition = viperSlideRight.getCurrentPosition();

        telemetry.update();

        waitForStart();
        ClawServo.setPosition(ServoRetracted); // 0.3333333


        viperSlideRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        viperSlideLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {
            // Driving using a java file imported from Roadrunner
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y / dampening,
                            -gamepad1.left_stick_x / dampening,
                            -gamepad1.right_stick_x / dampening
                    )
            );

            // Motor rotation logic (Active intake motor with encoder control)
            if (gamepad1.y && !gamepad1.x) {
                intakeMotorPosition += intakeMotorSpeed;
            } else if (gamepad1.x && !gamepad1.y) {
                intakeMotorPosition -= 2 * intakeMotorSpeed;
            }

            // If the motor position is negative and no button is pressed, return to 0
            if (intakeMotorPosition < 0 && !(gamepad1.x || gamepad1.y)) {
                intakeMotorPosition += intakeMotorSpeed;
            }

            // Constrain the intake motor position within the boundary limits
            intakeMotorPosition = Math.max(intakeMotorMinPosition, Math.min(intakeMotorMaxPosition, intakeMotorPosition));

            // Set the intake motor position using encoder values
            motor.setTargetPosition(intakeMotorPosition);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(MOTOR_POWER);  // Run motor to target positionaw


            // EXTENSION rotation logic (Extension motor with encoder control)
            if (gamepad1.left_bumper && !(gamepad1.left_trigger > 0.5)) {
                ExtensionMotorPosition += ExtensionMotorSpeed;
            } else if ((gamepad1.left_trigger > 0.5) && !gamepad1.left_bumper) {
                ExtensionMotorPosition -= ExtensionMotorSpeed;
            }

            // Constrain the Extension motor position within the boundary limits
            ExtensionMotorPosition = Math.max(ExtensionMotorMinPosition, Math.min(ExtensionMotorMaxPosition, ExtensionMotorPosition));

            // Set the Extension motor position using encoder values
            ExtensionMotor.setTargetPosition(ExtensionMotorPosition);
            ExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ExtensionMotor.setPower(EXTENSION_MOTOR_POWER);  // Run Extension to target position



            telemetry.addData("Rotation Encoder", motor.getCurrentPosition());
            telemetry.addData("Extension Encoder", ExtensionMotor.getCurrentPosition());



            //BOX SERVO:
            if (gamepad1.right_bumper) {
                RotateServo.setPosition(ROTATE_SERVO_TILTED); // Example value for rotated position
            } else {
                RotateServo.setPosition(ROTATE_SERVO_INITIAL); // Reset to initial position
            }

            telemetry.addData("Rotate Servo Position", RotateServo.getPosition());


            // Active intake system
            if (gamepad1.a) {
                servo.setPower(SERVO_POWER);
            } else if (gamepad1.b) {
                servo.setPower(-SERVO_POWER);
            } else {
                servo.setPower(0.0);
            }

            telemetry.addData("Servo Power", servo.getPower());

            // Viper Slides control (both up/down)

            if (gamepad1.dpad_up) {
                // Move the slides up
                viperSlidePosition += viperSlideEncoder; // Adjust step size as needed
            } else if (gamepad1.dpad_down) {
                // Move the slides down
                viperSlidePosition -= viperSlideEncoder; // Adjust step size as needed

            }

            // Constrain the position within physical limits
            viperSlidePosition = Math.max(0, Math.min(viperSlidePosition, 4355));

            // Set the target position and ensure the motors run to the target
            viperSlideLeft.setTargetPosition(viperSlidePosition);
            viperSlideRight.setTargetPosition(viperSlidePosition);

            viperSlideLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            viperSlideRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // Apply power to maintain or move to the target position
            viperSlideLeft.setPower(viperSlideSpeed);
            viperSlideRight.setPower(viperSlideSpeed);

            telemetry.addData("Viper Slide Right Position", viperSlideRight.getCurrentPosition());
            telemetry.addData("Viper Slide Left Position", viperSlideLeft.getCurrentPosition());
            telemetry.update();
        }
    }
}