package org.firstinspires.ftc.teamcode.CompetitionCode.Mechanisms;

//Created December 12th
//Mechanisms for Motors
//This is a code that is imported by the Main Opmode code
//I distributed the codes in multiple documents so the main OpMode doesn't get too complicated
//Made an array of Motors
// 0 - front left, 1- back left, 2- front right, 3 - back right.

// Used https://ftcchad.com/ for Autonomous methods


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class PbMecanumWheels {
    public DcMotor[] motors = new DcMotor[4];

    //Constants
    public static final double TICKS_PER_REV = 537.6;
    public static final double MAX_RPM = 312;
    public static double WHEEL_RADIUS = 1.88976; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 16.34; // in
    private double width = 40.64; //centimeters
    private double cpr = 537.6; //ticks per rotation
    private double gearRatio = 1;
    private double diameter = 9.6; //centimeters

    double cpc = (cpr) /(Math.PI * diameter); // counts per centimeter
    //2150.8 cpr * gearRatio / (2 * pi * diameter)


    private double bias = 0.8; //default 0.8
    private double meccyBias = 0.9; //change to adjust only strafing movement

    ////

    private double conversion = cpc * bias;

    private boolean exit = false;

    ////

    public Acceleration gravity;

    //--------------

    public void init(HardwareMap hwmap) {
        motors[0] = hwmap.get(DcMotor.class, "frontLeftMotor"); // frontLeftMotor
        motors[1] = hwmap.get(DcMotor.class, "backLeftMotor"); // backLeftMotor
        motors[2] = hwmap.get(DcMotor.class, "frontRightMotor"); //frontRightMotor
        motors[3] = hwmap.get(DcMotor.class, "backRightMotor"); //BackRightMotor


        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);


        motors[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[2].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[3].setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }


    /*  Methods List
    basic methods
        setAllMotor : set all motors to a certain speed
        setPower : set power by entering 4 different numbers that assigns to each 4 motor

    Driving stage methods
        drive : uses three variable forward, right and rotate to move , Only used in Opmode_Driving (driving stage)

    Automation stage methods
        moveToPosition : Drive forward or backwards for certain Position
            eg) 5m --> 500, 1.0, back 10m --> -1000 , 1.0

        strafeToPosition : Function to go horizontally

        turnWithGyro : Turning with Gyro
            devertify
            convertify
            initGyro
            turnWithEncoder

*/
    //Method to set all motors to a certain speed
    public void setAllMotor (double speed) {
        for (DcMotor motor : motors) {
            motor.setPower(speed);
        }
    }

    //Set each motors to a certain speed
    public void setPower(double frontLeftPower, double frontRightPower, double backLeftPower
            , double backRightPower) {
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double maxSpeed = 1.0;
        maxSpeed = Math.max(maxSpeed, Math.abs(frontLeftPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(frontRightPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(backLeftPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(backRightPower));

        frontLeftPower /= maxSpeed;
        frontRightPower /= maxSpeed;
        backLeftPower /= maxSpeed;
        backRightPower /= maxSpeed;

        motors[0].setPower(frontLeftPower);
        motors[1].setPower(frontRightPower);
        motors[2].setPower(backLeftPower);
        motors[3].setPower(backRightPower);
    }

    //Only going to be used for the driving stage
    //Drive by forward right and rotate value
    public void drive(double forward, double right, double rotate) {

        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backLeftPower = forward - right + rotate;
        double backRightPower = forward + right - rotate;
        setPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }


    // Drive forward or backwards for certain Position (first : exposition, second : speed)
    // eg 5m --> 500, 1.0, back 10m --> -1000 , 1.0
    public void moveToPosition(double centimeters, double speed) {
        int move = (int)(Math.round(centimeters * conversion));
        //
        for (DcMotor motor : motors) {
            motor.setTargetPosition(motor.getCurrentPosition() + move);
        }
        //
        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        //
        setPower(speed, speed, speed, speed);
        //
        while (motors[0].isBusy() && motors[1].isBusy() &&
                motors[2].isBusy() && motors[3].isBusy()){
            if (exit) {
                setAllMotor(0.0);
                return;
            }
        }
        setAllMotor(0.0);
    }

    // Move by going horizontally
    public void strafeToPosition(double centimeters, double speed){
        //
        int move = (int)(Math.round(centimeters * cpc * meccyBias));
        //
        motors[1].setTargetPosition(motors[1].getCurrentPosition() - move);
        motors[0].setTargetPosition(motors[0].getCurrentPosition() + move);
        motors[3].setTargetPosition(motors[3].getCurrentPosition() + move);
        motors[2].setTargetPosition(motors[2].getCurrentPosition() - move);
        // Inverted because of strafing

        //
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        //
        setAllMotor(speed);

        //
        while (motors[0].isBusy() && motors[1].isBusy() && motors[2].isBusy() && motors[3].isBusy()) {}
        setAllMotor(0.0);
        return;
    }


    /*
This function is used in the turnWithGyro function in the AutonomousOpMode to set the
encoder mode and turn.

This is because the Gyro is only imported in AutonomousOpMode
 */
    public void turnWithEncoder(double input){
        for(DcMotor motor : motors){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        //
        motors[0].setPower(input);
        motors[1].setPower(input);
        motors[2].setPower(-input);
        motors[3].setPower(-input);
    }
}
