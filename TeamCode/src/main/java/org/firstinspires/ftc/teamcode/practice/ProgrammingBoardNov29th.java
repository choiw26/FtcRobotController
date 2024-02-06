package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import javax.sql.StatementEvent;

public class ProgrammingBoardNov29th {
    private DcMotor motor; //DcMotor
    private DigitalChannel Digital; //Sensor
    private Servo servo; //Servo

    private AnalogInput pot;

    private ColorSensor colorSensor;

    private DistanceSensor distanceSensor;

    private double ticksPerRotation;

    private IMU imu;

    public void init(HardwareMap hwmap) {
        motor = hwmap.get(DcMotor.class, "DcMotor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Digital = hwmap.get(DigitalChannel.class, "Digital Channel");
        Digital.setMode(DigitalChannel.Mode.INPUT);
        ticksPerRotation = motor.getMotorType().getTicksPerRev();

        servo = hwmap.get(Servo.class, "Servo");

        pot = hwmap.get(AnalogInput.class, "Analog Input");

        colorSensor = hwmap.get(ColorSensor.class, "Color Sensor");

        distanceSensor = hwmap.get(DistanceSensor.class, "Distance Sensor");

        imu = hwmap.get(IMU.class, "IMU");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public void setMotorSpeed(double Speed) {
        motor.setPower(Speed);
    }

    public boolean touchSensorTouched () {
        return !Digital.getState();
    }
    public double getPotAngle () {
        return Range.scale(pot.getVoltage(), 0, pot.getMaxVoltage(), 0, 270);
    }

    public int getamountRed() {
        return colorSensor.red();
    }

    public double getDistance(DistanceUnit du) {
        return distanceSensor.getDistance(du);
    }

    public double getHeading(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }

}
