package org.firstinspires.ftc.teamcode.practice.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap; // The robot parts that is going to be used
public class ProgrammingBoard {
    //Define variables (one variable for each parts + extra using variable)
    private DigitalChannel sensor;
    private DcMotor motor;
    private double ticksPerRotation;

    public void init (HardwareMap hwmap) { //init function to connect each variable with the actual robot
        sensor = hwmap.get(DigitalChannel.class, "touch_sensor");
        sensor.setMode(DigitalChannel.Mode.INPUT);
        motor = hwmap.get(DcMotor.class, "DcMotor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        ticksPerRotation = motor.getMotorType().getTicksPerRev();
    }

    //Functions to execute commands
    public boolean ifPressed() {
        return !sensor.getState();
    }
    public void setMotor(double speed) {
        motor.setPower(speed);
    }
    public double NumberOfRev () {
        return motor.getCurrentPosition()/ticksPerRotation;
    }
}
