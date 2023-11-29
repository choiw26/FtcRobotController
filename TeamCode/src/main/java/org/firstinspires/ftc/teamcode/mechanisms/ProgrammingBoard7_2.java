package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
public class ProgrammingBoard7_2 {
    private DcMotor motor;
    private DigitalChannel touchSensor;
    private double ticksPerRotation;

    public void init(HardwareMap hwmap) {
        motor = hwmap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        touchSensor = hwmap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        ticksPerRotation = motor.getMotorType().getTicksPerRev();
    }

    public boolean getTouchSensor() {
        return !touchSensor.getState();
    }
    public void setMotorSpeed (double speed) {
        motor.setPower(speed);
    }
    public double motorRotation () {
        return motor.getCurrentPosition() / ticksPerRotation;
    }


}
