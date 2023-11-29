package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class ProgrammingBoard7_1 {
    private DigitalChannel touchsensor;
    private DcMotor Motor;

    public void init(HardwareMap hwMap) {
        touchsensor = hwMap.get(DigitalChannel.class, "touch sensor");
        touchsensor.setMode(DigitalChannel.Mode.INPUT);
        Motor = hwMap.get(DcMotor.class, "motor");
        Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isTouchSensorPressed() {
        return touchsensor.getState();
    }
    public void setMotorSpeed(double speed) {
        Motor.setPower(speed);
    }
}
