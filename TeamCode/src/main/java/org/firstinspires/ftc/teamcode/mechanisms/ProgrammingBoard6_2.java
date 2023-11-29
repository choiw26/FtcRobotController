package org.firstinspires.ftc.teamcode.mechanisms;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ProgrammingBoard6_2 {
    private DigitalChannel barometerPressure;

    public void init(HardwareMap hwMap) {
        barometerPressure = hwMap.get(DigitalChannel.class, "barometer");
        barometerPressure.setMode(DigitalChannel.Mode.INPUT);
    }
    public boolean getAtmosphericPressure() {
        return barometerPressure.getState();
    }

}
