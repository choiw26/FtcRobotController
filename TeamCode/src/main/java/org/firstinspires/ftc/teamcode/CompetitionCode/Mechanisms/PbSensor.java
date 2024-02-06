package org.firstinspires.ftc.teamcode.CompetitionCode.Mechanisms;

//Created December 12th
//Mechanisms for Sensors
//This is a code that is imported by the Main Opmode code
//I distributed the codes in multiple documents so the main OpMode doesn't get too complicated
//Sensors List:
// 1.April Tag Recognition
// 2.Distance Sensor
// 3. Color Sensor



import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//Import all the April Tag stuff please

public class PbSensor {

    private ColorSensor colorSensorLeft; // Define distance sensor
    private ColorSensor colorSensorRight;

    public void init(HardwareMap hwmap) {

        //Linking the control hub thingy with the code
        //Distance Sensor config "distanceSensor"
        colorSensorLeft = hwmap.get(ColorSensor.class, "color sensor left");
        colorSensorRight = hwmap.get(ColorSensor.class, "color sensor right");
    }

    //Method of getting the color value for the color sensor
    public int getRedLeft() {
        return colorSensorRight.red();
    }

    public int getRedRight() {
        return colorSensorRight.red();
    }

    //

    public int getBlueLeft() {
        return colorSensorLeft.blue();
    }

    public int getBlueRight() {
        return colorSensorRight.blue();
    }

}
