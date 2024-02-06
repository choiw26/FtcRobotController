package org.firstinspires.ftc.teamcode.practice;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Square {
    double length_cm = 10;

    public double getLength(DistanceUnit du) {
        return du.fromCm(length_cm);
    }
    public void setLength(double length, DistanceUnit du) {
        length_cm = du.toCm(length);
    }
}
