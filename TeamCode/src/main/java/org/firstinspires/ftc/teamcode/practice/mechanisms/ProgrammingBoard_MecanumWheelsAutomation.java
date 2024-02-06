package org.firstinspires.ftc.teamcode.practice.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;



public class ProgrammingBoard_MecanumWheelsAutomation {
    private DcMotor[] motors = new DcMotor[4];

    public void init(HardwareMap hwmap) {
        motors[0] = hwmap.get(DcMotor.class, "frontLeftMotor"); // frontLeftMotor
        motors[1] = hwmap.get(DcMotor.class, "backLeftMotor"); // backLeftMotor
        motors[2] = hwmap.get(DcMotor.class, "frontRightMotor"); //frontRightMotor
        motors[3] = hwmap.get(DcMotor.class, "backRightMotor"); //BackRightMotor

        motors[0].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[1].setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /*public void changeDirection (boolean fl, boolean bl, boolean fr, boolean br) {
        if (fl) {
            rightfront.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (bl) {

        }
        if (fr) {

        }
        if (br) {

        }
    } */
    public void setAllMotor (double speed) {
        for (DcMotor motor : motors) {
                motor.setPower(speed);
        }
    }

    public void leftFrontSpeed (double speed) {
        motors[0].setPower(speed);
    }
    public void leftBackSpeed (double speed) {
        motors[1].setPower(speed);
    }
    public void rightFrontSpeed (double speed) {
        motors[2].setPower(speed);
    }
    public void rightBackSpeed (double speed) {
        motors[3].setPower(speed);
    }
}
