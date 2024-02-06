package org.firstinspires.ftc.teamcode.CompetitionCode;

import  com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;

import org.firstinspires.ftc.teamcode.CompetitionCode.Mechanisms.PbClaw;
import org.firstinspires.ftc.teamcode.CompetitionCode.Mechanisms.PbSensor;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous
public class AutonomousOpModeRedBack extends LinearOpMode {
    private SampleMecanumDrive drive;
    private ColorSensor colorSensor;
    public static int redValue = 30;
    public static int blueValue = 30;

    @Override
    public void runOpMode() {
        PbClaw pbGrab = new PbClaw();
        pbGrab.init(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);

        int SpikeMark = 0;


        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(-36,-66, Math.toRadians(90)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-36,0, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(0,0, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(54,-39, Math.toRadians(0)), Math.toRadians(0))
                //

                //
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(54,-36, Math.toRadians(0)), Math.toRadians(0))
                .strafeLeft(10)
                .build();




        waitForStart();
        if(isStopRequested()) return;

        sleep(5000);

        //

        pbGrab.ClawClose();
        pbGrab.RotatePlace();
        //pbGrab.AngleViperRotate(20, 0.9);
        //while (pbGrab.viperRotate.isBusy()) { }

        //

        drive.followTrajectory(traj1);

        //
        //pbGrab.AngleViperRotate(49, 0.9);
        //while (pbGrab.viperRotate.isBusy()) { }
        //pbGrab.ViperSlideSetPosition(30, 0.5);
        //while (pbGrab.viperSlide.isBusy()) { }
        //
        pbGrab.ClawOpen();
        //
        //pbGrab.ViperSlideSetPosition(0, 0.5);
        //while (pbGrab.viperSlide.isBusy()) { }
        //pbGrab.AngleViperRotate(20, 0.9);
        //while (pbGrab.viperSlide.isBusy()) { }
        //

        drive.followTrajectory(traj2);
    }
}
