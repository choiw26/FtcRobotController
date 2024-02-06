package org.firstinspires.ftc.teamcode.CompetitionCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.CompetitionCode.Mechanisms.PbClaw;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;


@Autonomous
public class AutonomousOpModeRedFront extends LinearOpMode{
    @Override
    public void runOpMode() {
        //Define all the Programming Boards
        PbClaw pbGrab = new PbClaw();
        pbGrab.init(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        ElapsedTime runtime = new ElapsedTime();




        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(12,-63, Math.toRadians(90)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(54,-36, Math.toRadians(0)), Math.toRadians(0))
                //

                //
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(54,-36, Math.toRadians(0)), Math.toRadians(0))
                .strafeRight(20)
                .build();



        waitForStart();
        if(isStopRequested()) return;

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
        //pbGrab.ViperSlideSetPosition(15, 0.5);
        //while (pbGrab.viperSlide.isBusy()) { }
        //
        pbGrab.ClawOpen();
        pbGrab.ClawClose();
        //
        //pbGrab.ViperSlideSetPosition(0, 0.5);
        //while (pbGrab.viperSlide.isBusy()) { }
        //pbGrab.AngleViperRotate(20, 0.9);
        //while (pbGrab.viperSlide.isBusy()) { }
        //

        drive.followTrajectory(traj2);

    }




}