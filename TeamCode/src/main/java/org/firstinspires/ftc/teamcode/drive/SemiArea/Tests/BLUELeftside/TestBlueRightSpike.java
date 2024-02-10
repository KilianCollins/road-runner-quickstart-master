package org.firstinspires.ftc.teamcode.drive.SemiArea.Tests.BLUELeftside;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
DONE 2_9_24


 */
@Config
@Autonomous(name ="LEFT SIDE blue RIGHT spike")
public class TestBlueRightSpike extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(14.5, 61,Math.toRadians(90));
        TrajectorySequence middleSpike = drive.trajectorySequenceBuilder(startPose)

// left blue left spike
                .lineToConstantHeading(new Vector2d(29, 35))// error acounting is 5.5 --6.5in

                .turn(Math.toRadians(-90))//adds input to current heading
                .lineTo(new Vector2d(13,35))
                .lineTo(new Vector2d(38,35))

                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(middleSpike);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
