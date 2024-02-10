package org.firstinspires.ftc.teamcode.drive.SemiArea.Tests.REDRightSide;

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
@Autonomous(name ="side REd Right spike oooooooooLEFT")
public class TestREDsideRIGHTspikeLEFT extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(14.5, 61,Math.toRadians(90));
        TrajectorySequence middleSpike = drive.trajectorySequenceBuilder(startPose)


                .lineToConstantHeading(new Vector2d(0, 31))// error acounting is 5.5 --6.5in
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(14,31))



                .lineToConstantHeading(new Vector2d(0, 33))// error acounting is 5.5 --6.5in



                // red backsage left spike




//                .lineToConstantHeading(new Vector2d(29, 35))// error acounting is 5.5 --6.5in
//                .lineTo(new Vector2d(29,59))
//                .lineTo(new Vector2d(59,55))

//                .lineTo(new Vector2d(50, 33))//needs more towards bckstg
//                .lineToLinearHeading(new Pose2d(40, 38, Math.toRadians(100)))


//                .lineTo(new Vector2d(26,38))
//                .turn(Math.toRadians(-100)) // adds x deggres to start pos heading
//                .lineTo(new Vector2d(15,38))
//                .lineTo(new Vector2d(58,35))




//                .lineTo(new Vector2d(13.5,33.5))// dont use
//                .lineTo(new Vector2d(13.5,43.5))// dont use
//                .turn(Math.toRadians(45)) // Turns 45 degrees counter-clockwise
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
