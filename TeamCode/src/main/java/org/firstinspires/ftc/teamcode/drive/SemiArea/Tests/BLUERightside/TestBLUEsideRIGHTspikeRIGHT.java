package org.firstinspires.ftc.teamcode.drive.SemiArea.Tests.BLUERightside;

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



 */
@Config
@Autonomous(name ="side BLUE Right spike RIGHT")
public class TestBLUEsideRIGHTspikeRIGHT extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-37, 61, Math.toRadians(90));
        TrajectorySequence middleSpike = drive.trajectorySequenceBuilder(startPose)

// left blue left spike


                .lineToConstantHeading(new Vector2d(-53, 33))// error acounting is 5.5 --6.5in
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(-36, 33))// error acounting is 5.5 --6.5in
                .lineToConstantHeading(new Vector2d(-53, 33))// error acounting is 5.5 --6.5in

//              end


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
