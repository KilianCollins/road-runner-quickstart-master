package org.firstinspires.ftc.teamcode.drive.AutoLeague3;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Autonomous(name = "path test 12.19.23 back and forth stop")
@Disabled
public class Auto_Test_path_hardcode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive = null;

    private Encoder leftEncoder, rightEncoder = null;
    double inital_left_pos, inital_right_pos;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

//        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
//
//                .forward(10.0)
//                .back(9.0)
//                .strafeLeft(20.0)
//                .build();
////                .strafeRight(10)
////                .forward(5) /// the above trajec is not physicaly possible du to continutiy error,
        // remeber the hole in the graph

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(5)
                .build();

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);


        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        // sleep for x is built in
    }


    }



