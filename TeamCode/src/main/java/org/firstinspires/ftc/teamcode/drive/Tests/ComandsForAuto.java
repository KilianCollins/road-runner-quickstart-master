package org.firstinspires.ftc.teamcode.drive.Tests;


import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.AutoLeague3.Team6976HWMap2023;
@Disabled
public class ComandsForAuto {
    ElapsedTime runtime = new ElapsedTime();
    Team6976HWMap2023 driveTrain = new Team6976HWMap2023();

    public void forward(double power, int seconds){
        driveTrain.DriveLeftFront.setPower(power);
        driveTrain.DriveLeftBack.setPower(power);
        driveTrain.DriveRightFront.setPower(power);
        driveTrain.DriveRightBack.setPower(power);
        sleep(seconds);


    }





}
