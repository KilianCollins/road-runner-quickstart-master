package org.firstinspires.ftc.teamcode.drive.SemiArea;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.TeleOp.Team6976HWMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Autonomous(name="test odo auto ", group="auto")

public class TestODOAuto extends LinearOpMode{


    ElapsedTime Time = new ElapsedTime();
    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive, intakeMotor = null;

    private Encoder leftODO, rightODO = null;
    private double init_left_pos, init_right_pos;

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");

        intakeMotor =   hardwareMap.get(DcMotor.class, "intakeMotor");

        leftODO = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        rightODO = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));


        waitForStart();
       while (opModeIsActive()){



           init_right_pos = rightODO.getCurrentPosition();
           telemetry.addData(String.valueOf(init_left_pos), " left odo");
           telemetry.addData(String.valueOf(init_right_pos), "right odo");

       }





    }
}




