package org.firstinspires.ftc.teamcode.drive.AutoLeague3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.TeleOp.Team6976HWMap;

@Autonomous(name="blue far no pixels", group="6976")
public class AutoTeam6976ParkingBlueFar extends LinearOpMode{

    Team6976HWMap robot = new Team6976HWMap();
    ElapsedTime Time = new ElapsedTime();
    private double forwards_power = 0.4;
    private DcMotor intakeMotor = null;
    //private double forwards_power = -0.3;
    // double multy = 0.3;

    @Override
    public void runOpMode() {

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.Map(hardwareMap);
        waitForStart();


        Time.reset();
        robot.rightFrontDrive.setPower(forwards_power);//motor 1 //Setting the power to (multy) variable created above
        robot.leftFrontDrive.setPower(forwards_power);//motor 2 fast //Link to Wheel Direction Mapping Below
        robot.rightBackDrive.setPower(forwards_power);//motor 0 //https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
        robot.leftBackDrive.setPower(forwards_power);// motor 3
        sleep(4000); //works keep


        robot.rightFrontDrive.setPower(0); //Sets power to all motors to 0 after desired distance has been reached
        robot.leftFrontDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        sleep(500);
        intakeMotor.setPower(0.6);
        sleep(3000);

//
//    //turn
//        robot.rightFrontDrive.setPower(forwards_power);//motor 1 //Setting the power to (multy) variable created above
//        robot.leftFrontDrive.setPower(forwards_power);//motor 2 fast //Link to Wheel Direction Mapping Below
//        robot.rightBackDrive.setPower(forwards_power);//motor 0 //https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
//        robot.leftBackDrive.setPower(forwards_power);
//        sleep(2000);
//        robot.rightFrontDrive.setPower(0); //Sets power to all motors to 0 after desired distance has been reached
//        robot.leftFrontDrive.setPower(0);
//        robot.rightBackDrive.setPower(0);
//        robot.leftBackDrive.setPower(0);
//        sleep(500);
//
//    //contiue path
//        robot.rightFrontDrive.setPower(forwards_power);//motor 1 //Setting the power to (multy) variable created above
//        robot.leftFrontDrive.setPower(forwards_power);//motor 2 fast //Link to Wheel Direction Mapping Below
//        robot.rightBackDrive.setPower(forwards_power);//motor 0 //https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
//        robot.leftBackDrive.setPower(forwards_power);// motor 3
//        sleep(3000); //works keep

//        while(opModeIsActive() && Time.milliseconds() < 2000 && robot.DriveLeftFront.getCurrentPosition() < tick) { //If Encoder is outputting incorrectly, motor will automatically stop if time in miliseconds has been reached
//            telemetry.addData("Encoder Val", robot.DriveLeftFront.getCurrentPosition()); //Printing Telemtry values to the phone
//            telemetry.update(); //Constantly updates telemetry to the phone
//        }
//
//        robot.rightFrontDrive.setPower(0); //Sets power to all motors to 0 after desired distance has been reached
//        robot.leftFrontDrive.setPower(0);
//        robot.rightBackDrive.setPower(0);
//        robot.leftBackDrive.setPower(0);
        //robot.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Stops and resets the encoder to the 0 value for next use
        //robot.DriveLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}




