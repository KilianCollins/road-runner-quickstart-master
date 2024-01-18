package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Team6976HWMap {


    public DcMotor rightBackDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;

    // public DcMotor Elevator = null;
    // public Servo Intake = null;

    //public NormalizedColorSensor ColorSensor = null;

    HardwareMap hwMap =  null;

    public void Map(HardwareMap hardwareMap)
    {
        hwMap = hardwareMap;
        leftFrontDrive = hwMap.get(DcMotor.class,"leftFront");
        rightFrontDrive = hwMap.get(DcMotor.class,"rightFront");
        leftBackDrive = hwMap.get(DcMotor.class,"leftRear");
        rightBackDrive = hwMap.get(DcMotor.class,"rightRear");




        //leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        //leftBackDrive = hardwareMap.get(DcMotor.class, "leftRear");
        //rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        //rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


//
//        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
//        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //    Elevator = hwMap.get(DcMotor.class, "Elevator");
        //   Elevator.setDirection(DcMotorSimple.Direction.REVERSE);
        //   Elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //    Elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //    Intake = hwMap.get(Servo.class, "Intake");

        //   ColorSensor = hwMap.get(NormalizedColorSensor.class, "Sensor");

    }

}


