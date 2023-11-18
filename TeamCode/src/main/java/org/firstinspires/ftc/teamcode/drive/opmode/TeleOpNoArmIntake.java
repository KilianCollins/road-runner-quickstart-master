///* Copyright (c) 2021 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode.drive.opmode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.util.Encoder;
//
///*
// * This file contains an example of a Linear "OpMode".
// * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
// * The names of OpModes appear on the menu of the FTC Driver Station.
// * When a selection is made from the menu, the corresponding OpMode is executed.
// *
// * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
// * This code will work with either a Mecanum-Drive or an X-Drive train.
// * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
// * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
// *
// * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
// *
// * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
// * Each motion axis is controlled by one Joystick axis.
// *
// * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
// * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
// * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
// *
// * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
// * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
// * the direction of all 4 motors (see code below).
// *
// * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
// */
//
//@TeleOp(group="drive")
////@Disabled
//public class TeleOpNoArmIntake extends LinearOpMode {
//
//    // Declare OpMode members for each of the 4 motors.
//    private ElapsedTime runtime = new ElapsedTime();
//   // Servo servo;
/////////////////////////////////////////////////////////////////
//    //private static final double MID = 0.5;
//
/////////////////////////////////////////////////////////////////
//    private DcMotor leftFrontDrive = null;
//    private DcMotor leftBackDrive = null;
//    private DcMotor rightFrontDrive = null;
//    private DcMotor rightBackDrive = null;
////jaws
//    private  DcMotor intakeMotor = null;
//    private Servo right_Intake_Servo_Jaw = null;
//    private Servo left_Intake_Servo_Jaw = null;
////shoulder
//    private DcMotor left_Shoulder_Motor = null;
//    private DcMotor right_Shoulder_Motor = null;
////elbow
//    private Servo left_Elbow_Servo = null;
//    private Servo right_Elbow_Servo = null;
////wrist
//
//    private Encoder leftEncoder = null;
//    private Encoder rightEncoder = null;
//    private Encoder frontEncoder = null;
//    private int  low_shoulder = 200; //537.7 for 312
//    private double shoulder_power = 0.75;
//
//
//
//    @Override
//    public void runOpMode() {
//
//        // Initialize the hardware variables. Note that the strings used here must correspond
//        // to the names assigned during the robot configuration step on the DS or RC devices.
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//        //drive
//        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
//        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftRear");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");
//
//        ///////jaws
//        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
////        left_Intake_Servo_Jaw = hardwareMap.get(Servo.class, "leftIntakeServoJaw");
////        right_Intake_Servo_Jaw = hardwareMap.get(Servo.class, "rightIntakeServoJaw");
//
//        //shoulder
//        left_Shoulder_Motor = hardwareMap.get(DcMotor.class, "leftShoulderMotor");
//        right_Shoulder_Motor = hardwareMap.get(DcMotor.class, "rightShoulderMotor");
//
//        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));// remane odo pods
//        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
//        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
//
//
//        //elbow
////        left_Elbow_Servo = hardwareMap.get(Servo.class, "elbowLeftServo");
////        right_Elbow_Servo = hardwareMap.get(Servo.class, "elbowRightServo");
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//        // ########################################################################################
//        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
//        // ########################################################################################
//        // Most robots need the motors on one side to be reversed to drive forward.
//        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
//        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
//        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
//        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
//        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
//        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward
//        /*
//        dont need to delcare directions for servo  i think
//         */
//////////////////////////////////////////////////////////////////////////////////////
//        //drive
//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
//
//        //jaws
//        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
//
//        //shoulder
//        left_Shoulder_Motor.setDirection(DcMotor.Direction.FORWARD);
//        left_Shoulder_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//
//        right_Shoulder_Motor.setDirection(DcMotor.Direction.FORWARD);
//        right_Shoulder_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        right_Shoulder_Motor.setTargetPosition(low_shoulder);
//        left_Shoulder_Motor.setTargetPosition(low_shoulder);
//
//        right_Shoulder_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        left_Shoulder_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//
//////////////////////////////////////////////////////////////////////////////////////
//
//        // Wait for the game to start (driver presses PLAY)
//        telemetry.addData("Status", "Initialized");
//       // telemetry.addData("Front left/Right shoulder pos before start", "%4.2f, %4.2f",left_Shoulder_Motor.getCurrentPosition(), right_Shoulder_Motor.getCurrentPosition());
//        telemetry.update();
//
//        waitForStart();
//        runtime.reset();
//
//        // run until the end of the match (driver presses STOP)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//        while (opModeIsActive()) {
//            double max;
//
//            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
//            //game pad 1
//            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
//            double lateral =  gamepad1.left_stick_x;
//            double yaw     =  gamepad1.right_stick_x;
//            double intake = gamepad1.left_trigger;
//            double shoulder_test = gamepad2.left_trigger;
//           // boolean reverseIntake = gamepad1.left_bumper;
//            ////////////////////////////////// ////////////////////////////////// //////////////////////////////////
//            //game pad 2
//            //shoulder controls
//            double shoulder_resting_input_val = gamepad2.right_stick_x;
//
//
//           // double shoulder_Scoring_input_val = gamepad2.right_trigger;
//            // elbow
//           // double elbow_resting_pos = gamepad2.right_trigger;
//           // boolean elbow_Scoring_pos = gamepad2.right_bumper;
//
//
//            // Combine the joystick requests for each axis-motion to determine each wheel's power.
//            // Set up a variable for each drive wheel to save the power level for telemetry.
//
//            double leftODO = leftEncoder.getCurrentPosition();
//            double rightODO = rightEncoder.getCurrentPosition();
//            double backODO = frontEncoder.getCurrentPosition();
//
//
//
//
//
//            //drive G1
//            double leftFrontPower  = axial + lateral + yaw;
//            double rightFrontPower = axial - lateral - yaw;
//            double leftBackPower   = axial - lateral + yaw;
//            double rightBackPower  = axial + lateral - yaw;
//
//            double shoulder_power_score_output_pos = shoulder_resting_input_val; // trigger is a flaot
//
//
//            if (gamepad2.b){
//
//                left_Shoulder_Motor.setPower(shoulder_power);
//                right_Shoulder_Motor.setPower(shoulder_power);
//
//
//
//
//            }
//
//            // score G2
//           // double shoulder_power_at_rest_output_pos = t;//should be zero
//          //  double shoulder_power_score_output_pos = shoulder_Scoring_input_val;
//
//
//
//
//           // double  reverse_Intake = -(reverseIntake); //will need later
//           // double  intakereverse = -(intakeReverse);
//     //////////////////////////////////////////////////////////////
//           double limit = shoulder_power;
//
//
//
//            if (limit > 1.0){
//                shoulder_power = shoulder_power * 0.5;
//            }
///*
//needs a intake reverse
// */
//
//
//
//            // Normalize the values so no wheel power exceeds 100%
//            // This ensures that the robot maintains the desired motion.
//            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
//            max = Math.max(max, Math.abs(leftBackPower));
//            max = Math.max(max, Math.abs(rightBackPower));
//
//
//
//            if (max > 1.0) {
//                leftFrontPower  /= max;
//                rightFrontPower /= max;
//                leftBackPower   /= max;
//                rightBackPower  /= max;
//
//                //scrore G2
////                shoulder_power_at_rest_output_pos  /= max;
////                shoulder_power_score_output_pos  /= max;
//                //intakeMove  /= max;
//            }
//
//            // This is test code:
//            //
//            // Uncomment the following code to test your motor directions.
//            // Each button should make the corresponding motor run FORWARD.
//            //   1) First get all the motors to take to correct positions on the robot
//            //      by adjusting your Robot Configuration if necessary.
//            //   2) Then make sure they run in the correct direction by modifying the
//            //      the setDirection() calls above.
//            // Once the correct motors move in the correct direction re-comment this code.
//
//            /*
//            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
//            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
//            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
//            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
//            */
//            //test arm shoulder
////            shoulder_power_at_rest_output_pos  = gamepad1.x ? 0.40 : 0.0;  // X gamepad
////            shoulder_power_score_output_pos  = gamepad1.a ? .40 : 0.0;  // A gamepad
//
//
//            // Send calculated power to wheels
//            leftFrontDrive.setPower(leftFrontPower);
//            rightFrontDrive.setPower(rightFrontPower);
//            leftBackDrive.setPower(leftBackPower);
//            rightBackDrive.setPower(rightBackPower);
//            intakeMotor.setPower(intake);
//
////            left_Shoulder_Motor.setPower(shoulder_power_at_rest_output_pos);
////            right_Shoulder_Motor.setPower(shoulder_power_at_rest_output_pos);
//
//
//
//
//
//            // Show the elapsed game time and wheel power.
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
//            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
//            telemetry.addData("l: , r: , b: ", "%4.2f, %4.2f, %4.2f", leftODO,rightODO,backODO);
//            telemetry.addData("arm pos, l: , r: ", "%4.2f, %4.2f",  left_Shoulder_Motor.getCurrentPosition(), right_Shoulder_Motor.getCurrentPosition());
//
//          //  telemetry.addData("shoulder  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
//            telemetry.update();
//        }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    }
//
//}
