/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Encoder;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is m ade from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(group="drive")
//@Disabled

// the current teleop
public class LeaugeOneTeleOpNoRocket extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
   // Servo servo, private static final double MID = 0.5;

///////////////////////////////////////////////////////////////
    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive = null;
//jaws

    //intakeMotor commented out for League2
//    private  DcMotor intakeMotor = null;
    private Servo left_Intake_Servo_Jaw = null; //right_Intake_Servo_Jaw
//shoulder
    private DcMotor left_Shoulder_Motor, right_Shoulder_Motor, elbow_motor = null;
//elbow
    private Servo left_Elbow_Servo,
        right_Elbow_Servo,
        finger_one_servo,
        finger_two_servo,
        wrist_Right_Servo,
        wrist_Left_Servo,
        rocket_Launcher_servo  = null;
//wrist

    private Encoder leftEncoder = null;
    private Encoder rightEncoder = null;
    private Encoder frontEncoder = null;

    //537.7 for 312
    private int shoulder_scoring = -2900;// which directon for right sholder motor rotate??
    private int sleep_shoulder_pos = 80; // all the wa down

//end game
    private int hanging_pos_down = 1000;

    private int waiting_to_launch = -100;
    private int raise_to_launch = -2500;

//teleop
    private double low_elbow = 1;
    private double shoulder_power = 0.75;

    private double finger_score = 0.75;



    @Override
    public void runOpMode() {



        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//drive
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");
//jaws
        //intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
    //    left_Intake_Servo_Jaw = hardwareMap.get(Servo.class, "leftIntakeServoJaws");
    //    right_Intake_Servo_Jaw = hardwareMap.get(Servo.class, "rightIntakeServoJaws");


//shoulder
        left_Shoulder_Motor = hardwareMap.get(DcMotor.class, "leftShoulderMotor");// this is actually the right motor
        right_Shoulder_Motor = hardwareMap.get(DcMotor.class, "rightShoulderMotor");// left motor config reversed
  //elbow
        elbow_motor = hardwareMap.get(DcMotor.class, "elbowMotor");

 //wrist
     //    wrist_Right_Servo = hardwareMap.get(Servo.class, "wristRightServo");
 //fingers
    //     finger_one_servo = hardwareMap.get(Servo.class, "fingerOne");
    //     finger_two_servo = hardwareMap.get(Servo.class, "fingerTwo");
         rocket_Launcher_servo = hardwareMap.get(Servo.class, "droneLauncher");
//odometry
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));// // NEEW  port
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));



        //elbow
//        left_Elbow_Servo = hardwareMap.get(Servo.class, "elbowLeftServo");
//        right_Elbow_Servo = hardwareMap.get(Servo.class, "elbowRightServo");
/////////////////////////////////////////////////////////////////////////////////////////////////////////
        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward
        /*
        dont need to delcare directions for servo  i think/
        11/11/23 wrong, also need to set "finger_one_servo.scaleRange(min, max);" for reg servos
        11/11/23 servo does not accept negative doubles,
        11/11/23 when using multiple zeros you must find the, preset 0 and one

         */
////////////////////////////////////////////////////////////////////////////////////
        //drive
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        //jaws
     //   left_Intake_Servo_Jaw.setDirection(Servo.Direction.REVERSE);
       // intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        //shoulder
        left_Shoulder_Motor.setDirection(DcMotor.Direction.FORWARD);
        right_Shoulder_Motor.setDirection(DcMotor.Direction.FORWARD);
        //elbow
        left_Elbow_Servo.setDirection(Servo.Direction.REVERSE);
        //wrist
     //   wrist_Right_Servo.setDirection(Servo.Direction.FORWARD);
        //fingers
     //   finger_two_servo.setDirection(Servo.Direction.FORWARD);
     //   finger_one_servo.setDirection(Servo.Direction.FORWARD);
        //dl
        rocket_Launcher_servo.setDirection(Servo.Direction.REVERSE);



        telemetry.addData("Status", "Initialized");


       // telemetry.addData("Front left/Right shoulder pos before start", "%4.2f, %4.2f",left_Shoulder_Motor.getCurrentPosition(), right_Shoulder_Motor.getCurrentPosition());
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            //game pad 1
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
           // double intake  = gamepad1.left_trigger;


           // double shoulder_test = gamepad2.left_trigger;
           // boolean reverseIntake = gamepad1.left_bumper;
            ////////////////////////////////// ////////////////////////////////// //////////////////////////////////
            //game pad 2
            //shoulder controls
           // double shoulder_resting_input_val = gamepad2.right_stick_x;


           // double shoulder_Scoring_input_val = gamepad2.right_trigger;
            // elbow
           // double elbow_resting_pos = gamepad2.right_trigger;
           // boolean elbow_Scoring_pos = gamepad2.right_bumper;


            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.

            double leftODO = leftEncoder.getCurrentPosition();
            double rightODO = rightEncoder.getCurrentPosition();
            double backODO = frontEncoder.getCurrentPosition();







            //drive G1
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;


//            if(gamepad2.x){
//                right_Intake_Servo_Jaw.setPosition(0.75);
//            }
//            if(gamepad2.b){
//                right_Intake_Servo_Jaw.setPosition(0);
//            }
//
//
//            if(gamepad2.y){
//                right_Intake_Servo_Jaw.setPosition(0.3);
//
//            }



//            if (gamepad2.a) {
//                left_Shoulder_Motor.setTargetPosition(low_shoulder);
//                left_Shoulder_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                left_Shoulder_Motor.setPower(0.5);
//
//                right_Shoulder_Motor.setTargetPosition(low_shoulder);
//                right_Shoulder_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                right_Shoulder_Motor.setPower(0.5);
//
//            }
//            if(gamepad2.x){
//
//                right_Shoulder_Motor.setTargetPosition(low_shoulder);
//                right_Shoulder_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                right_Shoulder_Motor.setPower(0.5);
//            }
            double l = 0;
            double r = 0;
//            if(gamepad2.b){
//
//                l = l + 0.1;
//
//                r = r+ 0;
//
//                left_Elbow_Servo.setPosition(1);
//                //right_Elbow_Servo.setPosition(r);
//
//    //            telemetry.addData("elbow pos: ", left_Elbow_Servo.getPosition())
//            }

//            if(gamepad2.y){
//                finger_one_servo.setPosition(finger_score);
//                finger_two_servo.setPosition(finger_score);
//            }

/*
needs intake reverse button 10/8/23 --kilian
 */
            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));



            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;


            }


//          if(gamepad2.a){
//              right_Intake_Servo_Jaw.scaleRange(0.1, 0.7);
//              right_Intake_Servo_Jaw.setPosition(0.3);
//
//          }
////////////////////////// G1
       // if (gamepad1.y){// intake up
       //     left_Intake_Servo_Jaw.scaleRange(0, 1);// min, max "min has to be larger than max"
       //     left_Intake_Servo_Jaw.setPosition(0.7);

       //     right_Intake_Servo_Jaw.scaleRange(0, 1);
       //     right_Intake_Servo_Jaw.setPosition(0.7);

        //    intakeMotor.setPower(0);

      //  }
      //  if (gamepad1.a){// intake down
         //   left_Intake_Servo_Jaw.scaleRange(0, 1);// min, max "min has to be larger than max"
         //   left_Intake_Servo_Jaw.setPosition(0);

        //    right_Intake_Servo_Jaw.scaleRange(0, 1);
        //    right_Intake_Servo_Jaw.setPosition(0);

        //    intakeMotor.setPower(0.9);

       // }
        /*
        need to define what, in code, each button does, not just up and out or down, more description
        w3DFFGBN  BV FDD
         */

//        if (gamepad2.dpad_right){// outtake
//            finger_one_servo.scaleRange(0,1);
//            finger_one_servo.setPosition(0);
//
//            finger_two_servo.scaleRange(0,1);
//            finger_two_servo.setPosition(0);
//
//        }
//            if (gamepad2.dpad_left){ /// intake
//                finger_one_servo.scaleRange(0,1);
//                finger_one_servo.setPosition(0.4);
//
//                finger_two_servo.scaleRange(0,1);
//                finger_two_servo.setPosition(0.4);
//
//            }

//            if (gamepad2.dpad_left){
//                right_Shoulder_Motor.setTargetPosition(2500);
//                right_Shoulder_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                right_Shoulder_Motor.setPower(0.5);
//            }
//            if (gamepad2.dpad_right){
//                rocket_Launcher_servo.setDirection(Servo.Direction.REVERSE);
//                rocket_Launcher_servo.scaleRange(0,1);
//                rocket_Launcher_servo.setPosition(1);
//
//            }
//////////////////////// G2
//////g2 elbow //////////////
        if (gamepad2.y){ //135 past shoulder
            // up 1
            left_Elbow_Servo.scaleRange(0, 1);
            left_Elbow_Servo.setPosition(0.82);
        }

        if (gamepad2.x){// down 0, towards chassie also
            left_Elbow_Servo.scaleRange(0, 1);
            left_Elbow_Servo.setPosition(0);
        }
 ////////TESTING SHOULDER//////////////////TESTING SHOULDER//////////////////TESTING SHOULDER///////////////////////
//            if (gamepad2.dpad_up){// left up
//                left_Shoulder_Motor.setTargetPosition(-2900);
//                left_Shoulder_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                left_Shoulder_Motor.setPower(0.5);
//
//            }
//
//            if (gamepad2.dpad_down){ //right up

//            right_Shoulder_Motor.setTargetPosition(shoulder_scoring);
//            right_Shoulder_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            right_Shoulder_Motor.setPower(0.5);
//
//            }
//
//            if (gamepad2.dpad_left){// left sleep pos
//                left_Shoulder_Motor.setTargetPosition(80);
//                left_Shoulder_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                left_Shoulder_Motor.setPower(0.5);
//
//            }
//
//            if (gamepad2.dpad_right){// right sleep pos
//                right_Shoulder_Motor.setTargetPosition(80);
//                right_Shoulder_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                right_Shoulder_Motor.setPower(0.5);
//
//            }

///////TESTING SHOULDER//////\/\/\//////////TESTING SHOULDER///////\/\/\///////////TESTING SHOULDER/////////\/\/\/\/\////////////TESTING SHOULDER/\/\/\
////g2 Shoulder motor right
            if (gamepad2.dpad_up){// up
                left_Shoulder_Motor.setTargetPosition(shoulder_scoring);
                left_Shoulder_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                left_Shoulder_Motor.setPower(0.5);

                right_Shoulder_Motor.setTargetPosition(shoulder_scoring);
                right_Shoulder_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right_Shoulder_Motor.setPower(0.5);

            }
//
            if (gamepad2.dpad_down){ // down
                left_Shoulder_Motor.setTargetPosition(sleep_shoulder_pos);
                left_Shoulder_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                left_Shoulder_Motor.setPower(0.5);

                right_Shoulder_Motor.setTargetPosition(80);
                right_Shoulder_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right_Shoulder_Motor.setPower(0.5);

            }
////////////G1//////////////////////////////////////////////////////

            if (gamepad1.y){
            rocket_Launcher_servo.scaleRange(0, 1);
            rocket_Launcher_servo.setPosition(0.5);

        }
        if(gamepad1.a){
            rocket_Launcher_servo.scaleRange(0, 1);
            rocket_Launcher_servo.setPosition(0);

        }
//////dr laucnehr aimer

//            if (gamepad2.dpad_right){//whole arm down to chasie crunch for hanging 12/1/23
//                left_Shoulder_Motor.setTargetPosition(hanging_pos_down);
//                left_Shoulder_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                left_Shoulder_Motor.setPower(0.5);
//
//            }

       if (gamepad1.b){ // raise the drone laucnher
           right_Shoulder_Motor.setTargetPosition(waiting_to_launch); // 800
           right_Shoulder_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           right_Shoulder_Motor.setPower(0.5);

       }
       if (gamepad1.x){// sleepmode drone launcher
           right_Shoulder_Motor.setTargetPosition(raise_to_launch); //-1500
           right_Shoulder_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           right_Shoulder_Motor.setPower(0.5);

       }

//////////////////////// G!

 //       if(gamepad2.a){// up 1 wrist spit pix out
      //      wrist_Right_Servo.scaleRange(0,1);
      //      wrist_Right_Servo.setPosition(0.3);


//        }

 //       if (gamepad2.b){// down 0 wrist accept pix
      //      wrist_Right_Servo.scaleRange(0,1);
      //      wrist_Right_Servo.setPosition(0);

        //}




//        float leftTriggerData = gamepad2.left_trigger;
//
//        if (leftTriggerData !){
//
//
//        }
////////////////////////////////////////////// G2 fingers
//        if (gamepad2.left_trigger > 0){
//            finger_one_servo.scaleRange(0,1);
//            finger_one_servo.setPosition(1);
//
//        }
//        if (gamepad2.right_trigger > 0){
//            finger_two_servo.scaleRange(0,1);
//            finger_two_servo.setPosition(0);
//
//        }











//        if(gamepad2.x){
//            left_Intake_Servo_Jaw.
//
//        }










            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

//            if(gamepad2.x){//1
//                left_Elbow_Servo.scaleRange(0, 1);
//                left_Elbow_Servo.setPosition(1);
//
//            }
//            if (gamepad2.b){//0
//                left_Elbow_Servo.scaleRange(0,1);
//                left_Elbow_Servo.setPosition(0);
//
//            }
//            if (gamepad2.y){//0
//                right_Elbow_Servo.scaleRange(0,1);
//                right_Elbow_Servo.setPosition(1);
//
//
//            }
//            if (gamepad2.a){//1
//                right_Elbow_Servo.scaleRange(0,1);
//                right_Elbow_Servo.setPosition(0);
//            }




//
//            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
//            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
//            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
//            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad



            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            //intakeMotor.setPower(intake);











             // Show the elapsed game time and wheel power.
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
//            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
//            telemetry.addData("l: , r: , b: ", "%4.2f, %4.2f, %4.2f", leftEncoder.getCurrentPosition(),rightEncoder.getCurrentPosition(),frontEncoder.getCurrentPosition());
//            telemetry.addData("arm pos, l: , r: ", "%4.2f, %4.2f",  left_Shoulder_Motor.getCurrentPosition(), right_Shoulder_Motor.getCurrentPosition());
//
//            telemetry.addData("shoulder  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
//            telemetry.update();
        }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }

}
