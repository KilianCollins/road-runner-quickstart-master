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

package org.firstinspires.ftc.teamcode.drive.LC;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@TeleOp(name="Barath teleOp")
//@Disabled

// the current teleop
public class TeleOpSanjith1020024 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive = null;
    private DcMotor left_Shoulder_Motor, right_Shoulder_Motor, elbow_motor, intake_motor = null;
    private Encoder leftODO,rightODO = null;
    private Servo
            finger_one_servo,
            finger_two_servo,
            wrist_Right_Servo,
            drone_Launcher_servo = null;

   // private int pos = 0;



    private double finger_score = 0.75;
    private int arm_resting_pos = 100;
    private int arm_scoring_pos = 3500;
    private int elbow_scoring_pos = 1900;
    private int elbow_resting_pos = 50;


    @Override
    public void runOpMode() {


/////////////////////////////////////////////////////////////////////////////////////////////////////////
//drive
         leftBackDrive = hardwareMap.get(DcMotor.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");

        left_Shoulder_Motor = hardwareMap.get(DcMotor.class, "leftShoulderMotor");
        right_Shoulder_Motor = hardwareMap.get(DcMotor.class, "rightShoulderMotor");
//jaws


        elbow_motor = hardwareMap.get(DcMotor.class, "elbowMotor");
        intake_motor = hardwareMap.get(DcMotor.class, "intakeMotor");

        finger_one_servo =  hardwareMap.get(Servo.class,"fingerOne");
        finger_two_servo = hardwareMap.get(Servo.class,"fingerTwo");

        wrist_Right_Servo = hardwareMap.get(Servo.class,"rightWrist");

        drone_Launcher_servo = hardwareMap.get(Servo.class, "droneLauncher");


        leftODO = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));
        rightODO = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));

////////////////////////////////////////////////////////////////////////////////////
        //drive
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        intake_motor.setDirection(DcMotorSimple.Direction.FORWARD);

        left_Shoulder_Motor.setDirection(DcMotor.Direction.REVERSE);
        right_Shoulder_Motor.setDirection(DcMotor.Direction.REVERSE);

        elbow_motor.setDirection(DcMotor.Direction.FORWARD);

        wrist_Right_Servo.setDirection(Servo.Direction.FORWARD);
        drone_Launcher_servo.setDirection(Servo.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");



        // telemetry.addData("Front left/Right shoulder pos before start", "%4.2f, %4.2f",left_Shoulder_Motor.getCurrentPosition(), right_Shoulder_Motor.getCurrentPosition());
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        while (opModeIsActive()) {
            double max;


            //game pad 1
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            //drive G1
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;


            while(gamepad1.back){/*works reliably with bool buttons */
                if(gamepad1.y){/*drone launch __0__*/
                    drone_Launcher_servo.scaleRange(0,1);
                    drone_Launcher_servo.setPosition(0);
                }
                if(gamepad1.b){ /*drone hold __1__ */
                    drone_Launcher_servo.scaleRange(0,1);
                    drone_Launcher_servo.setPosition(0.5);
                }
            }
            /* outake last stage fixed 1_6_24 */
            if (gamepad1.right_trigger >= 0.5){ /*finger 1 2 up */
                finger_one_servo.scaleRange(0,1);
                finger_one_servo.setPosition(0.60);
                finger_two_servo.scaleRange(0,1);
                finger_two_servo.setPosition(0.60);
            }
            if(gamepad1.left_trigger >= 0.5){/*finger 1 2 down */
                finger_one_servo.scaleRange(0,1);
                finger_one_servo.setPosition(0);
                finger_two_servo.scaleRange(0,1);
                finger_two_servo.setPosition(0);
            }

            /* intake now fixed organization needed 1_6_24 */


//G2
            // G2
//            if (gamepad2.dpad_left) {/* elbow down */
//                elbow_motor.setTargetPosition(50);
//                elbow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                elbow_motor.setPower(0.5);
//                telemetry.addData("elbow down: ", elbow_motor.getCurrentPosition());
//                telemetry.update();
//
//                if(gamepad2.start){
//                    elbow_motor.setPower(0);
//                }
//                else{
//                    continue;
//                }
//
//            }
//            if (gamepad2.dpad_right) {/* elbow up */
//                elbow_motor.setTargetPosition(600);
//                elbow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                elbow_motor.setPower(0.3);
//                elbow_motor.setTargetPosition(650);
//                elbow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                elbow_motor.setPower(0.3);
//                telemetry.addData("elbow up: ", elbow_motor.getCurrentPosition());
//                telemetry.update();
//                if(gamepad2.start){
//                    elbow_motor.setPower(0);
//                }
//                else{
//                    continue;
//                }
//
//            }

            if(gamepad2.right_bumper){//hang
                elbow_motor.setTargetPosition(1000);
                elbow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elbow_motor.setPower(0.3);
                left_Shoulder_Motor.setTargetPosition(arm_resting_pos);
                left_Shoulder_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                left_Shoulder_Motor.setPower(0.5);
                telemetry.addData("left shoulder up: ", left_Shoulder_Motor.getCurrentPosition());
                telemetry.update();
                right_Shoulder_Motor.setTargetPosition(arm_resting_pos);
                right_Shoulder_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right_Shoulder_Motor.setPower(0.5);
            }
            if(gamepad2.left_bumper){//hang
                left_Shoulder_Motor.setTargetPosition(arm_scoring_pos);
                left_Shoulder_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                left_Shoulder_Motor.setPower(0.5);
                telemetry.addData("left shoulder up: ", left_Shoulder_Motor.getCurrentPosition());
                telemetry.update();
                right_Shoulder_Motor.setTargetPosition(arm_scoring_pos);
                right_Shoulder_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right_Shoulder_Motor.setPower(0.5);

                elbow_motor.setTargetPosition(elbow_scoring_pos);
                elbow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elbow_motor.setPower(0.3);

            }








            if (gamepad2.dpad_down) { /* left right down */
                left_Shoulder_Motor.setTargetPosition(arm_resting_pos);
                left_Shoulder_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                left_Shoulder_Motor.setPower(0.5);
                telemetry.addData("left shoulder down: ", left_Shoulder_Motor.getCurrentPosition());
                telemetry.update();
                right_Shoulder_Motor.setTargetPosition(arm_resting_pos);
                right_Shoulder_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right_Shoulder_Motor.setPower(0.5);
                telemetry.addData("right shoulder down: ", right_Shoulder_Motor.getCurrentPosition());
                telemetry.update();

                elbow_motor.setTargetPosition(elbow_resting_pos);
                elbow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elbow_motor.setPower(0.4);
                telemetry.addData("elbow down: ", elbow_motor.getCurrentPosition());
                telemetry.update();
            }
            if (gamepad2.dpad_up) { /* left right up */
                left_Shoulder_Motor.setTargetPosition(arm_scoring_pos);
                left_Shoulder_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                left_Shoulder_Motor.setPower(0.5);
                telemetry.addData("left shoulder up: ", left_Shoulder_Motor.getCurrentPosition());
                telemetry.update();
                right_Shoulder_Motor.setTargetPosition(arm_scoring_pos);
                right_Shoulder_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right_Shoulder_Motor.setPower(0.5);
                telemetry.addData("right shoulder up: ", right_Shoulder_Motor.getCurrentPosition());
                telemetry.update();

                elbow_motor.setTargetPosition(elbow_scoring_pos);
                elbow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elbow_motor.setPower(0.4);
                telemetry.addData("elbow up: ", elbow_motor.getCurrentPosition());
                telemetry.update();

            }

//            if (gamepad1.x){/*wrist right scoring */
//                wrist_Right_Servo.scaleRange(0,1);
//                wrist_Right_Servo.setPosition(0.85);
//            }
          while(gamepad2.back){
              if (gamepad2.a){
                  wrist_Right_Servo.scaleRange(0, 1);
                  wrist_Right_Servo.setPosition(0.9);
              }else{
                  wrist_Right_Servo.scaleRange(0, 1);
                  wrist_Right_Servo.setPosition(0.7);
              }
            }
            if(gamepad2.y){// chokeing
                wrist_Right_Servo.scaleRange(0,1);
                wrist_Right_Servo.setPosition(0.5);
            }
            if(gamepad2.b){ // resting pos

                wrist_Right_Servo.scaleRange(0,1);
                wrist_Right_Servo.setPosition(0.7);
            }
            if(gamepad2.right_trigger >= 0.5){

                intake_motor.setPower(1);
//                telemetry.addData("left odo: ", leftODO.getCurrentPosition());
//                telemetry.addData("right odo: ", rightODO.getCurrentPosition());
//                telemetry.update();

            }
            else{
                intake_motor.setPower(0);
            }
            if(gamepad2.left_trigger >= 0.5){
                intake_motor.setPower(-1);
            }
            else{
                intake_motor.setPower(0);
            }
//
//G1
/* drone launcher now works reliably 1_6_24 */


            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));


            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        }

    }

}