package org.firstinspires.ftc.teamcode.drive.SemiArea.AUTOV2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(group = "auto")

public class BlueFRONTSTAGECameraSemiAreaRR extends LinearOpMode {

   /// Team6976HWMap drive_train = new Team6976HWMap();

    double cX = 0;
    double cY = 0;
    double width = 0;

    double spikeLeft_MIN = 21;
    double spikeLeft_MAX = 22.9;

    double spikeMiddle_MIN = 26;
            ;//28.36 , 28.38, 28.
    double spikeMiddle_MAX = 28;

    double spike_OUT_OF_BOUNDS = 35.00; // right

    private double wheel_power_multi = 0.4;

    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive, intakeMotor = null;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.66142;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels



    @Override
    public void runOpMode() {

        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        RedBlobDetectionPipeline redBlobDetectionPipeline = new RedBlobDetectionPipeline();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
/////////////////////////////////////////////////////////////////////////////////////////////////////////
        //drive
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");
        //intake
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
////////////////////////////////////////////////////////////////////////////////////
        //drive
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
////////////////////////////////////////////////////////////////////////////////////


   while (opModeInInit()) {
       telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
       telemetry.addData("Distance in Inch", (redBlobDetectionPipeline.getDistance(width)));
       telemetry.update();

//pre initalization confrimation
       if ((redBlobDetectionPipeline.getDistance(width) > spikeLeft_MIN) && (redBlobDetectionPipeline.getDistance(width) < spikeLeft_MAX)) {
           telemetry.addLine("i see the prop its on spike right ");

       } else if ((redBlobDetectionPipeline.getDistance(width) > spikeMiddle_MIN) && (redBlobDetectionPipeline.getDistance(width) < spikeMiddle_MAX)) {
           telemetry.addLine("i see the prop its on spike middle ");

       } else if (redBlobDetectionPipeline.getDistance(width) > spike_OUT_OF_BOUNDS && redBlobDetectionPipeline.getDistance(width) < 40) {
           telemetry.addLine("i dont see it so it must be spike left");

       }

   }


        waitForStart();
            while (opModeIsActive()) {

            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Distance in Inch", (redBlobDetectionPipeline.getDistance(width)));
            telemetry.addData(" actual width val: ", (width));
           // telemetry.addData("  val: ", (width));
                    // spike left for mirrored is out of veiw
//spike left a
            if (redBlobDetectionPipeline.getDistance(width) > spikeLeft_MIN && redBlobDetectionPipeline.getDistance(width) < spikeLeft_MAX || gamepad1.a){
                telemetry.addLine("i see the prop its on spike left ");
                telemetry.update();





                // drives rb backwards 600ms
//                rightFrontDrive.setPower(-wheel_power_multi);
//                leftFrontDrive.setPower(-wheel_power_multi);
//                rightBackDrive.setPower(-wheel_power_multi);
//                leftBackDrive.setPower(-wheel_power_multi);
//                    sleep(900);
//                    rightFrontDrive.setPower(0);
//                    leftFrontDrive.setPower(0);
//                    rightBackDrive.setPower(0);
//                    leftBackDrive.setPower(0);
//                    sleep(200);
//                // strafe (rb) right (align intake with spike left)
//                rightFrontDrive.setPower(-wheel_power_multi);
//                leftFrontDrive.setPower(+wheel_power_multi);
//                rightBackDrive.setPower(+wheel_power_multi);
//                leftBackDrive.setPower(-wheel_power_multi);
//                    sleep(680);
//                    rightFrontDrive.setPower(0);
//                    leftFrontDrive.setPower(0);
//                    rightBackDrive.setPower(0);
//                    leftBackDrive.setPower(0);
//                    sleep(200);
//                // drives rb backwards 800ms
//                rightFrontDrive.setPower(-wheel_power_multi);
//                leftFrontDrive.setPower(-wheel_power_multi);
//                rightBackDrive.setPower(-wheel_power_multi);
//                leftBackDrive.setPower(-wheel_power_multi);
//                    sleep(500);
//                    rightFrontDrive.setPower(0);
//                    leftFrontDrive.setPower(0);
//                    rightBackDrive.setPower(0);
//                    leftBackDrive.setPower(0);
//                    sleep(1000);
//                //outtake purple pix
//                intakeMotor.setPower(0.3);
//                sleep(900);
//                intakeMotor.setPower(0);
//                //drive (rb) backwards again for 600ms (align with door)
//                rightFrontDrive.setPower(-wheel_power_multi);
//                leftFrontDrive.setPower(-wheel_power_multi);
//                rightBackDrive.setPower(-wheel_power_multi);
//                leftBackDrive.setPower(-wheel_power_multi);
//                    sleep(1000);
//                    rightFrontDrive.setPower(0);
//                    leftFrontDrive.setPower(0);
//                    rightBackDrive.setPower(0);
//                    leftBackDrive.setPower(0);
//                    sleep(200);
//                // drive (rb) forwars 200ms (avoid team prop)
////                rightFrontDrive.setPower(+wheel_power_multi);
////                leftFrontDrive.setPower(+wheel_power_multi);
////                rightBackDrive.setPower(+wheel_power_multi);
////                leftBackDrive.setPower(+wheel_power_multi);
////                    sleep(300);
////                    rightFrontDrive.setPower(0);
////                    leftFrontDrive.setPower(0);
////                    rightBackDrive.setPower(0);
////                    leftBackDrive.setPower(0);
////                    sleep(200);
//                // strafe (rb) left for 3 seconds
//                rightFrontDrive.setPower(+wheel_power_multi);
//                leftFrontDrive.setPower(-wheel_power_multi);
//                rightBackDrive.setPower(-wheel_power_multi);
//                leftBackDrive.setPower(+wheel_power_multi);
//                    sleep(3000);
//                    rightFrontDrive.setPower(0);
//                    leftFrontDrive.setPower(0);
//                    rightBackDrive.setPower(0);
//                    leftBackDrive.setPower(0);
//                    sleep(200);
//                // stop rotate (rb) left for 6-500ms// right
//                rightFrontDrive.setPower(-wheel_power_multi);
//                leftFrontDrive.setPower(+wheel_power_multi);
//                rightBackDrive.setPower(-wheel_power_multi);
//                leftBackDrive.setPower(+wheel_power_multi);
//                sleep(400);
//                rightFrontDrive.setPower(0);
//                leftFrontDrive.setPower(0);
//                rightBackDrive.setPower(0);
//                leftBackDrive.setPower(0);
//                sleep(300);
//
//                // strafe (rb) left 2 seconds
//                rightFrontDrive.setPower(+wheel_power_multi);
//                leftFrontDrive.setPower(-wheel_power_multi);
//                rightBackDrive.setPower(-wheel_power_multi);
//                leftBackDrive.setPower(+wheel_power_multi);
//                sleep(4000);
//                rightFrontDrive.setPower(0);
//                leftFrontDrive.setPower(0);
//                rightBackDrive.setPower(0);
//                leftBackDrive.setPower(0);
//                sleep(200);
//                //rotate for error (rb left) into backdrop
//                rightFrontDrive.setPower(+wheel_power_multi);
//                leftFrontDrive.setPower(-wheel_power_multi);
//                rightBackDrive.setPower(+wheel_power_multi);
//                leftBackDrive.setPower(-wheel_power_multi);
//                sleep(600);
//                rightFrontDrive.setPower(0);
//                leftFrontDrive.setPower(0);
//                rightBackDrive.setPower(0);
//                leftBackDrive.setPower(0);
//                sleep(300);
//
//                // outake yellow pixel
//                intakeMotor.setPower(0.3);
//                sleep(1000);
//
//





                //end

//                //straight backwards until  half way to spike (2secs)
//                 rightFrontDrive.setPower(-wheel_power_multi);
//                 leftFrontDrive.setPower(-wheel_power_multi);
//                 rightBackDrive.setPower(-wheel_power_multi);
//                 leftBackDrive.setPower(-wheel_power_multi);
//                     sleep(2000);
//                     rightFrontDrive.setPower(0);
//                     leftFrontDrive.setPower(0);
//                     rightBackDrive.setPower(0);
//                     leftBackDrive.setPower(0);
//                     sleep(600);
//                //rotate (robot right) so intake now perpendicular to left spike
//                 rightFrontDrive.setPower(-wheel_power_multi);
//                 leftFrontDrive.setPower(+wheel_power_multi);
//                 rightBackDrive.setPower(-wheel_power_multi);
//                 leftBackDrive.setPower(+wheel_power_multi);
//                    sleep(2500);
//                     rightFrontDrive.setPower(0);
//                     leftFrontDrive.setPower(0);
//                     rightBackDrive.setPower(0);
//                     leftBackDrive.setPower(0);
//                     sleep(600);
//                //strafe (robot right) until intake centered with left spike (2 secs)
//                 rightFrontDrive.setPower(-wheel_power_multi);
//                 leftFrontDrive.setPower(+wheel_power_multi);
//                 rightBackDrive.setPower(+wheel_power_multi);
//                 leftBackDrive.setPower(-wheel_power_multi);
//                     sleep(2000);
//                     rightFrontDrive.setPower(0);
//                     leftFrontDrive.setPower(0);
//                     rightBackDrive.setPower(0);
//                     leftBackDrive.setPower(0);
//                     sleep(600);
//                // drive forward for like 600ms a few inches
//                 rightFrontDrive.setPower(+wheel_power_multi);
//                 leftFrontDrive.setPower(+wheel_power_multi);
//                 rightBackDrive.setPower(+wheel_power_multi);
//                 leftBackDrive.setPower(+wheel_power_multi);
//                     sleep(600);
//                     rightFrontDrive.setPower(0);
//                     leftFrontDrive.setPower(0);
//                     rightBackDrive.setPower(0);
//                     leftBackDrive.setPower(0);
//                     sleep(600);
//
//                 //spit purple pix
//                 intakeMotor.setPower(-0.4);
//                    sleep(1000);
//
//                //back up for 600ms
//                 rightFrontDrive.setPower(-wheel_power_multi);
//                 leftFrontDrive.setPower(-wheel_power_multi);
//                 rightBackDrive.setPower(-wheel_power_multi);
//                 leftBackDrive.setPower(-wheel_power_multi);
//                     sleep(700);
//                     rightFrontDrive.setPower(0);
//                     leftFrontDrive.setPower(0);
//                     rightBackDrive.setPower(0);
//                     leftBackDrive.setPower(0);
//                     sleep(700);
//                //strafe (robot right) for 2 secs
//                 rightFrontDrive.setPower(-wheel_power_multi);
//                 leftFrontDrive.setPower(+wheel_power_multi);
//                 rightBackDrive.setPower(+wheel_power_multi);
//                 leftBackDrive.setPower(-wheel_power_multi);
//                     sleep(2000);
//                     rightFrontDrive.setPower(0);
//                     leftFrontDrive.setPower(0);
//                     rightBackDrive.setPower(0);
//                     leftBackDrive.setPower(0);
//                     sleep(600);
//                //stop then drive (robot forwards) for 4 secs into backdrop blue
//                 rightFrontDrive.setPower(+wheel_power_multi);
//                 leftFrontDrive.setPower(+wheel_power_multi);
//                 rightBackDrive.setPower(+wheel_power_multi);
//                 leftBackDrive.setPower(+wheel_power_multi);
//                     sleep(4000);
//                     rightFrontDrive.setPower(0);
//                     leftFrontDrive.setPower(0);
//                     rightBackDrive.setPower(0);
//                     leftBackDrive.setPower(0);
//                     sleep(1000);

                //end

//
//
//               //  drive backwards for 4.5 secs
//                rightFrontDrive.setPower(-wheel_power_multi);
//                leftFrontDrive.setPower(-wheel_power_multi);
//                rightBackDrive.setPower(-wheel_power_multi);
//                leftBackDrive.setPower(-wheel_power_multi);
//                    sleep(3000);
//                    rightFrontDrive.setPower(0);
//                    leftFrontDrive.setPower(0);
//                    rightBackDrive.setPower(0);
//                    leftBackDrive.setPower(0);
//                    sleep(600);
//                //strafe (robot left) line intake up with opposite wall facing short edge of right spike (1 sec)
//                rightFrontDrive.setPower(+wheel_power_multi);
//                leftFrontDrive.setPower(-wheel_power_multi);
//                rightBackDrive.setPower(-wheel_power_multi);
//                leftBackDrive.setPower(+wheel_power_multi);
//                    sleep(1000);
//                    rightFrontDrive.setPower(0);
//                    leftFrontDrive.setPower(0);
//                    rightBackDrive.setPower(0);
//                    leftBackDrive.setPower(0);
//                    sleep(600);
//                //drive (robot forwards) for 800ms
//                rightFrontDrive.setPower(+wheel_power_multi);
//                leftFrontDrive.setPower(+wheel_power_multi);
//                rightBackDrive.setPower(+wheel_power_multi);
//                leftBackDrive.setPower(+wheel_power_multi);
//                    sleep(800);
//                    rightFrontDrive.setPower(0);
//                    leftFrontDrive.setPower(0);
//                    rightBackDrive.setPower(0);
//                    leftBackDrive.setPower(0);
//                    sleep(600);
//                // spit purple pix
//                    intakeMotor.setPower(-0.4);
//                        sleep(1000);
//                //drive (robot backwards) for 2 secs
//                rightFrontDrive.setPower(-wheel_power_multi);
//                leftFrontDrive.setPower(-wheel_power_multi);
//                rightBackDrive.setPower(-wheel_power_multi);
//                leftBackDrive.setPower(-wheel_power_multi);
//                    sleep(2000);
//                    rightFrontDrive.setPower(0);
//                    leftFrontDrive.setPower(0);
//                    rightBackDrive.setPower(0);
//                    leftBackDrive.setPower(0);
//                    sleep(600);
//                //strafe (robot right) for 5 seconds into back drop blue
//                rightFrontDrive.setPower(-wheel_power_multi);
//                leftFrontDrive.setPower(+wheel_power_multi);
//                rightBackDrive.setPower(+wheel_power_multi);
//                leftBackDrive.setPower(-wheel_power_multi);
//                    sleep(5000);
//                    rightFrontDrive.setPower(0);
//                    leftFrontDrive.setPower(0);
//                    rightBackDrive.setPower(0);
//                    leftBackDrive.setPower(0);
//                    sleep(600);
//                //spit yellow pix
//                    intakeMotor.setPower(-0.4);
//                        sleep(2000);
                //strafe left
//                robot.DriveRightFront.setPower(+multy);
//                robot.DriveLeftFront.setPower(-multy);
//                robot.DriveRightBack.setPower(-multy);
//                robot.DriveLeftBack.setPower(+multy);
//
//                strafe right
//                robot.DriveRightFront.setPower(-multy);
//                robot.DriveLeftFront.setPower(+multy);
//                robot.DriveRightBack.setPower(+multy);
//                robot.DriveLeftBack.setPower(-multy);

// spike middle
//middle
            }
 // spike middle b
            if (redBlobDetectionPipeline.getDistance(width) > spikeMiddle_MIN && redBlobDetectionPipeline.getDistance(width) < spikeMiddle_MAX || gamepad1.b) {
                telemetry.addLine("i see the prop its on spike middle");
                telemetry.update();



                //intake faces wall
                // drives rb backwards 1900ms
//                    rightFrontDrive.setPower(-wheel_power_multi);
//                    leftFrontDrive.setPower(-wheel_power_multi);
//                    rightBackDrive.setPower(-wheel_power_multi);
//                    leftBackDrive.setPower(-wheel_power_multi);
//                        sleep(1500);
//                        rightFrontDrive.setPower(0);
//                        leftFrontDrive.setPower(0);
//                        rightBackDrive.setPower(0);
//                        leftBackDrive.setPower(0);
//                        sleep(1000);
//                //outtake purple pix
//                intakeMotor.setPower(0.3);
//                sleep(900);
//                //drive (rb) backwards again for 600ms
//                    rightFrontDrive.setPower(-wheel_power_multi);
//                    leftFrontDrive.setPower(-wheel_power_multi);
//                    rightBackDrive.setPower(-wheel_power_multi);
//                    leftBackDrive.setPower(-wheel_power_multi);
//                        sleep(600);
//                        rightFrontDrive.setPower(0);
//                        leftFrontDrive.setPower(0);
//                        rightBackDrive.setPower(0);
//                        leftBackDrive.setPower(0);
//                        sleep(600);
//                // drive (rb) forwars 200ms (avoid team prop)
////                    rightFrontDrive.setPower(+wheel_power_multi);
////                    leftFrontDrive.setPower(+wheel_power_multi);
////                    rightBackDrive.setPower(+wheel_power_multi);
////                        sleep(300);
////                        rightFrontDrive.setPower(0);
////                        leftFrontDrive.setPower(0);
////                        rightBackDrive.setPower(0);
////                        leftBackDrive.setPower(0);
////                        sleep(600);
//                // strafe (rb) left for 2 seconds
//                    rightFrontDrive.setPower(+wheel_power_multi);
//                    leftFrontDrive.setPower(-wheel_power_multi);
//                    rightBackDrive.setPower(-wheel_power_multi);
//                    leftBackDrive.setPower(+wheel_power_multi);
//                        sleep(2000);
//                        rightFrontDrive.setPower(0);
//                        leftFrontDrive.setPower(0);
//                        rightBackDrive.setPower(0);
//                        leftBackDrive.setPower(0);
//                        sleep(200);
//                // stop rotate (rb) left for 6-500ms// (rb right)
//                    rightFrontDrive.setPower(-wheel_power_multi);
//                    leftFrontDrive.setPower(+wheel_power_multi);
//                    rightBackDrive.setPower(-wheel_power_multi);
//                    leftBackDrive.setPower(+wheel_power_multi);
//                        sleep(600);
//                        rightFrontDrive.setPower(0);
//                        leftFrontDrive.setPower(0);
//                        rightBackDrive.setPower(0);
//                        leftBackDrive.setPower(0);
//                        sleep(300);
//                // strafe (rb) left 2 seconds
//                    rightFrontDrive.setPower(+wheel_power_multi);
//                    leftFrontDrive.setPower(-wheel_power_multi);
//                    rightBackDrive.setPower(-wheel_power_multi);
//                    leftBackDrive.setPower(+wheel_power_multi);
//                        sleep(2000);
//                        rightFrontDrive.setPower(0);
//                        leftFrontDrive.setPower(0);
//                        rightBackDrive.setPower(0);
//                        leftBackDrive.setPower(0);
//                        sleep(200);
//                // outake yellow pixel
//                    intakeMotor.setPower(0.3);
//                    sleep(1000);
//                    //end
//
//    // remeber robot is oriented intake towards wall
//
//            //drive straight backwards for 3 seconds (dirves over middle spike)
//                rightFrontDrive.setPower(-wheel_power_multi);
//                leftFrontDrive.setPower(-wheel_power_multi);
//                rightBackDrive.setPower(-wheel_power_multi);
//                leftBackDrive.setPower(-wheel_power_multi);
//                    sleep(1900);
//                    rightFrontDrive.setPower(0);
//                    leftFrontDrive.setPower(0);
//                    rightBackDrive.setPower(0);
//                    leftBackDrive.setPower(0);
//                    sleep(600);
//                //stop with intake 1-2 inchs away from mid spike
//
//                //spit purple pix
//                intakeMotor.setPower(0.5);
//                    sleep(400);
//                //drive straight back for 2 secs (10 inches ish)
//                rightFrontDrive.setPower(-wheel_power_multi);
//                leftFrontDrive.setPower(-wheel_power_multi);
//                rightBackDrive.setPower(-wheel_power_multi);
//                leftBackDrive.setPower(-wheel_power_multi);
//                    sleep(600);
//                    rightFrontDrive.setPower(0);
//                    leftFrontDrive.setPower(0);
//                    rightBackDrive.setPower(0);
//                    leftBackDrive.setPower(0);
//                    sleep(300);
//                // backup to avoid team prop
//                    rightFrontDrive.setPower(+wheel_power_multi);
//                    leftFrontDrive.setPower(+wheel_power_multi);
//                    rightBackDrive.setPower(+wheel_power_multi);
//                    leftBackDrive.setPower(+wheel_power_multi);
//                    sleep(400);
//                    rightFrontDrive.setPower(0);
//                    leftFrontDrive.setPower(0);
//                    rightBackDrive.setPower(0);
//                    leftBackDrive.setPower(0);
//                    sleep(300);
//
//                //strafe right(drivers left) for 4 secs into back drop blue
//                rightFrontDrive.setPower(-wheel_power_multi);
//                leftFrontDrive.setPower(+wheel_power_multi);
//                rightBackDrive.setPower(+wheel_power_multi);
//                leftBackDrive.setPower(-wheel_power_multi);
//                    sleep(4000);
//                    rightFrontDrive.setPower(0);
//                    leftFrontDrive.setPower(0);
//                    rightBackDrive.setPower(0);
//                    leftBackDrive.setPower(0);
//                    sleep(600);
//                //rotate rb left// needs to be right 1/20/24
//                    //moot//was right now left
//                rightFrontDrive.setPower(-wheel_power_multi);
//                leftFrontDrive.setPower(+wheel_power_multi);
//                rightBackDrive.setPower(-wheel_power_multi);
//                leftBackDrive.setPower(+wheel_power_multi);
//                    sleep(550);
//                    rightFrontDrive.setPower(0);
//                    leftFrontDrive.setPower(0);
//                    rightBackDrive.setPower(0);
//                    leftBackDrive.setPower(0);
//                    sleep(300);
//                //strafe right again
//                    rightFrontDrive.setPower(-wheel_power_multi);
//                    leftFrontDrive.setPower(+wheel_power_multi);
//                    rightBackDrive.setPower(+wheel_power_multi);
//                    leftBackDrive.setPower(-wheel_power_multi);
//                    sleep(3000);
//                    rightFrontDrive.setPower(0);
//                    leftFrontDrive.setPower(0);
//                    rightBackDrive.setPower(0);
//                    leftBackDrive.setPower(0);
//                    sleep(600);
//                // spit yellow pix
//                intakeMotor.setPower(0.4);
//                    sleep(1000);

            }
// spike //was left lc needs right//Right && prop not found
//right spike out of bounds y // --1_24_24 7:14pm
             if  (redBlobDetectionPipeline.getDistance(width) > spike_OUT_OF_BOUNDS && redBlobDetectionPipeline.getDistance(width) < 40 || gamepad1.y){
                telemetry.addLine("i dont see it so it must be spike left");
                telemetry.update();

             // drives rb backwards 600ms
//                 rightFrontDrive.setPower(-wheel_power_multi);
//                 leftFrontDrive.setPower(-wheel_power_multi);
//                 rightBackDrive.setPower(-wheel_power_multi);
//                 leftBackDrive.setPower(-wheel_power_multi);
//                     sleep(800);
//                     rightFrontDrive.setPower(0);
//                     leftFrontDrive.setPower(0);
//                     rightBackDrive.setPower(0);
//                     leftBackDrive.setPower(0);
//                     sleep(300);
//             // rotate (rb left) 400ms
//                 rightFrontDrive.setPower(+wheel_power_multi);
//                 leftFrontDrive.setPower(-wheel_power_multi);
//                 rightBackDrive.setPower(+wheel_power_multi);
//                 leftBackDrive.setPower(-wheel_power_multi);
//                     sleep(400);
//                     rightFrontDrive.setPower(0);
//                     leftFrontDrive.setPower(0);
//                     rightBackDrive.setPower(0);
//                     leftBackDrive.setPower(0);
//                     sleep(300);
//                 //spit
//                 intakeMotor.setPower(0.3);
//                    sleep(100);
//                 // rotate right 400ms
//                 rightFrontDrive.setPower(-wheel_power_multi);
//                 leftFrontDrive.setPower(+wheel_power_multi);
//                 rightBackDrive.setPower(-wheel_power_multi);
//                 leftBackDrive.setPower(+wheel_power_multi);
//                     sleep(400);
//                     rightFrontDrive.setPower(0);
//                     leftFrontDrive.setPower(0);
//                     rightBackDrive.setPower(0);
//                     leftBackDrive.setPower(0);
//                     sleep(300);
//                 // drive straight (rb backwards) line up 600ms
//                 rightFrontDrive.setPower(-wheel_power_multi);
//                 leftFrontDrive.setPower(-wheel_power_multi);
//                 rightBackDrive.setPower(-wheel_power_multi);
//                 leftBackDrive.setPower(-wheel_power_multi);
//                     sleep(800);
//                     rightFrontDrive.setPower(0);
//                     leftFrontDrive.setPower(0);
//                     rightBackDrive.setPower(0);
//                     leftBackDrive.setPower(0);
//                     sleep(300);
//                 //strafe (rb left) 3sec
//                 rightFrontDrive.setPower(+wheel_power_multi);
//                 leftFrontDrive.setPower(-wheel_power_multi);
//                 rightBackDrive.setPower(-wheel_power_multi);
//                 leftBackDrive.setPower(+wheel_power_multi);
//                 sleep(800);
//                 //spit
//                 intakeMotor.setPower(0.3);
//                 sleep(1000);
//                 // drive (rb forwards) 1900ms


                 //end



//
//                 // strafe (rb) right (aling intake with spike left)
//                 rightFrontDrive.setPower(-wheel_power_multi);
//                 leftFrontDrive.setPower(+wheel_power_multi);
//                 rightBackDrive.setPower(+wheel_power_multi);
//                 leftBackDrive.setPower(-wheel_power_multi);
//                 sleep(800);
//
//
//                 rightFrontDrive.setPower(0);
//                 leftFrontDrive.setPower(0);
//                 rightBackDrive.setPower(0);
//                 leftBackDrive.setPower(0);
//                 sleep(200);
//                 // drives rb backwards 800ms
//                 rightFrontDrive.setPower(-wheel_power_multi);
//                 leftFrontDrive.setPower(-wheel_power_multi);
//                 rightBackDrive.setPower(-wheel_power_multi);
//                 leftBackDrive.setPower(-wheel_power_multi);
//                 sleep(800);
//                 rightFrontDrive.setPower(0);
//                 leftFrontDrive.setPower(0);
//                 rightBackDrive.setPower(0);
//                 leftBackDrive.setPower(0);
//                 sleep(300);
//                 //outtake purple pix
//                 intakeMotor.setPower(400);
//                 //drive (rb) backwards again for 600ms
//                 rightFrontDrive.setPower(-wheel_power_multi);
//                 leftFrontDrive.setPower(-wheel_power_multi);
//                 rightBackDrive.setPower(-wheel_power_multi);
//                 leftBackDrive.setPower(-wheel_power_multi);
//                 sleep(600);
//                 rightFrontDrive.setPower(0);
//                 leftFrontDrive.setPower(0);
//                 rightBackDrive.setPower(0);
//                 leftBackDrive.setPower(0);
//                 sleep(300);
//                 // drive (rb) forwars 200ms (avoid team prop)
//                 rightFrontDrive.setPower(+wheel_power_multi);
//                 leftFrontDrive.setPower(+wheel_power_multi);
//                 rightBackDrive.setPower(+wheel_power_multi);
//                 leftBackDrive.setPower(+wheel_power_multi);
//                 sleep(300);
//                 rightFrontDrive.setPower(0);
//                 leftFrontDrive.setPower(0);
//                 rightBackDrive.setPower(0);
//                 leftBackDrive.setPower(0);
//                 sleep(300);
//                 // strafe (rb) left for 3 seconds
//                 rightFrontDrive.setPower(+wheel_power_multi);
//                 leftFrontDrive.setPower(-wheel_power_multi);
//                 rightBackDrive.setPower(-wheel_power_multi);
//                 leftBackDrive.setPower(+wheel_power_multi);
//                 sleep(3000);
//                 rightFrontDrive.setPower(0);
//                 leftFrontDrive.setPower(0);
//                 rightBackDrive.setPower(0);
//                 leftBackDrive.setPower(0);
//                 sleep(600);
//                 // stop rotate (rb) left for 6-500ms
//                 rightFrontDrive.setPower(+wheel_power_multi);
//                 leftFrontDrive.setPower(-wheel_power_multi);
//                 rightBackDrive.setPower(+wheel_power_multi);
//                 leftBackDrive.setPower(-wheel_power_multi);
//                 sleep(550);
//                 rightFrontDrive.setPower(0);
//                 leftFrontDrive.setPower(0);
//                 rightBackDrive.setPower(0);
//                 leftBackDrive.setPower(0);
//                 sleep(300);
//                 // strafe (rb) left 2 seconds
//                 rightFrontDrive.setPower(+wheel_power_multi);
//                 leftFrontDrive.setPower(-wheel_power_multi);
//                 rightBackDrive.setPower(-wheel_power_multi);
//                 leftBackDrive.setPower(+wheel_power_multi);
//                 sleep(2000);
//                 rightFrontDrive.setPower(0);
//                 leftFrontDrive.setPower(0);
//                 rightBackDrive.setPower(0);
//                 leftBackDrive.setPower(0);
//                 sleep(600);
//                 // outake yellow pixel
//                 intakeMotor.setPower(0.3);
//                 sleep(1000);

            }
                telemetry.update();


                // The OpenCV pipeline automatically processes frames and handles detection
        }

        // Release resources
        controlHubCam.stopStreaming();
    }

    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "webcam1"), cameraMonitorViewId); //

        controlHubCam.setPipeline(new RedBlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }



    class RedBlobDetectionPipeline extends OpenCvPipeline {


        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat redMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(redMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(240, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 100), 2);
                //Display the Distance                                                                                                  i want green text, read somewhere that cv uses BGR
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 50), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV); // was COLOR_BRG2HSV i chnaged the color space and it worked some how
                                    // for OpenCv HSV Color Space COLOR_RGB2HSV
            /*
                    these scalar values are converted from BGR to HSV (Blue, Green, Red) (Hue,Saturation, Value)

                    why use HSV? HSV checks for values the camera is actually built to see, humans uses the bgr but cameras are better at hsv
                    so using the hsv color space allows you to get a more accurate and consistent detection from the software to the camera
         need to detect more than just red?

         1.go to any hsv color picker website i.e. : https://colorpicker.me/#0081ff
         2.find a color that is pretty close to what you want to detect
         3. enter a range of values that's pretty broad for each,
            this is the HSV vals for a bright alliance marker blue.
                EX: Scalar lowerYellow = new Scalar(100, 100, 100);// hue, saturation, value **below statment is equal to this
                    Scalar upperYellow = new Scalar(180, 255, 255);// color, greyness, brightness ** above statment is equal to this

             */
            Scalar lowerRed = new Scalar(100, 100, 100);// Scalar(hue, saturation, value of the color(BRIGHTNESS)
            Scalar upperRed = new Scalar(180, 255, 255);// in open cv hue == color, saturation == greyness, value == brightness


            Mat redMask = new Mat();
            Core.inRange(hsvFrame, lowerRed, upperRed, redMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_CLOSE, kernel);

            return redMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }

        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }
        public double getDistance(double width){
            double distance = (objectWidthInRealWorldUnits * focalLength) / width;
            //double spikeRight = distance >
            return distance;
        }



    }



}