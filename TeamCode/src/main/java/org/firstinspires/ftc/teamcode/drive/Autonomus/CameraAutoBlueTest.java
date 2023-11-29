package org.firstinspires.ftc.teamcode.drive.Autonomus;

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

@Autonomous(name = "OpenCV Testing detects distance from BLUE")

public class CameraAutoBlueTest extends LinearOpMode {

    double cX = 0;
    double cY = 0;
    double width = 0;

    double spikeRight_MIN = 20.0;
    double spikeRight_MAX = 23.0;

    double spikeMiddle_MIN = 29.0;
    double spikeMiddle_MAX = 31.0;

    double spike_OUT_OF_BOUNDS = 35.00;

    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive = null;

    private double front_left_wheel_power, front_right_wheel_power, back_left_wheel_power, back_right_wheel_power;

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

        BlueBlobDetectionPipeline blueBlobDetectionPipeline = new BlueBlobDetectionPipeline();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//drive
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");
//jaws
//        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
//        left_Intake_Servo_Jaw = hardwareMap.get(Servo.class, "leftIntakeServoJaws");
//        right_Intake_Servo_Jaw = hardwareMap.get(Servo.class, "rightIntakeServoJaws");

        //2 SERVOS
//shoulder
//        left_Shoulder_Motor = hardwareMap.get(DcMotor.class, "leftShoulderMotor");
//        right_Shoulder_Motor = hardwareMap.get(DcMotor.class, "rightShoulderMotor");
//  //elbow
//         left_Elbow_Servo = hardwareMap.get(Servo.class, "elbowLeftServo");
//         right_Elbow_Servo = hardwareMap.get(Servo.class, "elbowRightServo");
        //wrist
        // wrist_Right_Servo = hardwareMap.get(Servo.class, "wristRightServo");
// //fingers
//         finger_one_servo = hardwareMap.get(Servo.class, "fingerOne");
//         finger_two_servo = hardwareMap.get(Servo.class, "fingerTwo");
//         rocket_Launcher_servo = hardwareMap.get(Servo.class, "droneLauncher");
////odometry
//        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));// remane odo pods
//        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
//        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));


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
        dont need to delcare directions for servo  i think
         */
////////////////////////////////////////////////////////////////////////////////////
        //drive
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

   while (opModeInInit()) {
       telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
       telemetry.addData("Distance in Inch", (blueBlobDetectionPipeline.getDistance(width)));
       telemetry.update();

//pre initalization confrimation
       if ((blueBlobDetectionPipeline.getDistance(width) > spikeRight_MIN) && (blueBlobDetectionPipeline.getDistance(width) < spikeRight_MAX)) {
           telemetry.addLine("i see the prop its on spike right ");

       } else if ((blueBlobDetectionPipeline.getDistance(width) > spikeMiddle_MIN) && (blueBlobDetectionPipeline.getDistance(width) < spikeMiddle_MAX)) {
           telemetry.addLine("i see the prop its on spike middle ");

       } else if (blueBlobDetectionPipeline.getDistance(width) > spike_OUT_OF_BOUNDS && blueBlobDetectionPipeline.getDistance(width) < 40) {
           telemetry.addLine("i dont see it so it must be spike left");

       }
       continue;
   }


        waitForStart();

//        boolean spikeRight = getDistance(width) > 23.00 && getDistance(width) < 25.50;
//        boolean spikeMiddle = getDistance(width) > 25.00 && getDistance(width) < 29.00;
//// spike left is not ealisly inframe so the default will be left
//        boolean spikeLEFT = getDistance(width) > 29;





            while (opModeIsActive()) {

            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Distance in Inch", (blueBlobDetectionPipeline.getDistance(width)));
            telemetry.addData(" actual width val: ", (width));
           // telemetry.addData("  val: ", (width));

// spike right
            if (blueBlobDetectionPipeline.getDistance(width) > spikeRight_MIN && blueBlobDetectionPipeline.getDistance(width) < spikeRight_MAX ){
                telemetry.addLine("i see the prop its on spike right ");
                telemetry.update();

                //front
                //leftFrontDrive.setPower(0.5);
               // rightFrontDrive.setPower(0.5);
                //back
            //strafe right

// spike middle
            } if (blueBlobDetectionPipeline.getDistance(width) > spikeMiddle_MIN && blueBlobDetectionPipeline.getDistance(width) < spikeMiddle_MAX ) {
                telemetry.addLine("i see the prop its on spike middle 999999999999999");
                telemetry.update();
        //strafe left

            }
// spike left && prop not found
             if  (blueBlobDetectionPipeline.getDistance(width) > spike_OUT_OF_BOUNDS && blueBlobDetectionPipeline.getDistance(width) < 40 ){
                telemetry.addLine("i dont see it so it must be spike left");
                telemetry.update();

                 //leftFrontDrive.setPower(-0.5);
               // rightFrontDrive.setPower(-0.5);
                //back
//                rightBackDrive.setPower(0.5);
//                leftBackDrive.setPower(-0.5);
//                sleep(3500);
//                rightBackDrive.setPower(0);
//                leftBackDrive.setPower(0);

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
                hardwareMap.get(WebcamName.class, "webcam1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new BlueBlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }



    class BlueBlobDetectionPipeline extends OpenCvPipeline {


        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat blueMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(blueMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

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
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV); // was COLOR_BRG2HSV i chnaged the color space and it worked some how
                                    // for OpenCv HSV Color Space
            /*
                    these scalar values are converted from BGR to HSV (Blue, Green, Red) (Hue,Saturation, Value)

                    why use HSV? HSV checks for values the camera is actually built to see, humans uses the bgr but cameras are better at hsv
                    so using the hsv color space allows you to get a more accurate and consistent detection from the software to the camera
         need to detect more than just red?

         1.go to any hsv color picker website i.e. : https://colorpicker.me/#0081ff
         2.find a color that is pretty close to what you want to detect
         3. enter a range of values that's pretty broad for each,
            this is the HSV vals for a bright alliance marker red.
                EX: Scalar lowerYellow = new Scalar(100, 100, 100);// hue, saturation, value **below statment is equal to this
                    Scalar upperYellow = new Scalar(180, 255, 255);// color, greyness, brightness ** above statment is equal to this

             */
            Scalar lowerBlue = new Scalar(100, 100, 100);// Scalar(hue, saturation, value of the color(BRIGHTNESS)
            Scalar upperBlue = new Scalar(180, 255, 255);// in open cv hue == color, saturation == greyness, value == brightness


            Mat blueMask = new Mat();
            Core.inRange(hsvFrame, lowerBlue, upperBlue, blueMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_CLOSE, kernel);

            return blueMask;
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