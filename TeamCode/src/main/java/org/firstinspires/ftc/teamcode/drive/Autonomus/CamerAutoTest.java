package org.firstinspires.ftc.teamcode.drive.Autonomus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "OpenCV Testing detects distance from red")
@Disabled
public class CamerAutoTest extends LinearOpMode {

    double cX = 0;
    double cY = 0;
    double width = 0;

    double spikeRight_MIN = 20.0;
    double spikeRight_MAX = 23.0;

    double spikeMiddle_MIN = 29.0;
    double spikeMiddle_MAX = 31.0;

    double spike_OUT_OF_BOUNDS = 35.00;

    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive = null;

    private Encoder leftEncoder, rightEncoder = null;


    //drive
    double drive_forward = 0;
    double drive_backward = 0;





    private double front_left_wheel_power, front_right_wheel_power, back_left_wheel_power, back_right_wheel_power;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.54331;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels



    @Override
    public void runOpMode() {

        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        YellowBlobDetectionPipeline yellowBlobDetectionPipeline = new YellowBlobDetectionPipeline();

        leftFrontDrive  = hardwareMap.get(DcMotor .class, "leftFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
         rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");

         leftEncoder = hardwareMap.get(Encoder.class, "frontLeftODO");
         rightEncoder = hardwareMap.get(Encoder.class, "frontRightODO");
        // backEncoder = hardwareMap.get(Encoder.class, "backODO");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

   while (opModeInInit()) {
       telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
       telemetry.addData("Distance in Inch", (yellowBlobDetectionPipeline.getDistance(width)));
       telemetry.update();

//pre initalization confrimation
       if ((yellowBlobDetectionPipeline.getDistance(width) > spikeRight_MIN) && (yellowBlobDetectionPipeline.getDistance(width) < spikeRight_MAX)) {
           telemetry.addLine("i see the prop its on spike right ");

       } else if ((yellowBlobDetectionPipeline.getDistance(width) > spikeMiddle_MIN) && (yellowBlobDetectionPipeline.getDistance(width) < spikeMiddle_MAX)) {
           telemetry.addLine("i see the prop its on spike middle ");

       } else if (yellowBlobDetectionPipeline.getDistance(width) > spike_OUT_OF_BOUNDS && yellowBlobDetectionPipeline.getDistance(width) < 40) {
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
            telemetry.addData("Distance in Inch", (yellowBlobDetectionPipeline.getDistance(width)));
            telemetry.addData(" actual width val: ", (width));
           // telemetry.addData("  val: ", (width));

// spike right
            if (yellowBlobDetectionPipeline.getDistance(width) > spikeRight_MIN && yellowBlobDetectionPipeline.getDistance(width) < spikeRight_MAX ){
                telemetry.addLine("i see the prop its on spike right ");
                telemetry.update();
            //strafe right







// spike middle
            } if (yellowBlobDetectionPipeline.getDistance(width) > spikeMiddle_MIN && yellowBlobDetectionPipeline.getDistance(width) < spikeMiddle_MAX ) {
                telemetry.addLine("i see the prop its on spike middle 999999999999999");
                telemetry.update();


        //strafe left
                rightBackDrive.setPower(0.5);
                rightFrontDrive.setPower(-0.5);

                leftFrontDrive.setPower(0.5);
                leftBackDrive.setPower(-0.5);


                sleep(1000);
                rightBackDrive.setPower(0);
                leftBackDrive.setPower(0);
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);


            }
// spike left && prop not found
             if  (yellowBlobDetectionPipeline.getDistance(width) > spike_OUT_OF_BOUNDS && yellowBlobDetectionPipeline.getDistance(width) < 40 ){
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


                //
//                rightBackDrive.setPower(-0.5);
//                rightFrontDrive.setPower(0.5);
//                leftFrontDrive.setPower(-0.5);
//                leftBackDrive.setPower(0.5);
//                sleep(1800);
//                rightBackDrive.setPower(0);
//                leftBackDrive.setPower(0);
//                leftFrontDrive.setPower(0);
//                rightFrontDrive.setPower(0);

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

        controlHubCam.setPipeline(new YellowBlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }



    class YellowBlobDetectionPipeline extends OpenCvPipeline {


        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
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
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerYellow = new Scalar(100, 100, 100);// hue, saturation, value **below statment is equal to this
            Scalar upperYellow = new Scalar(180, 255, 255);// color, greyness, brightness ** above statment is equal to this


            Mat yellowMask = new Mat();
            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;
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