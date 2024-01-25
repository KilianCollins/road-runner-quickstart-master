//package org.firstinspires.ftc.teamcode.drive.Autonomus;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.MatOfPoint;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.core.Size;
//import org.opencv.imgproc.Imgproc;
//import org.opencv.imgproc.Moments;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//import java.util.ArrayList;
//import java.util.List;
//
//@Autonomous(name = "OpenCV Testing detects distance from red")
//
//public class CameraAutoRedXpositonTest extends LinearOpMode {
//
//    double cX = 0;
//    double cLeft = 0; // for spike positons
//
//    double cMiddle = 0;// for spike postions
//    double cY = 0;
//    double width = 0;
//
//    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive = null;
//
//    private double front_left_wheel_power, front_right_wheel_power, back_left_wheel_power, back_right_wheel_power;
//
//    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
//    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
//    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution
//
//    // Calculate the distance using the formula
//    public static final double objectWidthInRealWorldUnits = 4.00;  // Replace with the actual width of the object in real-world units
//    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels
//
//
//
//    @Override
//    public void runOpMode() {
//
//        initOpenCV();
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//        FtcDashboard.getInstance().startCameraStream(controlHubCam, 60);// maxfps was 30 11/24/23
//
//       // leftFrontDrive  = hardwareMap.get(DcMotor .class, "leftFront");
//        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftRear");
//       // rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
//         rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");
//
//       // leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//       // rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
//
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            YellowBlobDetectionPipeline yellowblobpipeline = new YellowBlobDetectionPipeline();
//
//            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
//            telemetry.addData("Distance in Inch", (getDistance(width)));
//            telemetry.update();
//
//
//            if (getDistance(width) > 4.26 && getDistance(width) < 5.00 ){
//                telemetry.addLine("i see it");
//                //front
//                //leftFrontDrive.setPower(0.5);
//               // rightFrontDrive.setPower(0.5);
//                //back
////                rightBackDrive.setPower(0.5);
////                leftBackDrive.setPower(0.5);
//            }else{
//                telemetry.addLine("i dont see it ");
//                //leftFrontDrive.setPower(-0.5);
//               // rightFrontDrive.setPower(-0.5);
//                //back
////                rightBackDrive.setPower(-0.5);
////                leftBackDrive.setPower(-0.5);
//
//            }
//
//
//
//
//
//
//
//
//            // The OpenCV pipeline automatically processes frames and handles detection
//        }
//
//        // Release resources
//        controlHubCam.stopStreaming();
//    }
//
//    private void initOpenCV() {
//
//        // Create an instance of the camera
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//
//        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
//        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
//                hardwareMap.get(WebcamName.class, "webcam1"), cameraMonitorViewId);
//
//        controlHubCam.setPipeline(new YellowBlobDetectionPipeline());
//
//        controlHubCam.openCameraDevice();
//        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
//    }
////
////    class BlueBlobDetectionPipeline extends  OpenCvPipeline{
////
////        public Mat processFrame(Mat input){
////
////            List<MatOfPoint> contours = new ArrayList<>();
////            Mat hierarchy = new Mat();
////            // Preprocess the frame to detect yellow regions
////            Mat blueMask = preprocessFrame(input);
////
////            // Find contours of the detected yellow regions
////
////            Imgproc.findContours(blueMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
////
////            // Find the largest yellow contour (blob)
////            MatOfPoint largestContour = findLargestContour(contours);
////
////
////
////
////
////        }
////
////
////    }
////
////
//
//
//
//
//
//
//    class YellowBlobDetectionPipeline extends OpenCvPipeline {
//
//
//        @Override
//        public Mat processFrame(Mat input) {
//            // Preprocess the frame to detect yellow regions
//            Mat yellowMask = preprocessFrame(input);
//
//            // Find contours of the detected yellow regions
//            List<MatOfPoint> contours = new ArrayList<>();
//            Mat hierarchy = new Mat();
//            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//
//            // Find the largest yellow contour (blob)
//            MatOfPoint largestContour = findLargestContour(contours);
//
//            if (largestContour != null) {
//                // Draw a red outline around the largest detected object
//                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
//                // Calculate the width of the bounding box
//                width = calculateWidth(largestContour);
//
//                // Display the width next to the label
//                String widthLabel = "Width: " + (int) width + " pixels";
//                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
//                //Display the Distance
//                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
//                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
//                // Calculate the centroid of the largest contour
//                Moments moments = Imgproc.moments(largestContour);
//                cX = moments.get_m10() / moments.get_m00();
//                cY = moments.get_m01() / moments.get_m00();
//
//                // Draw a dot at the centroid
//                String label = "(" + (int) cX + ", " + (int) cY + ")";
//                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
//                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);
//
//            }
//
//            return input;
//        }
//
//        private Mat preprocessFrame(Mat frame) {
//            Mat hsvFrame = new Mat();
//            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);
//
//            Scalar lowerYellow = new Scalar(100, 100, 100);
//            Scalar upperYellow = new Scalar(180, 255, 255);
//
//
//            Mat yellowMask = new Mat();
//            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);
//
//            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
//            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
//            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);
//
//            return yellowMask;
//        }
//
//        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
//            double maxArea = 0;
//            MatOfPoint largestContour = null;
//
//            for (MatOfPoint contour : contours) {
//                double area = Imgproc.contourArea(contour);
//                if (area > maxArea) {
//                    maxArea = area;
//                    largestContour = contour;
//                }
//            }
//
//            return largestContour;
//        }
//        private double calculateWidth(MatOfPoint contour) {
//            Rect boundingRect = Imgproc.boundingRect(contour);
//            return boundingRect.width;
//        }
//
//    }
//    public static double getDistance(double width){
//        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
//        return distance;
//    }
//
//
//}