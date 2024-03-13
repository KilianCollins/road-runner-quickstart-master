package org.firstinspires.ftc.teamcode.drive.SemiArea.AUTOV2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
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

public class RedBACKstageCameraSemiAreaRR extends LinearOpMode {

   /// Team6976HWMap drive_train = new Team6976HWMap();

    double cX = 0;
    double cY = 0;
    double width = 0;

    double spikeRight_MIN = 21;
    double spikeRight_MAX = 22.9;

    double spikeMiddle_MIN = 26;
            ;//28.36 , 28.38, 28.
    double spikeMiddle_MAX = 28;

    double spike_OUT_OF_BOUNDS = 35.00; // left // truss side

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

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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
       if ((redBlobDetectionPipeline.getDistance(width) > spikeRight_MIN) && (redBlobDetectionPipeline.getDistance(width) < spikeRight_MAX)) {
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
//spike right
            if (redBlobDetectionPipeline.getDistance(width) > spikeRight_MIN && redBlobDetectionPipeline.getDistance(width) < spikeRight_MAX || gamepad1.a){
                telemetry.addLine("i see the prop its on spike left ");
                telemetry.update();

                Pose2d startPose1 = new Pose2d(14.5, 61,Math.toRadians(90));
                TrajectorySequence middleSpike1 = drive.trajectorySequenceBuilder(startPose1)

                        .lineToConstantHeading(new Vector2d(0, 33))// error acounting is 5.5 --6.5in
                        .lineToConstantHeading(new Vector2d(0, 50))

                        .build();
                drive.followTrajectorySequence(middleSpike1);
                break;

            }
 // spike middle b
            if (redBlobDetectionPipeline.getDistance(width) > spikeMiddle_MIN && redBlobDetectionPipeline.getDistance(width) < spikeMiddle_MAX || gamepad1.b) {
                telemetry.addLine("i see the prop its on spike middle");
                telemetry.update();

                Pose2d startPose2 = new Pose2d(14.5, 61, Math.toRadians(90));
                TrajectorySequence middleSpike2 = drive.trajectorySequenceBuilder(startPose2)

                        .lineToConstantHeading(new Vector2d(14.5, 33))// error acounting is 5.5 --6.5in
                        .lineTo(new Vector2d(14.5,48))
                        .build();

                drive.followTrajectorySequence(middleSpike2);
                break;

            }

//left spike out of bounds y // --1_24_24 7:14pm
             if  (redBlobDetectionPipeline.getDistance(width) > spike_OUT_OF_BOUNDS && redBlobDetectionPipeline.getDistance(width) < 40 || gamepad1.y){
                telemetry.addLine("i dont see it so it must be spike left");
                telemetry.update();

                 Pose2d startPose3 = new Pose2d(14.5, 61,Math.toRadians(90));
                 TrajectorySequence middleSpike3 = drive.trajectorySequenceBuilder(startPose3)

                         .lineToConstantHeading(new Vector2d(0, 31))// error acounting is 5.5 --6.5in
                         .turn(Math.toRadians(90))
                         .lineToConstantHeading(new Vector2d(14,31))

                         .lineToConstantHeading(new Vector2d(0, 33))
                         .build();

                 drive.followTrajectorySequence(middleSpike3);
                 break;

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
            Scalar lowerRed = new Scalar(70, 70, 70);// Scalar(hue, saturation, value of the color(BRIGHTNESS)
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