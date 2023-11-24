package org.firstinspires.ftc.teamcode.drive.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.imgproc.Imgproc;

@Autonomous(name="camerA auto", group="drive ")
@Disabled
public class CameraAuto extends OpMode {
    //class membersaut
    OpenCvCamera webcam1 = null;



    @Override
    public void init(){
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
        int cameraMoniterVeiwId = hardwareMap.appContext.getResources().getIdentifier("cameraMoniterId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName,cameraMoniterVeiwId);

        webcam1.setPipeline(new FindPositonOfPixelOnSpike_Pipline());
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()









        {
            @Override
            public void onOpened() {
                webcam1.startStreaming(640,360, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
                //telemetry.addLine("code failed");
            }
        });

    }

    @Override
    public void loop(){

    }

class  FindPositonOfPixelOnSpike_Pipline extends OpenCvPipeline{
        Mat YCbCr = new Mat();
        Mat left_crop;
        Mat right_crop;
       // Mat middle_crop;
        //red
        double left_average_find;
        double right_average_find;
       // double middle_average_find;
        Mat output = new Mat();
        Scalar rectColor_Red = new Scalar(255.5,0.0,0.0);
      //  Scalar rectColor_Blue = new Scalar(0.0,0.0,255.5);

        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_YCrCb2BGR);
            telemetry.addLine("pipline running ");

//makes rectangle for scanning for object
            Rect left_rect = new Rect(1, 1, 319, 359);
            Rect right_rect = new Rect(30, 1, 319, 359);

// this draws the rectangels on returned display for human trouble shooting
            input.copyTo(output);
            Imgproc.rectangle(output,left_rect, rectColor_Red,2 );
            Imgproc.rectangle(output,right_rect, rectColor_Red, 2);

            left_crop = YCbCr.submat(left_rect);
            right_crop = YCbCr.submat(right_rect);
//extract the chanel red for processing
            Core.extractChannel(left_crop,left_crop,2);
            Core.extractChannel(right_crop,right_crop,2);// channel 2 is red channel,
//gets the average value of the color we are looking for in this case RED
            Scalar left_average = Core.mean(left_crop);
            Scalar right_average = Core.mean(right_crop);


//this is suposed to take the first number in the returend val which dicedes the if statment actions
            left_average_find =  left_average.val[0];
            right_average_find =  right_average.val[0];

            if (left_average_find > right_average_find){
                    telemetry.addLine("prop LEFT");
            }

            else{
                telemetry.addLine("prop RIGHT");
            }

//end of pipeline
            return(output);
        }



    }


}
