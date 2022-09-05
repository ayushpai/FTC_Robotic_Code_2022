
package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

//@Config
@TeleOp
public class RedVisionTest extends LinearOpMode
{
    OpenCvCamera webcam;
    SkystoneDeterminationPipeline pipeline;
    public static int analysis;
    FtcDashboard        dashboard                       = FtcDashboard.getInstance();




    @Override
    public void runOpMode()
    {

        //FtcDashboard.start(hardwareMap.appContext);
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
       // webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "lWebcam"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        pipeline = new SkystoneDeterminationPipeline();

        webcam.setPipeline(pipeline);

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */

        webcam.openCameraDevice();
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                CameraStreamSource cameraStreamSource = webcam;
                webcam.openCameraDevice();
                FtcDashboard.getInstance().startCameraStream(cameraStreamSource, 60);

            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {

            telemetry.addData("Analysis 1", pipeline.getAnalysis1());
            telemetry.addData("Analysis 2", pipeline.getAnalysis2());
            telemetry.addData("Analysis 3", pipeline.getAnalysis3());
            telemetry.addData("Position", pipeline.getPosition());


            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(100);
        }
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {

        /*
         * An enum to define the skystone position
         */
        String position;


        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(RobotConstants.r1XRed,RobotConstants.r1YRed);
        static Point REGION1_TOPLEFT_ANCHOR_POINT2 = new Point(RobotConstants.r2XRed,RobotConstants.r2YRed);
        static Point REGION1_TOPLEFT_ANCHOR_POINT3 = new Point(RobotConstants.r3XRed,RobotConstants.r3YRed);
        static final int REGION_WIDTH = 50;
        static final int REGION_HEIGHT = 55;

        final int FOUR_RING_THRESHOLD = 135;
        final int ONE_RING_THRESHOLD = 129;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        static final Scalar BLUE2 = new Scalar(0, 0, 255);
        static final Scalar GREEN2 = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */


        static final int REGION_WIDTH2 = 50;
        static final int REGION_HEIGHT2 = 55;


        Point region1_pointA2 = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT2.x,
                REGION1_TOPLEFT_ANCHOR_POINT2.y);
        Point region1_pointB2 = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT2.x + REGION_WIDTH2,
                REGION1_TOPLEFT_ANCHOR_POINT2.y + REGION_HEIGHT2);



        static final int REGION_WIDTH3 = 50;
        static final int REGION_HEIGHT3 = 55;

        Point region1_pointA3 = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT3.x,
                REGION1_TOPLEFT_ANCHOR_POINT3.y);
        Point region1_pointB3 = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT3.x + REGION_WIDTH3,
                REGION1_TOPLEFT_ANCHOR_POINT3.y + REGION_HEIGHT3);

        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb, region3_Cb;

        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;
        int avg2;
        int avg3;

        // Volatile since accessed by OpMode thread w/o synchronization

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region1_pointA2, region1_pointB2));
            region3_Cb = Cb.submat(new Rect(region1_pointA3, region1_pointB3));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);


            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];





            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA2, // First point which defines the rectangle
                    region1_pointB2, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA3, // First point which defines the rectangle
                    region1_pointB3, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines



            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA2, // First point which defines the rectangle
                    region1_pointB2, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA3, // First point which defines the rectangle
                    region1_pointB3, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill



            boolean test = true;


            return input;
        }

        public int getAnalysis1()
        {
            return avg1;
        }
        public int getAnalysis2()
        {
            return avg2;
        }
        public int getAnalysis3()
        {
            return avg3;
        }
        public String getPosition(){
            if(avg1 < avg2 && avg1 < avg3 && avg1 <= RobotConstants.redVisionThreshold){
                return "left";
            }
            else if(avg2 < avg1 && avg2 < avg3 && avg2 <= RobotConstants.redVisionThreshold){
                return "center";
            }
            else if(avg3 < avg1 && avg3 < avg2 && avg3 <= RobotConstants.redVisionThreshold){
                return "right";
            }
            else{
                return "right";
            }
        }
    }
}