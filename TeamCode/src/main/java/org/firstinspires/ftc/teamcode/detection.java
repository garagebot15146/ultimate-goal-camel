package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp (name = "detection", group = "Tests")
//@Disabled
public class detection extends LinearOpMode {

    OpenCvCamera camera;

    // CONSTANTS

    final int X_LEFT = 350;
    final int X_RIGHT = 750;
    final int Y_UP = 650;
    final int Y_MIDDLE = 350;
    final int Y_DOWN = 100;



    @Override
    public void runOpMode()
    {
        // Camera Init
        int cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );

        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        // Loading pipeline
        RingPipeline visionPipeline = new RingPipeline();
        camera.setPipeline(visionPipeline);

        // Start Streaming
        camera.openCameraDeviceAsync(() -> camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT));

        // Stream Camera
        FtcDashboard.getInstance().startCameraStream(camera, 30);


        waitForStart();

        while (opModeIsActive())
        {
            // Get data from the pipeline and output it to the telemetry. This are the variables you are going to work with.
            telemetry.addData("Ring 1:",visionPipeline.ring1); // Will return 0 if there is 1 ring, otherwise 1
            telemetry.addData("Ring 4:",visionPipeline.ring4); // Will return 0 if there is 4 rings, otherwise 1
            telemetry.update();
        }

        FtcDashboard.getInstance().stopCameraStream();

    }

    // Pipeline class
    class RingPipeline extends OpenCvPipeline {


        // Working Mat variables
        Mat YCrCb = new Mat(); // This will store the whole YCrCb channel
        Mat Cb = new Mat(); // This will store the Cb Channel (part from YCrCb)
        Mat tholdMat = new Mat(); // This will store the threshold

        // Drawing variables
        Scalar GRAY = new Scalar(220, 220, 220); // RGB values for gray.
        Scalar GREEN = new Scalar(0, 255, 0); // RGB values for green.

        // Variables that will store the results of our pipeline
        public int ring1;
        public int ring4;

        // Space which we will annalise data
        public Point BigSquare1 = new Point(X_LEFT, Y_UP);
        public Point BigSquare2 = new Point(X_RIGHT, Y_DOWN);

        public Point SmallSquare1 = new Point(X_LEFT, Y_MIDDLE);
        public Point SmallSquare2 = new Point(X_RIGHT, Y_DOWN);

        @Override
        public Mat processFrame(Mat input) {

            // Img processing
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
            Imgproc.threshold(Cb, tholdMat, 150, 255, Imgproc.THRESH_BINARY_INV);

            // Drawing Points
            int BigSquarePointX = (int) ((BigSquare1.x + BigSquare2.x) / 2);
            int BigSquarePointY = (int) ((BigSquare1.y + SmallSquare1.y) / 2);

            int SmallSquarePointX = (int) ((SmallSquare1.x + SmallSquare2.x) / 2);
            int SmallSquarePointY = (int) ((SmallSquare1.y + SmallSquare2.y) / 2);

            // Point BigSquarePoint = new Point((int)((BigSquare1.x + BigSqare2.x) / 2),(int)((BigSquare1.y + SmallSquare1.y) / 2));
            // Point SmallSquarePoint = new Point((int)((SmallSquare1.x + SmallSquare2.x) / 2),(int)((SmallSquare1.y + SmallSquare2.y) / 2));

            double[] bigSquarePointValues = tholdMat.get(BigSquarePointY, BigSquarePointX);
            double[] smallSquarePointValues = tholdMat.get(SmallSquarePointY, SmallSquarePointX);

            ring4 = (int) bigSquarePointValues[0];
            ring1 = (int) smallSquarePointValues[0];

            // Big Square
            Imgproc.rectangle(
                    input,
                    BigSquare1,
                    BigSquare2,
                    GRAY,
                    20
            );

            // Small Square
            Imgproc.rectangle(
                    input,
                    SmallSquare1,
                    SmallSquare2,
                    GRAY,
                    20
            );

            // Big Square Point
            Imgproc.circle(
                    input,
                    new Point(BigSquarePointX, BigSquarePointY),
                    50,
                    GRAY,
                    10
            );

            // Small Square Point
            Imgproc.circle(
                    input,
                    new Point(SmallSquarePointX, SmallSquarePointY),
                    50,
                    GRAY,
                    10
            );

            // Change colors if the pipeline detected something

            if (ring1 == 0 && ring4 == 0) {
                Imgproc.rectangle(
                        input,
                        BigSquare1,
                        BigSquare2,
                        GREEN,
                        10
                );
                Imgproc.circle(
                        input,
                        new Point(BigSquarePointX, BigSquarePointY),
                        50,
                        GREEN,
                        10
                );
            }
            if (ring1 == 0) {
                Imgproc.rectangle(
                        input,
                        SmallSquare1,
                        SmallSquare2,
                        GREEN,
                        10
                );
                Imgproc.circle(
                        input,
                        new Point(SmallSquarePointX, SmallSquarePointY),
                        50,
                        GREEN,
                        10
                );
            }

            return input;
        }
    }


}