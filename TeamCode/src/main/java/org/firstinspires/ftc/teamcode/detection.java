package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name = "detection", group = "Tests")
//@Disabled
public class detection extends LinearOpMode {

    OpenCvCamera camera;

    public String stack = "";


    @Override
    public void runOpMode() {
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

        while (opModeIsActive()) {
            telemetry.addData("Stack:", stack);
            telemetry.addData("Top Region:", visionPipeline.avgTop);
            telemetry.addData("Bottom Region:", visionPipeline.avgBottom);
            telemetry.update();
        }

        FtcDashboard.getInstance().stopCameraStream();

    }

    // Pipeline class
    class RingPipeline extends OpenCvPipeline {

        // Constants
        final int X_LEFT_T = 360;
        final int X_RIGHT_T = 550;
        final int Y_UP_T = 400;
        final int Y_DOWN_T = 430;

        final int X_LEFT_B = 360;
        final int X_RIGHT_B = 550;
        final int Y_UP_B = 510;
        final int Y_DOWN_B = 540;

        // Working Mat variables
        Mat yCbCrChan2Mat = new Mat();
        Mat bottomRegion = new Mat();
        Mat topRegion = new Mat();

        // Drawing variables
        Scalar GRAY = new Scalar(220, 220, 220); // RGB values for gray.
        Scalar GREEN = new Scalar(0, 255, 0); // RGB values for green.
        Scalar RED = new Scalar(255, 0, 0); // RGB values for red.

        // Variables that will store the results of our pipeline
        public int avgTop;
        public int avgBottom;
        public int threshold = 110;

        // Space which we will annalise data
        public Point TopSquare1 = new Point(X_LEFT_T, Y_UP_T);
        public Point TopSquare2 = new Point(X_RIGHT_T, Y_DOWN_T);

        public Point BottomSquare1 = new Point(X_LEFT_B, Y_UP_B);
        public Point BottomSquare2 = new Point(X_RIGHT_B, Y_DOWN_B);

        // Drawing Points
        int TopSquareX = (int) ((TopSquare1.x + TopSquare2.x) / 2);
        int TopSquareY = (int) ((TopSquare1.y + TopSquare2.y) / 2);

        int BottomSquareX = (int) ((BottomSquare1.x + BottomSquare2.x) / 2);
        int BottomSquareY = (int) ((BottomSquare1.y + BottomSquare2.y) / 2);

        @Override
        public Mat processFrame(Mat input) {

            // Img processing
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            topRegion = yCbCrChan2Mat.submat(new Rect(TopSquare1, TopSquare2));
            avgTop = (int) Core.mean(topRegion).val[0];

            bottomRegion = yCbCrChan2Mat.submat(new Rect(BottomSquare1, BottomSquare2));
            avgBottom = (int) Core.mean(bottomRegion).val[0];

            if (avgTop < threshold) {
                stack = "FOUR";
            } else if (avgTop > threshold && avgBottom < threshold) {
                stack = "ONE";
            } else {
                stack = "ZERO";
            }
            // Top Region
            Imgproc.rectangle(
                    input,
                    TopSquare1,
                    TopSquare2,
                    RED,
                    2
            );

            // Top Region Point
            Imgproc.circle(
                    input,
                    new Point(TopSquareX, TopSquareY),
                    5,
                    RED,
                    2
            );

            // Bottom Region
            Imgproc.rectangle(
                    input,
                    BottomSquare1,
                    BottomSquare2,
                    RED,
                    2
            );

            // Bottom Region Point
            Imgproc.circle(
                    input,
                    new Point(BottomSquareX, BottomSquareY),
                    5,
                    RED,
                    2
            );

            switch (stack) {
                case "FOUR":
                    // Top Region
                    Imgproc.rectangle(
                            input,
                            TopSquare1,
                            TopSquare2,
                            GREEN,
                            2
                    );

                    // Top Region Point
                    Imgproc.circle(
                            input,
                            new Point(TopSquareX, TopSquareY),
                            5,
                            GREEN,
                            2
                    );

                    // Bottom Region
                    Imgproc.rectangle(
                            input,
                            BottomSquare1,
                            BottomSquare2,
                            GREEN,
                            2
                    );

                    // Bottom Region Point
                    Imgproc.circle(
                            input,
                            new Point(BottomSquareX, BottomSquareY),
                            5,
                            GREEN,
                            2
                    );
                    break;
                case "ONE":
                    // Bottom Region
                    Imgproc.rectangle(
                            input,
                            BottomSquare1,
                            BottomSquare2,
                            GREEN,
                            2
                    );

                    // Bottom Region Point
                    Imgproc.circle(
                            input,
                            new Point(BottomSquareX, BottomSquareY),
                            5,
                            GREEN,
                            2
                    );
                    break;
                case "ZERO":
                    break;
            }
            return input;
        }
    }


}