package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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


@Autonomous(name = "Auto", group = "Autonomous")
//@Disabled
public class auto extends LinearOpMode {

    //Runtime
    private ElapsedTime runtime = new ElapsedTime();
    double turretTime = 0;

    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.77953;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    //Vision
    OpenCvCamera camera;
    public String stack = "";

    //Road Runner
    static SampleMecanumDrive drive;

    //Initialize IMU
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double angleOffset = 0;

    //Turret
    double turretTicks; //Keeps track of motor's ticks ONLY during this session
    double turretAngleTargetDegrees; //Tells the turret what local angle to turn towards
    double turretAngleErrorDegrees; //Tells how far off the turret's local angle is from its local target
    double turretGlobalAngleTargetDegrees; //Sets the global angle target regardless of robot orientation

    //Display on Dashboard
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        //Hardware map
        drive = new SampleMecanumDrive(hardwareMap);
        //Set starting position
        Pose2d startPose = new Pose2d(-63, -33, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        Pose2d myPose = drive.getPoseEstimate();




        //Kicker
        drive.kicker.setPosition(drive.kickerInit);
        //Claw
        drive.wobbleGoalArm.setPosition(drive.wobbleUp);
        drive.wobblePincher.setPosition(drive.wobblePinchClose);
        //Reset turret's ticks
        drive.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Suspension
        // drive.frontSuspendServo.setPosition(drive.frontSuspendUp);
        //Lift
        drive.lift.setPosition(drive.liftUp);



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

//PATH CONSTANTS

//
////CASE A INIT START
//        Trajectory trajectoryA1 = drive.trajectoryBuilder(new Pose2d())
//                .strafeTo(new Vector2d(powerShotX, powerShotStrafe))
//                .build();
////CASE A INIT END
//
////CASE B INIT START
//        Trajectory trajectoryB1 = drive.trajectoryBuilder(new Pose2d())
//                .strafeTo(new Vector2d(powerShotX, powerShotStrafe))
//                .build();
////CASE B INIT END

//CASE C INIT START
        Trajectory trajectoryC1 = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(-43, -13))
                .build();
        Trajectory trajectoryC2 = drive.trajectoryBuilder(trajectoryC1.end())
                .strafeTo(new Vector2d(-5, 0))
                .build();
//CASE C INIT END

        telemetry.addData("Status", "Pipeline Initializing");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        telemetry.update();

        //Suspension Down
        drive.frontSuspendServo.setPosition(drive.frontSuspendDown);

        String height = "FOUR";
//        String height = stack;
        telemetry.addData("Ring Stack", stack);
        telemetry.update();
        FtcDashboard.getInstance().stopCameraStream();

        switch (height) {
            case "ZERO":
//                //Turn on shooter. Move to Power Shot
//                shooter.setPower(1);
//                drive.followTrajectory(trajectoryA1);
//                sleep(1000);
//                //Take shot 1
//                kick(1);
//                //Turn to left powershot
//                drive.turn(Math.toRadians(firstAngle));
//                //Take shot 2
//                kick(1);
//                //Turn to right powershot
//                drive.turn(Math.toRadians(secondAngle));
//                //Take shot 3
//                kick(1);
//                shooter.setPower(0);
//                //Reset heading to 0
//                drive.turn(Math.toRadians(thirdAngle));
//                //Drive to zone
//                drive.followTrajectory(trajectoryA2);
//                //Drop off wobble goal
//                armAngle(-100, 0.5);
//                clawServo.setPosition(clawOpen);
//                sleep(500);
//                armAngle(120, 0.4);
//                clawServo.setPosition(clawClose);
//                sleep(300);
//                //Prepare to pickup second wobble goal
//                drive.followTrajectory(trajectoryA3);
//                armAngle(-90, 0.3);
//                clawServo.setPosition(clawOpen);
//                sleep(200);
//                //Collect second wobble goal
//                drive.followTrajectory(trajectoryA4);
//                clawServo.setPosition(clawClose);
//                sleep(500);
//                //Move to zone again
//                drive.followTrajectory(trajectoryA5);
//                //Drop off wobble goal 2
//                clawServo.setPosition(clawOpen);
//                sleep(500);
//                armAngle(120, 0.4);
//                clawServo.setPosition(clawClose);
//                //Park on tape
//                drive.followTrajectory(trajectoryA6);
//                drive.followTrajectory(trajectoryA7);
//                sleep(2000);
//                telemetry.addData("Path A", "Complete");
//                telemetry.update();
                break;
            case "ONE":
//                //Turn on shooter. Move to Power Shot
//                shooter.setPower(1);
//                drive.followTrajectory(trajectoryB1);
//                sleep(1000);
//                //Take shot 1
//                kick(1);
//                //Turn to left powershot
//                drive.turn(Math.toRadians(firstAngle));
//                //Take shot 2
//                kick(1);
//                //Turn to right powershot
//                drive.turn(Math.toRadians(secondAngle));
//                //Take shot 3
//                kick(1);
//                shooter.setPower(0);
//                //Reset heading to 0
//                drive.turn(Math.toRadians(thirdAngle));
//                //Basket down. Drive to zone
//                basketDown();
//                drive.followTrajectory(trajectoryB2);
//                //Drop off wobble goal
//                armAngle(-100, 0.5);
//                clawServo.setPosition(clawOpen);
//                sleep(500);
//                armAngle(120, 0.4);
//                clawServo.setPosition(clawClose);
//                sleep(300);
//                //Prepare to intake one ring
//                drive.followTrajectory(trajectoryB3);
//                shooter.setPower(1);
//                shootFlap.setPosition(0.1311);
//                //Prepare to pickup second wobble goal
//                drive.followTrajectory(trajectoryB4);
//                sleep(700);
//                armAngle(-90, 0.3);
//                clawServo.setPosition(clawOpen);
//                sleep(200);
//                //Collect second wobble goal
//                drive.followTrajectory(trajectoryB5);
//                clawServo.setPosition(clawClose);
//                backIntake.setPower(0);
//                basketUp();
//                sleep(500);
//                //Drive to align with goal
//                drive.followTrajectory(trajectoryB6);
//                drive.followTrajectory(trajectoryB65);
//                //Shoot ring
//                drive.followTrajectory(trajectoryB7);
//                //Move to zone again
//                drive.followTrajectory(trajectoryB8);
//                //Drop off wobble goal 2, retract kicker
//                clawServo.setPosition(clawOpen);
//                kicker.setPosition(kickerInit);
//                sleep(500);
//                armAngle(120, 0.4);
//                clawServo.setPosition(clawClose);
//                //Park on tape
//                drive.followTrajectory(trajectoryB9);
//                sleep(2000);
//                telemetry.addData("Path B", "Complete");
//                telemetry.update();
                break;
            case "FOUR":
                //Put the stuff on dashboard
                dashboard = FtcDashboard.getInstance();
                dashboard.setTelemetryTransmissionInterval(25);

//                drive.shooter.setPower(1);
//                drive.followTrajectory(trajectoryC1);
//                drive.followTrajectory(trajectoryC2);
                telemetry.update();
//                sleep(2000);
                shootPowerShots();
                drive.shooter.setPower(0);
                sleep(10000);

                drive.update();
                telemetry.addData("x pose", myPose.getX());
                telemetry.addData("y pose", myPose.getY());
                telemetry.addData("heading", myPose.getHeading());
                telemetry.update();
                sleep(10000);

//                drive.followTrajectory(trajectoryC3);
//                drive.followTrajectory(trajectoryC4);
//                drive.followTrajectory(trajectoryC45);
//                drive.followTrajectory(trajectoryC48);
//                drive.followTrajectory(trajectoryC5);
//                drive.followTrajectory(trajectoryC6);
//                drive.followTrajectory(trajectoryC7);
//                drive.followTrajectory(trajectoryC8);
                telemetry.addData("Path C", "Complete");
                telemetry.update();
                break;
        }


        //Transfer Position
        drive.update();
        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    public void shootGoal(int shotCount) {
        //Set flap angle
        drive.leftFlap.setPosition(drive.leftFlapGoal);
        drive.rightFlap.setPosition(drive.rightFlapGoal);

        //Updating Position
        drive.update();
        Pose2d myPose = drive.getPoseEstimate();

        //Fix odometry's heading range
        double odoHeading;
        if (Math.toDegrees(myPose.getHeading()) > 180) {
            odoHeading = Math.toDegrees(myPose.getHeading()) - 360;
        } else {
            odoHeading = Math.toDegrees(myPose.getHeading());
        }

        //Set target
        turretGlobalAngleTargetDegrees = 90 - Math.toDegrees(Math.atan((72 - myPose.getX()) / (-36 - myPose.getY())));
        turretAngleTargetDegrees = turretGlobalAngleTargetDegrees - odoHeading + drive.turretAngleOffset + (0.05 * (myPose.getY() + 36));

        //Limit the range of motion for the turret
        if (turretAngleTargetDegrees > 8.5) {
            turretAngleTargetDegrees = 8.5;
        } else if (turretAngleTargetDegrees < -38) {
            turretAngleTargetDegrees = -38;
        }

        //Set timer
        turretTime = runtime.milliseconds();

        while (turretTime + 500 > runtime.milliseconds() && !isStopRequested()) {
            //Update the turret's ticks
            turretTicks = drive.turretMotor.getCurrentPosition();

            //Calculate angle error
            turretAngleErrorDegrees = (turretTicks * -0.32360) - turretAngleTargetDegrees;

            //Apply power for correction
            if (turretAngleErrorDegrees > 0) {
                if (turretAngleErrorDegrees > 8) {
                    //Don't go too fast if turret is far from target
                    drive.turretMotor.setPower(0.2);
                } else {
                    drive.turretMotor.setPower(Math.pow(0.1 * turretAngleErrorDegrees - 0.5848, 3) + 0.2);
                }
            } else if (turretAngleErrorDegrees < 0) {
                if (turretAngleErrorDegrees < -8) {
                    //Don't go too fast if turret is far from target
                    drive.turretMotor.setPower(-0.2);
                } else {
                    drive.turretMotor.setPower(Math.pow(0.1 * turretAngleErrorDegrees + 0.5848, 3) - 0.2);
                }
            }
        }

        //Kick
        for (int i = 0; i < shotCount; i++) {
            sleep(150);
            drive.kicker.setPosition(drive.kickerTo);
            sleep(150);
            drive.kicker.setPosition(drive.kickerInit);
        }
    }

    public void shootPowerShots() {
        //Set flap angle
        drive.leftFlap.setPosition(drive.leftFlapPowerShot);
        drive.rightFlap.setPosition(drive.rightFlapPowerShot);

        //Updating Position
        drive.update();
        Pose2d myPose = drive.getPoseEstimate();

        //Put the stuff on dashboard
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        //Fix odometry's heading range
        double odoHeading;
        if (Math.toDegrees(myPose.getHeading()) > 180) {
            odoHeading = Math.toDegrees(myPose.getHeading()) - 360;
        } else {
            odoHeading = Math.toDegrees(myPose.getHeading());
        }

        //Set target (Left)
        turretGlobalAngleTargetDegrees = 90 - Math.toDegrees(Math.atan( (72 - myPose.getX()) / (-5 - myPose.getY()) ) );
        turretAngleTargetDegrees = turretGlobalAngleTargetDegrees - odoHeading + drive.turretAngleOffset + (0.1 * (myPose.getY() + 36));

        telemetry.addData("global target", turretGlobalAngleTargetDegrees);
        telemetry.addData("local target", turretAngleTargetDegrees);
        telemetry.update();

        //Limit the range of motion for the turret
        if (turretAngleTargetDegrees > 8.5) {
            turretAngleTargetDegrees = 8.5;
        } else if (turretAngleTargetDegrees < -38) {
            turretAngleTargetDegrees = -38;
        }

        //Set timer
        turretTime = runtime.milliseconds();

        while (turretTime + 500 > runtime.milliseconds() && !isStopRequested()) {
            //Update the turret's ticks
            turretTicks = drive.turretMotor.getCurrentPosition();

            //Calculate angle error
            turretAngleErrorDegrees = (turretTicks * -0.32360) - turretAngleTargetDegrees;

            //Apply power for correction
            if (turretAngleErrorDegrees > 0) {
                if (turretAngleErrorDegrees > 8) {
                    //Don't go too fast if turret is far from target
                    drive.turretMotor.setPower(0.2);
                } else {
                    drive.turretMotor.setPower(Math.pow(0.1 * turretAngleErrorDegrees - 0.5848, 3) + 0.2);
                }
            } else if (turretAngleErrorDegrees < 0) {
                if (turretAngleErrorDegrees < -8) {
                    //Don't go too fast if turret is far from target
                    drive.turretMotor.setPower(-0.2);
                } else {
                    drive.turretMotor.setPower(Math.pow(0.1 * turretAngleErrorDegrees + 0.5848, 3) - 0.2);
                }
            }
        }

        //Kick
        drive.kicker.setPosition(drive.kickerTo);
        sleep(150);
        drive.kicker.setPosition(drive.kickerInit);

        //////////////////

        //Set target (Middle)
        turretGlobalAngleTargetDegrees = 90 - Math.toDegrees(Math.atan((72 - myPose.getX()) / (-12 - myPose.getY())));
        turretAngleTargetDegrees = turretGlobalAngleTargetDegrees - odoHeading + drive.turretAngleOffset + (0.1 * (myPose.getY() + 36));

        //Limit the range of motion for the turret
        if (turretAngleTargetDegrees > 8.5) {
            turretAngleTargetDegrees = 8.5;
        } else if (turretAngleTargetDegrees < -38) {
            turretAngleTargetDegrees = -38;
        }

        //Refresh time
        turretTime = runtime.milliseconds();

        //turretTime + 200 for control video
        while (turretTime + 350 > runtime.milliseconds() && !isStopRequested()) {
            //Update the turret's ticks
            turretTicks = drive.turretMotor.getCurrentPosition();

            //Calculate angle error
            turretAngleErrorDegrees = (turretTicks * -0.32360) - turretAngleTargetDegrees;

            //Apply power for correction
            if (turretAngleErrorDegrees > 0) {
                if (turretAngleErrorDegrees > 8) {
                    //Don't go too fast if turret is far from target
                    drive.turretMotor.setPower(0.2);
                } else {
                    drive.turretMotor.setPower(Math.pow(0.1 * turretAngleErrorDegrees - 0.5848, 3) + 0.2);
                }
            } else if (turretAngleErrorDegrees < 0) {
                if (turretAngleErrorDegrees < -8) {
                    //Don't go too fast if turret is far from target
                    drive.turretMotor.setPower(-0.2);
                } else {
                    drive.turretMotor.setPower(Math.pow(0.1 * turretAngleErrorDegrees + 0.5848, 3) - 0.2);
                }
            }
        }

        //Kick
        drive.kicker.setPosition(drive.kickerTo);
        sleep(150);
        drive.kicker.setPosition(drive.kickerInit);

        //////////////////

        //Set target (Right)
        turretGlobalAngleTargetDegrees = 90 - Math.toDegrees(Math.atan((72 - myPose.getX()) / (-19.5 - myPose.getY())));
        turretAngleTargetDegrees = turretGlobalAngleTargetDegrees - odoHeading + drive.turretAngleOffset + (0.1 * (myPose.getY() + 36));

        //Limit the range of motion for the turret
        if (turretAngleTargetDegrees > 8.5) {
            turretAngleTargetDegrees = 8.5;
        } else if (turretAngleTargetDegrees < -38) {
            turretAngleTargetDegrees = -38;
        }

        //Refresh time
        turretTime = runtime.milliseconds();

        while (turretTime + 350 > runtime.milliseconds() && !isStopRequested()) {
            //Update the turret's ticks
            turretTicks = drive.turretMotor.getCurrentPosition();

            //Calculate angle error
            turretAngleErrorDegrees = (turretTicks * -0.32360) - turretAngleTargetDegrees;

            //Apply power for correction
            if (turretAngleErrorDegrees > 0) {
                if (turretAngleErrorDegrees > 8) {
                    //Don't go too fast if turret is far from target
                    drive.turretMotor.setPower(0.2);
                } else {
                    drive.turretMotor.setPower(Math.pow(0.1 * turretAngleErrorDegrees - 0.5848, 3) + 0.2);
                }
            } else if (turretAngleErrorDegrees < 0) {
                if (turretAngleErrorDegrees < -8) {
                    //Don't go too fast if turret is far from target
                    drive.turretMotor.setPower(-0.2);
                } else {
                    drive.turretMotor.setPower(Math.pow(0.1 * turretAngleErrorDegrees + 0.5848, 3) - 0.2);
                }
            }
        }

        //Kick
        drive.kicker.setPosition(drive.kickerTo);
        sleep(150);
        drive.kicker.setPosition(drive.kickerInit);
    }

    // Pipeline class
    class RingPipeline extends OpenCvPipeline {

        // Constants
        final int X_LEFT_T = 360;
        final int X_RIGHT_T = 550;
        final int Y_UP_T = 430;
        final int Y_DOWN_T = 460;

        final int X_LEFT_B = 360;
        final int X_RIGHT_B = 550;
        final int Y_UP_B = 530;
        final int Y_DOWN_B = 560;

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

            return input;
        }
    }

}