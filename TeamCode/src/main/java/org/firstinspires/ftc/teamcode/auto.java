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
        Pose2d startPose = new Pose2d(-64, -16, Math.toRadians(0));
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
        //Lift
        drive.lift.setPosition(drive.liftUp);
        //Ringblocker
        drive.ringBlocker.setPosition(drive.ringBlockUp);


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
        Trajectory trajectoryA1 = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(-43, -13))
                .build();
        Trajectory trajectoryA2 = drive.trajectoryBuilder(trajectoryA1.end())
                .strafeTo(new Vector2d(-5, -4))
                .build();
        Trajectory trajectoryA3 = drive.trajectoryBuilder(trajectoryA2.end())
                .lineToLinearHeading(new Pose2d(32, -10, Math.toRadians(90)))
                .build();
        Trajectory trajectoryA4 = drive.trajectoryBuilder(trajectoryA3.end().plus(new Pose2d(0, 0, Math.toRadians(-90))), false)
                .lineToLinearHeading(new Pose2d(-25, -37, Math.toRadians(0)))
                .build();
        Trajectory trajectoryA5 = drive.trajectoryBuilder(trajectoryA4.end())
                .strafeTo(new Vector2d(-2, -35))
                .build();
        Trajectory trajectoryA6 = drive.trajectoryBuilder(trajectoryA5.end())
                .lineToLinearHeading(new Pose2d(-58, -20, Math.toRadians(90)))
                .build();
        Trajectory trajectoryA7 = drive.trajectoryBuilder(trajectoryA6.end())
                .strafeTo(new Vector2d(-53.7, -34.3))
                .build();
        Trajectory trajectoryA8 = drive.trajectoryBuilder(trajectoryA7.end().plus(new Pose2d(0, 0, Math.toRadians(90))), false)
                .strafeTo(new Vector2d(18, -37))
                .build();
        Trajectory trajectoryA9 = drive.trajectoryBuilder(trajectoryA8.end())
                .strafeTo(new Vector2d(12, -34))
                .build();
////CASE A INIT END
//
////CASE B INIT START
        Trajectory trajectoryB1 = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(-6, -16))
                .build();
        Trajectory trajectoryB2 = drive.trajectoryBuilder(trajectoryB1.end())
                .lineToLinearHeading(new Pose2d(45, -23))
                .build();
        Trajectory trajectoryB3 = drive.trajectoryBuilder(trajectoryB2.end())
                .strafeTo(new Vector2d(-25, -34))
                .build();
        Trajectory trajectoryB4 = drive.trajectoryBuilder(trajectoryB3.end())
                .strafeTo(new Vector2d(-12, -34))
                .build();
        Trajectory trajectoryB5 = drive.trajectoryBuilder(trajectoryB4.end())
                .strafeTo(new Vector2d(-43, -26))
                .build();
        Trajectory trajectoryB6 = drive.trajectoryBuilder(trajectoryB5.end())
                .strafeTo(new Vector2d(-41.5, -32.6))
                .build();
        Trajectory trajectoryB7 = drive.trajectoryBuilder(trajectoryB6.end())
                .strafeTo(new Vector2d(40, -14))
                .build();
        Trajectory trajectoryB8 = drive.trajectoryBuilder(trajectoryB7.end())
                .strafeTo(new Vector2d(7, -11))
                .build();
////CASE B INIT END

//CASE C INIT START

        Trajectory trajectoryC1 = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(-6, -12))
                .build();
        Trajectory trajectoryC2 = drive.trajectoryBuilder(trajectoryC1.end())
                .strafeTo(new Vector2d(59, -45))
                .build();
        Trajectory trajectoryC3 = drive.trajectoryBuilder(trajectoryC2.end())
                .strafeTo(new Vector2d(-6, -31))
                .build();
        Trajectory trajectoryC4 = drive.trajectoryBuilder(trajectoryC3.end())
                .strafeTo(new Vector2d(-18, -31))
                .build();
        Trajectory trajectoryC5 = drive.trajectoryBuilder(trajectoryC4.end())
                .strafeTo(new Vector2d(-22, -31))
                .build();
        Trajectory trajectoryC6 = drive.trajectoryBuilder(trajectoryC5.end())
                .strafeTo(new Vector2d(-12, -31))
                .build();
        Trajectory trajectoryC7 = drive.trajectoryBuilder(trajectoryC6.end())
                .strafeTo(new Vector2d(-30, -31))
                .build();
        Trajectory trajectoryC8 = drive.trajectoryBuilder(trajectoryC7.end())
                .strafeTo(new Vector2d(-37, -31))
                .build();
        Trajectory trajectoryC9 = drive.trajectoryBuilder(trajectoryC8.end())
                .strafeTo(new Vector2d(-14, -31))
                .build();
        Trajectory trajectoryC10 = drive.trajectoryBuilder(trajectoryC9.end())
                .strafeTo(new Vector2d(-42, -21))
                .build();
        Trajectory trajectoryC105 = drive.trajectoryBuilder(trajectoryC10.end())
                .strafeTo(new Vector2d(-42, -30.5))
                .build();
        Trajectory trajectoryC11 = drive.trajectoryBuilder(trajectoryC105.end())
                .strafeTo(new Vector2d(61, -40))
                .build();
        Trajectory trajectoryC12 = drive.trajectoryBuilder(trajectoryC11.end())
                .strafeTo(new Vector2d(7, -36))
                .build();
//CASE C INIT END

//CASE T INIT START
        Trajectory trajectoryT1 = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(-43, -13))
                .build();
        Trajectory trajectoryT2 = drive.trajectoryBuilder(trajectoryT1.end())
                .strafeTo(new Vector2d(-5, -4))
                .build();
        Trajectory trajectoryT3 = drive.trajectoryBuilder(trajectoryT2.end())
                .strafeTo(new Vector2d(-6, -35))
                .build();
        Trajectory trajectoryT4 = drive.trajectoryBuilder(trajectoryT3.end())
                .strafeTo(new Vector2d(-15, -35))
                .build();
        Trajectory trajectoryT5 = drive.trajectoryBuilder(trajectoryT4.end())
                .strafeTo(new Vector2d(-20, -35))
                .build();
        Trajectory trajectoryT6 = drive.trajectoryBuilder(trajectoryT5.end())
                .strafeTo(new Vector2d(-5, -35))
                .build();
        Trajectory trajectoryT7 = drive.trajectoryBuilder(trajectoryT6.end())
                .strafeTo(new Vector2d(-30, -35))
                .build();
        Trajectory trajectoryT8 = drive.trajectoryBuilder(trajectoryT7.end())
                .strafeTo(new Vector2d(-37, -35))
                .build();
        Trajectory trajectoryT9 = drive.trajectoryBuilder(trajectoryT8.end())
                .strafeTo(new Vector2d(-5, -35))
                .build();
        Trajectory trajectoryT10 = drive.trajectoryBuilder(trajectoryT9.end().plus(new Pose2d(0, 0, Math.toRadians(139))), false)
                .strafeTo(new Vector2d(42, -50))
                .build();
        Trajectory trajectoryT11 = drive.trajectoryBuilder(trajectoryT10.end())
                .strafeTo(new Vector2d(7, -30))
                .build();
//CASE T INIT END

        telemetry.addData("Status", "Pipeline Initializing");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        telemetry.update();

        //Suspension Down
        drive.frontSuspendServo.setPosition(drive.frontSuspendDown);

        drive.wobbleGoalArm.setPosition(drive.wobbleUp);

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
                drive.shooter.setPower(1);
                drive.followTrajectory(trajectoryB1);
                shootPowerShots(0, 0 ,0);
                drive.followTrajectory(trajectoryB2);
                drive.wobbleGoalArm.setPosition(drive.wobbleDown);
                sleep(270);
                drive.wobblePincher.setPosition(drive.wobblePinchOpen);
                sleep(800);
                drive.wobbleGoalArm.setPosition(drive.wobbleUp);
                drive.lift.setPosition(drive.liftDown);
                drive.backIntake.setPower(-1);
                drive.followTrajectory(trajectoryB3);
                drive.wobbleGoalArm.setPosition(drive.wobbleDown);
                drive.followTrajectory(trajectoryB4);
                sleep(800);
                drive.lift.setPosition(drive.liftUp);
                sleep(800);
                shootGoal(2, 0, -0.007);
                sleep(800);
                drive.wobbleGoalArm.setPosition(drive.wobbleUp);
                drive.lift.setPosition(drive.liftDown);
                drive.followTrajectory(trajectoryB5);
                drive.wobbleGoalArm.setPosition(drive.wobbleDown);
                drive.wobblePincher.setPosition(drive.wobblePinchOpen);
                drive.followTrajectory(trajectoryB6);
                drive.wobblePincher.setPosition(drive.wobblePinchClose);
                sleep(800);
                drive.wobbleGoalArm.setPosition(drive.wobbleUp);
                drive.followTrajectory(trajectoryB7);
                drive.wobbleGoalArm.setPosition(drive.wobbleDown);
                sleep(200);
                drive.wobblePincher.setPosition(drive.wobblePinchOpen);
                sleep(800);
                drive.wobbleGoalArm.setPosition(drive.wobbleUp);
                drive.followTrajectory(trajectoryB8);
                telemetry.addData("Path B", "Complete");
                telemetry.update();
                break;
            case "FOUR":
                drive.shooter.setPower(1);
                drive.followTrajectory(trajectoryC1);
                shootPowerShots(2.2, -2.5 , -2.5);
                sleep(300);
                drive.followTrajectory(trajectoryC2);
                drive.wobbleGoalArm.setPosition(drive.wobbleDown);
                sleep(150);
                drive.wobblePincher.setPosition(drive.wobblePinchOpen);
                sleep(100);
                drive.wobbleGoalArm.setPosition(drive.wobbleUp);
                drive.followTrajectory(trajectoryC3);
                drive.backIntake.setPower(-1);
                drive.lift.setPosition(drive.liftDown);
                drive.followTrajectory(trajectoryC4);
                drive.followTrajectory(trajectoryC5);
                drive.followTrajectory(trajectoryC6);
                drive.lift.setPosition(drive.liftUp);
                sleep(200);
                shootGoal(2, 0, -0.005);
                drive.lift.setPosition(drive.liftDown);
                drive.followTrajectory(trajectoryC7);
                drive.followTrajectory(trajectoryC8);
                drive.followTrajectory(trajectoryC9);
                drive.lift.setPosition(drive.liftUp);
                sleep(200);
                shootGoal(3, -1.5, -0.007);
                sleep(300);
                drive.followTrajectory(trajectoryC10);
                drive.wobbleGoalArm.setPosition(drive.wobbleDown);
                drive.followTrajectory(trajectoryC105);
                drive.wobblePincher.setPosition(drive.wobblePinchClose);
                sleep(300);
                drive.followTrajectory(trajectoryC11);
                drive.wobblePincher.setPosition(drive.wobblePinchOpen - 0.1);
                drive.wobbleGoalArm.setPosition(drive.wobbleUp);
                drive.followTrajectory(trajectoryC12);
                telemetry.addData("Path C", "Complete");
                telemetry.update();
                break;
            case "TEST":
                drive.shooter.setPower(1);
                drive.followTrajectory(trajectoryT1);
                drive.followTrajectory(trajectoryT2);
                shootPowerShots(0, 0 ,0);
                drive.followTrajectory(trajectoryT3);
                drive.backIntake.setPower(-1);
                drive.lift.setPosition(drive.liftDown);
                drive.followTrajectory(trajectoryT4);
                drive.followTrajectory(trajectoryT5);
                drive.followTrajectory(trajectoryT6);
                drive.lift.setPosition(drive.liftUp);
                sleep(300);
                shootGoal(2, 3.5, 0);
                drive.lift.setPosition(drive.liftDown);
                drive.followTrajectory(trajectoryT7);
                drive.followTrajectory(trajectoryT8);
                drive.followTrajectory(trajectoryT9);
                drive.lift.setPosition(drive.liftUp);
                sleep(200);
                shootGoal(2, 3.5, 0);
                sleep(300);
                drive.turn(Math.toRadians(139));
                drive.followTrajectory(trajectoryT10);
                drive.wobbleGoalArm.setPosition(drive.wobbleDown);
                sleep(200);
                drive.wobblePincher.setPosition(drive.wobblePinchOpen);
                drive.followTrajectory(trajectoryT11);
                telemetry.addData("Path T", "Complete");
                telemetry.update();
                break;
        }


        //Transfer Position
        drive.update();
        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    public void shootGoal(int shotCount, double yOffset, double extraHeight) {
        //Set flap angle
        drive.leftFlap.setPosition(drive.leftFlapGoal + extraHeight);
        drive.rightFlap.setPosition(drive.rightFlapGoal - extraHeight);

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
        turretGlobalAngleTargetDegrees = -90 - Math.toDegrees(Math.atan((72 - myPose.getX()) / ((-36) - myPose.getY())));
        if (turretGlobalAngleTargetDegrees < -100) {
            turretGlobalAngleTargetDegrees = turretGlobalAngleTargetDegrees + 180;
        }
        turretAngleTargetDegrees = turretGlobalAngleTargetDegrees - odoHeading + drive.turretAngleOffset + (0.05 * (myPose.getY() + 36));

        //Limit the range of motion for the turret
        if (turretAngleTargetDegrees > 8.5) {
            turretAngleTargetDegrees = 8.5 + yOffset;
        } else if (turretAngleTargetDegrees < -38) {
            turretAngleTargetDegrees = -38 + yOffset;
        }

        //Set timer
        turretTime = runtime.milliseconds();

        while (turretTime + 500 > runtime.milliseconds() && !isStopRequested()) {
            //Update the turret's ticks
            turretTicks = drive.turretMotor.getCurrentPosition() + drive.turretStartTicksOff;

            //Calculate angle error
            turretAngleErrorDegrees = (turretTicks * -0.32360) - turretAngleTargetDegrees - yOffset;

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

    public void shootPowerShots(double off1, double off2, double off3 /*Positive is left*/ ) {
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
        turretGlobalAngleTargetDegrees = -90 - Math.toDegrees(Math.atan((72 - myPose.getX()) / ((-5) - myPose.getY())));
        if (turretGlobalAngleTargetDegrees < -100) {
            turretGlobalAngleTargetDegrees = turretGlobalAngleTargetDegrees + 180;
        }
        turretAngleTargetDegrees = turretGlobalAngleTargetDegrees - odoHeading + drive.turretAngleOffset + (0.1 * (myPose.getY() + 36));

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
            turretTicks = drive.turretMotor.getCurrentPosition() + drive.turretStartTicksOff;

            //Calculate angle error
            turretAngleErrorDegrees = (turretTicks * -0.32360) - turretAngleTargetDegrees - off1;
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

        telemetry.addData("global target", turretGlobalAngleTargetDegrees);
        telemetry.addData("local target", turretAngleTargetDegrees);
        telemetry.addData("turret ticks", turretTicks);
        telemetry.addData("target error", turretAngleErrorDegrees);
        telemetry.update();
        //Kick
        drive.kicker.setPosition(drive.kickerTo);
        sleep(300);
        drive.kicker.setPosition(drive.kickerInit);
        sleep(200);


        //////////////////

        //Set target (Middle)
        turretGlobalAngleTargetDegrees = -90 - Math.toDegrees(Math.atan((72 - myPose.getX()) / ((-12) - myPose.getY())));
        if (turretGlobalAngleTargetDegrees < -100) {
            turretGlobalAngleTargetDegrees = turretGlobalAngleTargetDegrees + 180;
        }
        turretAngleTargetDegrees = turretGlobalAngleTargetDegrees - odoHeading + drive.turretAngleOffset + (0.1 * (myPose.getY() + 36));

        //Limit the range of motion for the turret
        if (turretAngleTargetDegrees > 8.5) {
            turretAngleTargetDegrees = 8.5 + off2;
        } else if (turretAngleTargetDegrees < -38) {
            turretAngleTargetDegrees = -38 + off2;
        }

        //Refresh time
        turretTime = runtime.milliseconds();

        //turretTime + 200 for control video
        while (turretTime + 350 > runtime.milliseconds() && !isStopRequested()) {
            //Update the turret's ticks
            turretTicks = drive.turretMotor.getCurrentPosition() + drive.turretStartTicksOff;

            //Calculate angle error
            turretAngleErrorDegrees = (turretTicks * -0.32360) - turretAngleTargetDegrees - off2;

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
        sleep(300);
        drive.kicker.setPosition(drive.kickerInit);
        sleep(200);

        //////////////////

        //Set target (Right)
        turretGlobalAngleTargetDegrees = -90 - Math.toDegrees(Math.atan((72 - myPose.getX()) / ((-19.5) - myPose.getY())));
        if (turretGlobalAngleTargetDegrees < -100) {
            turretGlobalAngleTargetDegrees = turretGlobalAngleTargetDegrees + 180;
        }
        turretAngleTargetDegrees = turretGlobalAngleTargetDegrees - odoHeading + drive.turretAngleOffset + (0.1 * (myPose.getY() + 36));

        //Limit the range of motion for the turret
        if (turretAngleTargetDegrees > 8.5) {
            turretAngleTargetDegrees = 8.5 + off3;
        } else if (turretAngleTargetDegrees < -38) {
            turretAngleTargetDegrees = -38 + off3;
        }

        //Refresh time
        turretTime = runtime.milliseconds();

        while (turretTime + 350 > runtime.milliseconds() && !isStopRequested()) {
            //Update the turret's ticks
            turretTicks = drive.turretMotor.getCurrentPosition() + drive.turretStartTicksOff;

            //Calculate angle error
            turretAngleErrorDegrees = (turretTicks * -0.32360) - turretAngleTargetDegrees - off3;

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
        final int X_LEFT_T = 1035;
        final int X_RIGHT_T = 1230;
        final int Y_UP_T = 450;
        final int Y_DOWN_T = 475;

        final int X_LEFT_B = 1035;
        final int X_RIGHT_B = 1230;
        final int Y_UP_B = 560;
        final int Y_DOWN_B = 585;

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