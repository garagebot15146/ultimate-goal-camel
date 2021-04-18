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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


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

//    //Vision
//    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
//    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution
//
//    private static final int HORIZON = 270; // horizon value to tune
//
//    private static final boolean DEBUG = false; // if debug is wanted, change to true
//
//    private static final boolean USING_WEBCAM = true; // change to true if using webcam
//    private static final String WEBCAM_NAME = "Webcam"; // insert webcam name from configuration if using webcam
//
//    private UGContourRingPipeline pipeline;
//    private OpenCvCamera camera;

    static SampleMecanumDrive drive;

    //Initialize IMU
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double angleOffset = 0;

    //Turret
    double startTurretTicks; //Notes the starting encoder tick value of the turret motor
    double turretTicks; //Keeps track of motor's ticks ONLY during this session
    double turretAngleTargetDegrees; //Tells the turret what local angle to turn towards
    double turretAngleErrorDegrees; //Tells how far off the turret's local angle is from its local target
    double turretGlobalAngleTargetDegrees; //Sets the global angle target regardless of robot orientation


    @Override
    public void runOpMode() {
        //Hardware map
        drive = new SampleMecanumDrive(hardwareMap);

//        //Hardware Map IMU
//        imu = hardwareMap.get(BNO055IMU .class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//
//        parameters.mode                = BNO055IMU.SensorMode.IMU;
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.loggingEnabled      = false;
//
//        imu.initialize(parameters);

        //Kicker
        drive.kicker.setPosition(drive.kickerInit);

        //Claw
        drive.wobbleGoalArm.setPosition(drive.wobbleUp);
        drive.wobblePincher.setPosition(drive.wobblePinchClose);

        //Reset turret's ticks
        drive.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Suspension
//        drive.frontSuspendServo.setPosition(drive.frontSuspendUp);

        //Lift
        drive.lift.setPosition(drive.liftUp);


//        int cameraMonitorViewId = this
//                .hardwareMap
//                .appContext
//                .getResources().getIdentifier(
//                        "cameraMonitorViewId",
//                        "id",
//                        hardwareMap.appContext.getPackageName()
//                );
//
//        camera = OpenCvCameraFactory
//                .getInstance()
//                .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
//
//
//        camera.setPipeline(pipeline = new UGContourRingPipeline(telemetry, DEBUG));
//
//        UGContourRingPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);
//
//        UGContourRingPipeline.Config.setHORIZON(HORIZON);
//
//        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));
//
//        FtcDashboard.getInstance().startCameraStream(camera, 30);

////PATH CONSTANTS
//        double dc = 0.5;
//        double powerShotX = 89 * dc;
//        double powerShotStrafe = 40 * dc;
//        double firstAngle = -4;
//        double secondAngle = -6.5;
//        double thirdAngle = 10.5;
//
////CASE A INIT START
//        Trajectory trajectoryA1 = drive.trajectoryBuilder(new Pose2d())
//                .strafeTo(new Vector2d(powerShotX, powerShotStrafe))
//                .build();
//
//        Trajectory trajectoryA2 = drive.trajectoryBuilder(trajectoryA1.end())
//                .strafeTo(new Vector2d(85 * dc, -30 * dc))
//                .build();
//
//        Trajectory trajectoryA3 = drive.trajectoryBuilder(trajectoryA2.end())
//                .lineToLinearHeading(new Pose2d(28.5 * dc, 15 * dc, Math.toRadians(-90)))
//                .build();
//
//        Trajectory trajectoryA4 = drive.trajectoryBuilder(trajectoryA3.end())
//                .lineToLinearHeading(new Pose2d(28.5 * dc, -6.5 * dc, Math.toRadians(-90)))
//                .build();
//
//        Trajectory trajectoryA5 = drive.trajectoryBuilder(trajectoryA4.end())
//                .lineToLinearHeading(new Pose2d(85 * dc, -40 * dc, Math.toRadians(0)))
//                .build();
//
//        Trajectory trajectoryA6 = drive.trajectoryBuilder(trajectoryA5.end())
//                .strafeTo(new Vector2d(75 * dc, -30 * dc))
//                .build();
//
//        Trajectory trajectoryA7 = drive.trajectoryBuilder(trajectoryA6.end())
//                .strafeTo(new Vector2d(107 * dc, 30 * dc))
//                .build();
////CASE A INIT END
//
////CASE B INIT START
//        Trajectory trajectoryB1 = drive.trajectoryBuilder(new Pose2d())
//                .strafeTo(new Vector2d(powerShotX, powerShotStrafe))
//                .build();
//
//        Trajectory trajectoryB2 = drive.trajectoryBuilder(trajectoryB1.end())
//                .strafeTo(new Vector2d(124.3 * dc, 0 * dc))
//                .build();
//
//        Trajectory trajectoryB3 = drive.trajectoryBuilder(trajectoryB2.end())
//                .strafeTo(new Vector2d(41 * dc, -5 * dc))
//                .addDisplacementMarker(10 * dc, () -> {
//                    backIntake.setPower(1);
//                })
//                .build();
//
//        Trajectory trajectoryB4 = drive.trajectoryBuilder(trajectoryB3.end())
//                .lineToLinearHeading(new Pose2d(31 * dc, 20 * dc, Math.toRadians(-90)))
//                .build();
//
//        Trajectory trajectoryB5 = drive.trajectoryBuilder(trajectoryB4.end())
//                .lineToLinearHeading(new Pose2d(31 * dc, 0 * dc, Math.toRadians(-90)))
//                .build();
//
//        Trajectory trajectoryB6 = drive.trajectoryBuilder(trajectoryB5.end())
//                .lineToLinearHeading(new Pose2d(50 * dc, 8 * dc, Math.toRadians(0)))
//                .build();
//
//        Trajectory trajectoryB65 = drive.trajectoryBuilder(trajectoryB6.end())
//                .lineToLinearHeading(new Pose2d(58 * dc, -10 * dc, Math.toRadians(0)))
//                .build();
//
//        Trajectory trajectoryB7 = drive.trajectoryBuilder(trajectoryB65.end())
//                .lineToLinearHeading(new Pose2d(powerShotX, -10 * dc, Math.toRadians(0)))
//                .addDisplacementMarker(14 * dc, () -> {
//                    kicker.setPosition(kickerTo);
//                    shooter.setPower(0);
//                })
//                .build();
//
//        Trajectory trajectoryB8 = drive.trajectoryBuilder(trajectoryB7.end())
//                .strafeTo(new Vector2d(122 * dc, 14.5 * dc))
//                .build();
//
//        Trajectory trajectoryB9 = drive.trajectoryBuilder(trajectoryB8.end())
//                .strafeTo(new Vector2d(107 * dc, 30 * dc))
//                .build();
////CASE B INIT END
//
////CASE C INIT START
//        Trajectory trajectoryC1 = drive.trajectoryBuilder(new Pose2d())
//                .strafeTo(new Vector2d(powerShotX, powerShotStrafe))
//                .build();
//
//        Trajectory trajectoryC2 = drive.trajectoryBuilder(trajectoryC1.end())
//                .strafeTo(new Vector2d(160 * dc, -30 * dc))
//                .build();
//
//        Trajectory trajectoryC3 = drive.trajectoryBuilder(trajectoryC2.end())
//                .strafeTo(new Vector2d(130 * dc, 0 * dc))
//                .build();
//
//        Trajectory trajectoryC4 = drive.trajectoryBuilder(trajectoryC3.end())
//                .strafeTo(new Vector2d( 73 * dc, 0 * dc))
//                .build();
//
//        Trajectory trajectoryC45 = drive.trajectoryBuilder(trajectoryC4.end())
//                .strafeTo(new Vector2d( 68 * dc, 0 * dc))
//                .build();
//
//        Trajectory trajectoryC48 = drive.trajectoryBuilder(trajectoryC45.end())
//                .strafeTo(new Vector2d( 62 * dc, 0 * dc))
//                .build();
//
//        Trajectory trajectoryC5 = drive.trajectoryBuilder(trajectoryC48.end())
//                .lineToLinearHeading(new Pose2d(93 * dc, -9 * dc))
//                .build();
//
//        Trajectory trajectoryC6 = drive.trajectoryBuilder(trajectoryC5.end())
//                .lineToLinearHeading(new Pose2d(70 * dc, -30 * dc, Math.toRadians(-180)))
//                .build();
//
//        Trajectory trajectoryC7 = drive.trajectoryBuilder(trajectoryC6.end())
//                .lineToLinearHeading(new Pose2d(41 * dc, -30 * dc, Math.toRadians(-180)))
//                .build();
//
//        Trajectory trajectoryC8 = drive.trajectoryBuilder(trajectoryC7.end())
//                .lineToLinearHeading(new Pose2d(150 * dc, -30 * dc, Math.toRadians(-180)))
//                .build();
////CASE C INIT END
//
        telemetry.addData("Status", "Wait for 3 seconds");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        telemetry.update();

        //Suspension Down
        drive.frontSuspendServo.setPosition(drive.frontSuspendDown);

        drive.shooter.setPower(1);
        sleep(3000);
        shootGoal(3);
        sleep(100);

////        String height = "FOUR";
//        String height = pipeline.getHeight().name();
//        telemetry.addData("Ring Stack", height);
//        telemetry.update();
//        FtcDashboard.getInstance().stopCameraStream();
//
//        switch (height) {
//            case "ZERO":
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
//                break;
//            case "ONE":
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
//                break;
//            case "FOUR":
//                //Turn on shooter. Move to Power Shot
//                shooter.setPower(1);
//                drive.followTrajectory(trajectoryC1);
//                sleep(500);
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
//                shootFlap.setPosition(flapAngleGoal);
//                drive.followTrajectory(trajectoryC2);
//                //Drop off wobble goal
//                armAngle(-100, 0.5);
//                clawServo.setPosition(clawOpen);
//                sleep(450);
//                armAngle(120, 0.4);
//                clawServo.setPosition(clawClose);
//                sleep(300);
//                //Move to ring Y
//                drive.followTrajectory(trajectoryC3);
//                shooter.setPower(1);
//                backIntake.setPower(1);
//                drive.followTrajectory(trajectoryC4);
//                sleep(540);
//                drive.followTrajectory(trajectoryC45);
//                sleep(540);
//                drive.followTrajectory(trajectoryC48);
//                sleep(540);
//                drive.followTrajectory(trajectoryC5);
//                basketUp();
//                sleep(800);
//                kick(4);
//                sleep(500);
//                drive.followTrajectory(trajectoryC6);
//                backIntake.setPower(0);
//                armAngle(-100, 0.5);
//                clawServo.setPosition(clawOpen);
//                drive.followTrajectory(trajectoryC7);
//                clawServo.setPosition(clawClose);
//                sleep(150);
//                drive.followTrajectory(trajectoryC8);
//                drive.turn(Math.toRadians(-180));
//                //Drop off wobble goal again and book it
//                clawServo.setPosition(clawOpen);
//                encoderDrive(1, -12, -12, -12, -12, 4);
//                telemetry.addData("Path C", "Complete");
//                telemetry.update();
//                break;
//        }
//

        //Transfer Position
        drive.update();
        Pose2d myPose = drive.getPoseEstimate();
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
        turretGlobalAngleTargetDegrees = -90 - Math.toDegrees(Math.atan( (72 - myPose.getX()) / (-36 - myPose.getY()) ) );
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
            } else if (turretAngleErrorDegrees < 0){
                if (turretAngleErrorDegrees < -8) {
                    //Don't go too fast if turret is far from target
                    drive.turretMotor.setPower(-0.2);
                } else {
                    drive.turretMotor.setPower( Math.pow(0.1 * turretAngleErrorDegrees + 0.5848, 3) - 0.2);
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

        //Fix odometry's heading range
        double odoHeading;
        if (Math.toDegrees(myPose.getHeading()) > 180) {
            odoHeading = Math.toDegrees(myPose.getHeading()) - 360;
        } else {
            odoHeading = Math.toDegrees(myPose.getHeading());
        }

        //Set target (Left)
        turretGlobalAngleTargetDegrees = -90 - Math.toDegrees(Math.atan( (72 - myPose.getX()) / (-5 - myPose.getY()) ) );
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
            } else if (turretAngleErrorDegrees < 0){
                if (turretAngleErrorDegrees < -8) {
                    //Don't go too fast if turret is far from target
                    drive.turretMotor.setPower(-0.2);
                } else {
                    drive.turretMotor.setPower( Math.pow(0.1 * turretAngleErrorDegrees + 0.5848, 3) - 0.2);
                }
            }
        }

        //Kick
        drive.kicker.setPosition(drive.kickerTo);
        sleep(150);
        drive.kicker.setPosition(drive.kickerInit);

        //////////////////

        //Set target (Middle)
        turretGlobalAngleTargetDegrees = -90 - Math.toDegrees(Math.atan( (72 - myPose.getX()) / (-12 - myPose.getY()) ) );
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
            } else if (turretAngleErrorDegrees < 0){
                if (turretAngleErrorDegrees < -8) {
                    //Don't go too fast if turret is far from target
                    drive.turretMotor.setPower(-0.2);
                } else {
                    drive.turretMotor.setPower( Math.pow(0.1 * turretAngleErrorDegrees + 0.5848, 3) - 0.2);
                }
            }
        }

        //Kick
        drive.kicker.setPosition(drive.kickerTo);
        sleep(150);
        drive.kicker.setPosition(drive.kickerInit);

        //////////////////

        //Set target (Right)
        turretGlobalAngleTargetDegrees = -90 - Math.toDegrees(Math.atan( (72 - myPose.getX()) / (-19.5 - myPose.getY()) ) );
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
            } else if (turretAngleErrorDegrees < 0){
                if (turretAngleErrorDegrees < -8) {
                    //Don't go too fast if turret is far from target
                    drive.turretMotor.setPower(-0.2);
                } else {
                    drive.turretMotor.setPower( Math.pow(0.1 * turretAngleErrorDegrees + 0.5848, 3) - 0.2);
                }
            }
        }

        //Kick
        drive.kicker.setPosition(drive.kickerTo);
        sleep(150);
        drive.kicker.setPosition(drive.kickerInit);
    }
}

