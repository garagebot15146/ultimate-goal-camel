package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "Auto", group = "Autonomous")
//@Disabled
public class auto extends LinearOpMode {

    enum PATH {
        A,
        B,
        C
    }

    //Declare motors/servos variables
    private ElapsedTime runtime = new ElapsedTime();
    //Initialize Motors/Servos
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    private DcMotor frontIntake = null;
    private DcMotor backIntake = null;

    private DcMotor clawArm = null;
    private Servo clawServo = null;
    double clawClose = 0.90;
    double clawOpen = 0.6;

    private DcMotor shooter = null;
    private Servo kicker = null;
    double kickerInit = 0.3200;
    double kickerTo = 0.5848;
    private Servo shootFlap;
    double flapAngleGoal = 0.121; //Higher = Steeper
    double flapAnglePowerShot = 0.107;

    private Servo leftLift = null;
    private Servo rightLift = null;

    double leftLiftUp = 0.9176; //1 Top
    double rightLiftUp = 0.9413; //1 Top

    double leftLiftDown = 0.45;
    double rightLiftDown = 0.45;


    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.77953;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    //Vision
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    private static final int HORIZON = 100; // horizon value to tune

    private static final boolean DEBUG = false; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "Webcam"; // insert webcam name from configuration if using webcam

    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;

    @Override
    public void runOpMode() {

        //Hardware Maps
        //Wheels
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        backIntake = hardwareMap.get(DcMotor.class, "backIntake");

        clawArm = hardwareMap.get(DcMotor.class, "clawArm");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        //Servos
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        kicker = hardwareMap.get(Servo.class, "kicker");
        kicker.setPosition(kickerInit);
        shootFlap = hardwareMap.get(Servo.class, "shootFlap");
        shootFlap.setPosition(flapAnglePowerShot);

        leftLift = hardwareMap.get(Servo.class, "leftLift");
        rightLift = hardwareMap.get(Servo.class, "rightLift");

        leftLift.setPosition(1 - leftLiftUp);
        rightLift.setPosition(rightLiftUp);

        //Set motor run modes
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set Directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        frontIntake.setDirection(DcMotor.Direction.FORWARD);
        backIntake.setDirection(DcMotor.Direction.FORWARD);

        shooter.setDirection(DcMotor.Direction.REVERSE);

        //Claw Arm
        clawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawServo.setPosition(clawClose);
        basketUp();


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

//PATH CONSTANTS
        double dc = 0.5;
        double powerShotX = 87 * dc;
        double powerShotStrafe = 40 * dc;

//CASE A INIT START
        Trajectory trajectoryA1 = drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(powerShotX, powerShotStrafe))
                .build();

        Trajectory trajectoryA2 = drive.trajectoryBuilder(trajectoryA1.end())
                .strafeTo(new Vector2d(85 * dc, -30 * dc))
                .build();

        Trajectory trajectoryA3 = drive.trajectoryBuilder(trajectoryA2.end())
                .lineToLinearHeading(new Pose2d(27 * dc, 15 * dc, Math.toRadians(-90)))
                .build();

        Trajectory trajectoryA4 = drive.trajectoryBuilder(trajectoryA3.end())
                .lineToLinearHeading(new Pose2d(27 * dc, -6.5 * dc, Math.toRadians(-90)))
                .build();

        Trajectory trajectoryA5 = drive.trajectoryBuilder(trajectoryA4.end())
                .lineToLinearHeading(new Pose2d(85 * dc, -40 * dc, Math.toRadians(0)))
                .build();

        Trajectory trajectoryA6 = drive.trajectoryBuilder(trajectoryA5.end())
                .strafeTo(new Vector2d(75 * dc, -40 * dc))
                .build();

        Trajectory trajectoryA7 = drive.trajectoryBuilder(trajectoryA6.end())
                .strafeTo(new Vector2d(107 * dc, 30 * dc))
                .build();

        Trajectory returnHomeA = drive.trajectoryBuilder(trajectoryA7.end().plus(new Pose2d(0, 0, Math.toRadians(-92))), false)
                .lineToLinearHeading(new Pose2d(3 * dc, 0 * dc, Math.toRadians(0)))
                .build();
//CASE A INIT END

//CASE B INIT START
        Trajectory trajectoryB1 = drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(powerShotX, powerShotStrafe))
                .build();

        Trajectory trajectoryB2 = drive.trajectoryBuilder(trajectoryB1.end())
                .strafeTo(new Vector2d(124.3 * dc, 0 * dc))
                .build();

        Trajectory trajectoryB3 = drive.trajectoryBuilder(trajectoryB2.end())
                .strafeTo(new Vector2d(46 * dc, -5 * dc))
                .addDisplacementMarker(10 * dc, () -> {
                    backIntake.setPower(1);
                })
                .build();

        Trajectory trajectoryB4 = drive.trajectoryBuilder(trajectoryB3.end())
                .lineToLinearHeading(new Pose2d(31 * dc, 20 * dc, Math.toRadians(-90)))
                .build();

        Trajectory trajectoryB5 = drive.trajectoryBuilder(trajectoryB4.end())
                .lineToLinearHeading(new Pose2d(31 * dc, 0 * dc, Math.toRadians(-90)))
                .build();

        Trajectory trajectoryB6 = drive.trajectoryBuilder(trajectoryB5.end())
                .lineToLinearHeading(new Pose2d(50 * dc, 8 * dc, Math.toRadians(0)))
                .build();

        Trajectory trajectoryB7 = drive.trajectoryBuilder(trajectoryB6.end())
                .lineToLinearHeading(new Pose2d(powerShotX, 8 * dc, Math.toRadians(0)))
                .addDisplacementMarker(25 * dc, () -> {
                    kick(1);
                    shooter.setPower(0);
                })
                .build();

        Trajectory trajectoryB8 = drive.trajectoryBuilder(trajectoryB7.end())
                .strafeTo(new Vector2d(118 * dc, 14.5 * dc))
                .build();

        Trajectory trajectoryB9 = drive.trajectoryBuilder(trajectoryB8.end())
                .strafeTo(new Vector2d(107 * dc, 30 * dc))
                .build();

        Trajectory returnHomeB = drive.trajectoryBuilder(trajectoryB9.end().plus(new Pose2d(0, 0, Math.toRadians(-92))), false)
                .lineToLinearHeading(new Pose2d(3 * dc, 0 * dc, Math.toRadians(0)))
                .build();
//CASE B INIT END

        telemetry.addData("Status", "Initialized");
        waitForStart();

        if (isStopRequested()) return;

        PATH path = PATH.B;

        switch (path) {
            case A:
                //Turn on shooter. Move to Power Shot
                shooter.setPower(1);
                drive.followTrajectory(trajectoryA1);
                sleep(1000);
                //Take shot 1
                kick(1);
                //Turn to left powershot
                drive.turn(Math.toRadians(4));
                //Take shot 2
                kick(1);
                //Turn to right powershot
                drive.turn(Math.toRadians(-10));
                //Take shot 3
                kick(1);
                shooter.setPower(0);
                //Reset heading to 0
                drive.turn(Math.toRadians(6));
                //Drive to zone
                drive.followTrajectory(trajectoryA2);
                //Drop off wobble goal
                armAngle(-90, 0.3);
                clawServo.setPosition(clawOpen);
                sleep(500);
                armAngle(120, 0.4);
                clawServo.setPosition(clawClose);
                sleep(300);
                //Prepare to pickup second wobble goal
                drive.followTrajectory(trajectoryA3);
                armAngle(-90, 0.3);
                clawServo.setPosition(clawOpen);
                sleep(200);
                //Collect second wobble goal
                drive.followTrajectory(trajectoryA4);
                clawServo.setPosition(clawClose);
                sleep(500);
                //Move to zone again
                drive.followTrajectory(trajectoryA5);
                //Drop off wobble goal 2
                clawServo.setPosition(clawOpen);
                sleep(500);
                armAngle(120, 0.4);
                clawServo.setPosition(clawClose);
                //Park on tape
                drive.followTrajectory(trajectoryA6);
                drive.followTrajectory(trajectoryA7);
                sleep(4000);
                //Return home
                drive.followTrajectory(returnHomeA);
                telemetry.addData("Path A", "Complete");
                telemetry.update();
                break;
            case B:
                //Turn on shooter. Move to Power Shot
                shooter.setPower(1);
                drive.followTrajectory(trajectoryB1);
                sleep(1000);
                //Take shot 1
                kick(1);
                //Turn to left powershot
                drive.turn(Math.toRadians(4));
                //Take shot 2
                kick(1);
                //Turn to right powershot
                drive.turn(Math.toRadians(-10));
                //Take shot 3
                kick(1);
                shooter.setPower(0);
                //Reset heading to 0
                drive.turn(Math.toRadians(6));
                //Basket down. Drive to zone
                basketDown();
                drive.followTrajectory(trajectoryB2);
                //Drop off wobble goal
                armAngle(-90, 0.3);
                clawServo.setPosition(clawOpen);
                sleep(500);
                armAngle(120, 0.4);
                clawServo.setPosition(clawClose);
                sleep(300);
                //Prepare to intake one ring
                drive.followTrajectory(trajectoryB3);
                shooter.setPower(1);
                shootFlap.setPosition(flapAngleGoal);
                //Prepare to pickup second wobble goal
                drive.followTrajectory(trajectoryB4);
                backIntake.setPower(0);
                basketUp();
                armAngle(-90, 0.3);
                clawServo.setPosition(clawOpen);
                sleep(200);
                //Collect second wobble goal
                drive.followTrajectory(trajectoryB5);
                clawServo.setPosition(clawClose);
                sleep(500);
                //Drive to align with goal
                drive.followTrajectory(trajectoryB6);
                //Shoot ring
                drive.followTrajectory(trajectoryB7);
                //Move to zone again
                drive.followTrajectory(trajectoryB8);
                //Drop off wobble goal 2
                clawServo.setPosition(clawOpen);
                sleep(500);
                armAngle(120, 0.4);
                clawServo.setPosition(clawClose);
                //Park on tape
                drive.followTrajectory(trajectoryB9);
                sleep(4000);
                //Return home
                drive.followTrajectory(returnHomeB);
                telemetry.addData("Path B", "Complete");
                telemetry.update();
                break;
            case C:
                telemetry.addData("Path C", "Complete");
                telemetry.update();
                break;
        }

    }


    public void armAngle(double degrees, double power) {
        int newTarget = clawArm.getCurrentPosition() + (int) (degrees * 1.4933);
        clawArm.setTargetPosition(newTarget);
        // Turn On RUN_TO_POSITION
        clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // reset the timeout time and start motion.
        runtime.reset();
        clawArm.setPower(power);

        double currentTime = runtime.milliseconds();
        int clawPosition = clawArm.getCurrentPosition();
        boolean clawStuck = false;

        while (clawArm.isBusy() && !clawStuck) {
            //Check if motor stuck
            if (runtime.milliseconds() > currentTime + 100) {
                currentTime = runtime.milliseconds();

                if (((clawPosition + 4 > clawArm.getCurrentPosition()) && degrees > 0) ||
                        (clawPosition - 4 < clawArm.getCurrentPosition()) && degrees < 0) {
                    clawArm.setPower(0);
                    clawStuck = true;
                } else {
                    clawPosition = clawArm.getCurrentPosition();
                }

            }
        }

        //Stop
        clawArm.setPower(0);
        clawArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void kick(int kickCount) {
        for (int i = 0; i < kickCount; i++) {
            kicker.setPosition(kickerTo);
            sleep(150);
            kicker.setPosition(kickerInit);
            sleep(200);
        }
    }

    public void basketUp() {
        leftLift.setPosition(1 - leftLiftUp);
        rightLift.setPosition(rightLiftUp);
    }

    public void basketDown() {
        leftLift.setPosition(1 - leftLiftDown);
        rightLift.setPosition(rightLiftDown);
    }

    public void encoderDrive(double speed,
                             double leftFrontInches, double rightFrontInches, double leftBackInches, double rightBackInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int) (leftFrontInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() + (int) (rightFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBack.getCurrentPosition() + (int) (leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBack.getCurrentPosition() + (int) (rightBackInches * COUNTS_PER_INCH);

            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            leftBack.setTargetPosition(newLeftBackTarget);
            rightBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {

//                // Display it for the driver.
//                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
//                telemetry.addData("Path2",  "Running at %7d :%7d",
//                                            robot.leftDrive.getCurrentPosition(),
//                                            robot.rightDrive.getCurrentPosition());
//                telemetry.update();
            }

            // Stop all motion;
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

}

