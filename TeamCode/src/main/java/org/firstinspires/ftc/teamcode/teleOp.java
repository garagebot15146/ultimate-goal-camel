/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.Encoder;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="teleOp", group="Iterative Opmode")
//@Disabled
public class teleOp extends OpMode
{
    //Declare runtime variable
    private ElapsedTime runtime = new ElapsedTime();
    double currentTime;
    double shooterTime = runtime.milliseconds();
    double autoKickTime;

    //Initialize IMU
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double angleOffset = 0;

    //Variables for running methods
    //Shooter
    boolean shooterToggle = false;
    boolean shooterOn = false;
    boolean g2RightTriggerPressed = false;

    //Kicker
    boolean kickerHasRun = false;

    boolean blockerToggle = false;
    boolean blockerDown = false;

    //Shooter RPM
    double shooterPosition;
    double shooterRPM;

    static SampleMecanumDrive drive;

    //Setting position
    boolean startPositionSet = false;

    //Display on Dashboard
    private FtcDashboard dashboard;

    //Wobble Goal
    int wobbleStage = 0; //0 -> Initialize. 1 -> Down and Open, 2 -> Close, 3 -> Up, 4 -> Open
    boolean wobbleHasRun = false;

    //Turret
    double turretTicks; //Keeps track of motor's ticks ONLY during this session
    double turretAngleTargetDegrees; //Tells the turret what local angle to turn towards
    double turretAngleErrorDegrees; //Tells how far off the turret's local angle is from its local target
    double turretGlobalAngleTargetDegrees; //Sets the global angle target regardless of robot orientation
    boolean turretManual = true;
    boolean turretManualMethodToggle = false;
    int turretTarget = 0; //0 = Goal, 1, 2, 3 are powershots left to right

    //testing pods
    private Encoder leftEncoder, rightEncoder;
    double leftDistance, rightDistance;

    //Initialize
    @Override
    public void init() {

        //Hardware Map IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);

        drive = new SampleMecanumDrive(hardwareMap);

        //Wobble Goal
        drive.wobbleGoalArm.setPosition(drive.wobbleUp);
        drive.wobblePincher.setPosition(drive.wobblePinchClose);

        //Kicker
        drive.kicker.setPosition(drive.kickerInit);

        //testing pods
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontIntake"));
        leftDistance = leftEncoder.getCurrentPosition();
        rightDistance = rightEncoder.getCurrentPosition();

        //Initialized
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    //Repeat loop prior to hitting play
    @Override
    public void init_loop() {
    }

    //Play once
    @Override
    public void start() {

        runtime.reset();
        currentTime = 0;
    }

    //Play loop
    @Override
    public void loop() {
        //Set starting position (run once)
        if (startPositionSet == false) {
            drive.setPoseEstimate(PoseStorage.currentPose);
            startPositionSet = true;
        }

        //Update position
        drive.update();

        //Retrieve Position
        Pose2d myPose = drive.getPoseEstimate();
        telemetry.addData("x", myPose.getX());
        telemetry.addData("y", myPose.getY());
        telemetry.addData("imu heading", lastAngles.firstAngle);
        telemetry.addData("odo heading", Math.toDegrees(myPose.getHeading()));

        //testing pods
        telemetry.addData("left distance", leftDistance - leftEncoder.getCurrentPosition());
        telemetry.addData("right distance", rightDistance - rightEncoder.getCurrentPosition());

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        //Set variables for motor powers
        //Wheels
        double leftFrontPower = 0;
        double leftBackPower = 0;
        double rightFrontPower = 0;
        double rightBackPower = 0;

        //Used for Calculations:
        double forward = 0;
        double side = 0;
        double turn = 0;

        /////////////
        //GAMEPAD 1//
        /////////////

        //DRIVE
        forward = -gamepad1.left_stick_y;
        side = gamepad1.left_stick_x; //Positive means right
        turn = gamepad1.right_stick_x; //Positive means turn right


        leftFrontPower = (forward + side + turn) / 2;
        leftBackPower = (forward - side + turn) / 2;
        rightFrontPower = (forward - side - turn) / 2;
        rightBackPower = (forward + side - turn) / 2;

        //Boost; Slow; Normal
        if (gamepad1.x) {
            //BOOST

            leftFrontPower = leftFrontPower * 2;
            leftBackPower = leftBackPower * 2;
            rightFrontPower = rightFrontPower * 2;
            rightBackPower = rightBackPower * 2;

        } else if (gamepad1.a) {
            //SLOW

            leftFrontPower = leftFrontPower * 0.6;
            leftBackPower = leftBackPower * 0.6;
            rightFrontPower = rightFrontPower * 0.6;
            rightBackPower = rightBackPower * 0.6;

        } else {
            //This is normal.  Don't put anything here.
        }

        // Send power to wheel motors
        drive.leftFront.setPower(leftFrontPower);
        drive.rightFront.setPower(rightFrontPower);
        drive.leftBack.setPower(leftBackPower);
        drive.rightBack.setPower(rightBackPower);


        //Blockers
        if (gamepad1.left_bumper && blockerToggle == false) {
            blockerToggle = true;
            //If blocker is already down, move it up and vise versa
            if (blockerDown == false) {
                drive.ringBlocker.setPosition(drive.ringBlockDown);
                blockerDown = true;
            } else if (blockerDown == true) {
                drive.ringBlocker.setPosition(drive.ringBlockUp);
                blockerDown = false;
            }
        } else if (!gamepad1.left_bumper && blockerToggle == true) {
            blockerToggle = false;
        }

        //Reset Position
        if (gamepad1.right_bumper) {
            drive.setPoseEstimate(new Pose2d(0, -39, Math.toRadians(0)));
            angleOffset = lastAngles.firstAngle;
        }

        //Target Select
        if (gamepad1.dpad_right) {
            //Goal
            turretTarget = 0;
            drive.leftFlap.setPosition(drive.leftFlapGoal);
            drive.rightFlap.setPosition(drive.rightFlapGoal);
        } else if (gamepad1.dpad_up) {
            //Left
            turretTarget = 1;
            drive.leftFlap.setPosition(drive.leftFlapPowerShot);
            drive.rightFlap.setPosition(drive.rightFlapPowerShot);
        } else if (gamepad1.dpad_left) {
            //Middle
            turretTarget = 2;
            drive.leftFlap.setPosition(drive.leftFlapPowerShot);
            drive.rightFlap.setPosition(drive.rightFlapPowerShot);
        } else if (gamepad1.dpad_down) {
            //Right
            turretTarget = 3;
            drive.leftFlap.setPosition(drive.leftFlapPowerShot);
            drive.rightFlap.setPosition(drive.rightFlapPowerShot);
        }

        /////////////
        //GAMEPAD 2//
        /////////////

        //Intake
        if (gamepad2.left_stick_y > 0.1) {
            //In

            drive.frontIntake.setPower(-1);
            drive.backIntake.setPower(1);
        } else if (gamepad2.left_stick_y < -0.1 ) {
            //Out
            drive.frontIntake.setPower(1);
            drive.backIntake.setPower(-1);
        } else {
            //Stop
            drive.frontIntake.setPower(0);
            drive.backIntake.setPower(0);
        }

        //If intake is active, bring lift down, unless Y is pressed
        if((gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1) && !gamepad2.b ) {
            drive.lift.setPosition(drive.liftDown);
        }

        //Lift
        if (gamepad2.dpad_up) {
            drive.lift.setPosition(drive.liftUp);
        } else if (gamepad2.dpad_down) {
            drive.lift.setPosition(drive.liftDown);
        }


        //Shooter
        //Toggle
        if (gamepad2.y && shooterToggle == false) {
            shooterToggle = true;

            if (shooterOn == false) {
                //Toggle On
                drive.shooter.setPower(1);
                shooterOn = true;
            } else if (shooterOn == true) {
                //Toggle off
                drive.shooter.setPower(0);
                shooterOn = false;
            }
        } else if (!gamepad2.y && shooterToggle == true) {
            shooterToggle = false;
        }
        //Right Trigger (Vanilla) Takes priority
        if (gamepad2.right_trigger > 0.1 && g2RightTriggerPressed == false) {
            //On
            drive.shooter.setPower(1);
            shooterToggle = false;
            shooterOn = false;
            g2RightTriggerPressed = true;
        } else if (gamepad2.right_trigger < 0.1 && g2RightTriggerPressed == true) {
            //Off
            g2RightTriggerPressed = false;
            drive.shooter.setPower(0);
        }

        //Measure RPM
        if (shooterTime + 100 < runtime.milliseconds()) {
            shooterTime = runtime.milliseconds();
            shooterPosition = drive.shooter.getCurrentPosition();
            shooterRPM = (drive.shooter.getCurrentPosition() - shooterPosition) / 28 * 10 * 60;
        }
//        telemetry.addData("Shooter RPM", (int) shooterRPM);


        //Kicker
        if (gamepad2.a && !kickerHasRun && !gamepad2.start && runtime.milliseconds() > currentTime + 300) {
            currentTime = runtime.milliseconds();
            drive.kicker.setPosition(drive.kickerTo);
            kickerHasRun = true;
            //Bring down ring blocker
            drive.ringBlocker.setPosition(drive.ringBlockDown);
            blockerDown = true;
        }
        if (runtime.milliseconds() > currentTime + 150 && kickerHasRun == true) {
            drive.kicker.setPosition(drive.kickerInit);
            kickerHasRun = false;
        }

        //Wobble Goal
        //Arm
        if (gamepad2.right_bumper && wobbleHasRun == false) {
            //Cycle through stages by pressing button
            wobbleStage = wobbleStage + 1;
            if (wobbleStage == 5){
                wobbleStage = 1;
            }
            wobbleHasRun = true;
        } else if (!gamepad2.right_bumper && wobbleHasRun == true) {
            wobbleHasRun = false;
        }
        //Run each stage
        if (wobbleStage == 0) {
            //Init: Arm up, pinch close
            drive.wobbleGoalArm.setPosition(drive.wobbleUp);
            drive.wobblePincher.setPosition(drive.wobblePinchClose);
        } else if (wobbleStage == 1) {
            //Arm Down, pinch open
            drive.wobbleGoalArm.setPosition(drive.wobbleDown);
            drive.wobblePincher.setPosition(drive.wobblePinchOpen);
        } else if (wobbleStage == 2) {
            //Arm down, pinch close
            drive.wobbleGoalArm.setPosition(drive.wobbleDown);
            drive.wobblePincher.setPosition(drive.wobblePinchClose);
        } else if (wobbleStage == 3) {
            //Arm Up, pinch close
            drive.wobbleGoalArm.setPosition(drive.wobbleUp);
            drive.wobblePincher.setPosition(drive.wobblePinchClose);
        } else if (wobbleStage == 4) {
            //Arm up, pinch open
            drive.wobbleGoalArm.setPosition(drive.wobbleUp);
            drive.wobblePincher.setPosition(drive.wobblePinchOpen);
        }


        //Toggle manual turret controls
        if (turretManualMethodToggle == false && gamepad2.right_stick_button) {
            turretManualMethodToggle = true;
            //Switch turret control mode
            if (turretManual == false) {
                turretManual = true;
            } else if (turretManual == true) {
                turretManual = false;
            }
        } else if (turretManualMethodToggle == true && !gamepad2.right_stick_button) {
            turretManualMethodToggle = false;
        }


        //IMU
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //////////
        //TURRET//
        //////////

        //1 Tick = 0.32360 degrees.
        //1 Degree = 3.08793 ticks.

        //keep track of the ticks on the turret motor for this session.
        turretTicks = drive.turretMotor.getCurrentPosition();

        //Calculate Local angle target
        //Check if manual
        if (!turretManual) {
            //Not manual

            //Pick target:
            if (turretTarget == 0) {
                //Goal
                turretGlobalAngleTargetDegrees = -90 - Math.toDegrees(Math.atan( (72 - myPose.getX()) / (-36 - myPose.getY()) ) );
            } else if (turretTarget == 1) {
                //Powershot left
                turretGlobalAngleTargetDegrees = -90 - Math.toDegrees(Math.atan( (72 - myPose.getX()) / (-4.5 - myPose.getY()) ) );
            } else if (turretTarget == 2) {
                //Powershot middle
                turretGlobalAngleTargetDegrees = -90 - Math.toDegrees(Math.atan( (72 - myPose.getX()) / (-12 - myPose.getY()) ) );
            } else if (turretTarget == 3) {
                //Powershot right
                turretGlobalAngleTargetDegrees = -90 - Math.toDegrees(Math.atan( (72 - myPose.getX()) / (-19.5 - myPose.getY()) ) );
            }

            if (turretGlobalAngleTargetDegrees < -100) {
                turretGlobalAngleTargetDegrees = turretGlobalAngleTargetDegrees + 180;
            }
            turretAngleTargetDegrees = turretGlobalAngleTargetDegrees - (lastAngles.firstAngle - angleOffset) + drive.turretAngleOffset + (0.05 * (myPose.getY() + 36));
        } else if (turretManual) {
            turretAngleTargetDegrees = turretAngleTargetDegrees - gamepad2.right_stick_x * 0.5;
        }

        //Limit the range of motion for the turret
        if (turretAngleTargetDegrees > 8.5) {
            turretAngleTargetDegrees = 8.5;
        } else if (turretAngleTargetDegrees < -38) {
            turretAngleTargetDegrees = -38;
        }

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

        telemetry.addData("turretGlobalAngle", turretGlobalAngleTargetDegrees);
    }

    //Stop code
    @Override
    public void stop() {
    }

}