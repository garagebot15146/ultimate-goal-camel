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

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


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
    double vibrateTime = runtime.milliseconds();

    //Initialize IMU
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();

    //Set Motor objects
    Motor leftFront;
    Motor rightFront;
    Motor leftBack;
    Motor rightBack;

    Motor shooter;
    Motor backIntake;
    Motor frontIntake;

    private DcMotor clawArm = null;

    //Set Servo objects
    SimpleServo leftLift;
    SimpleServo rightLift;

    double leftLiftUp = 0.9476; //1 Top
    double rightLiftUp = 0.9713; //1 Top

    double leftLiftDown = 0.4815;
    double rightLiftDown = 0.4783;

    boolean g1rightbumperpressed = false;
    boolean vibrated = false;
    boolean switchVibrate = false;

    SimpleServo clawServo;
    double clawClose = 0.90;
    double clawOpen = 0.6;

    SimpleServo kicker;
    SimpleServo shootFlap;

    double kickerInit = 0.3200;
    double kickerTo = 0.5848;

    boolean kickerHasRun = false;
    boolean kickerMethodRun = false;

    double flapAngleGoal = 0.121; //Higher = Steeper
    double flapAnglePowerShot = 0.1;

    //Shooter RPM
    double shooterPosition;
    double shooterRPM;

    //Initialize
    @Override
    public void init() {
        //Define motors/servos hardware maps
        leftFront = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_435);
        rightFront = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_435);
        leftBack = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_435);
        rightBack = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_435);

        shooter = new Motor(hardwareMap, "shooter", 28, 6000);
        backIntake = new Motor(hardwareMap, "backIntake", 5, 6);
        frontIntake = new Motor(hardwareMap, "frontIntake", 5, 6);

        clawArm = hardwareMap.get(DcMotor.class, "clawArm");

        leftLift = new SimpleServo(hardwareMap, "leftLift");
        rightLift = new SimpleServo(hardwareMap, "rightLift");

        kicker = new SimpleServo(hardwareMap, "kicker") ;
        shootFlap = new SimpleServo(hardwareMap, "shootFlap") ;

        clawServo = new SimpleServo(hardwareMap, "clawServo");

        //Set Run modes
        leftFront.setRunMode(Motor.RunMode.RawPower);
        rightFront.setRunMode(Motor.RunMode.RawPower);
        leftBack.setRunMode(Motor.RunMode.RawPower);
        rightBack.setRunMode(Motor.RunMode.RawPower);

        clawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setRunMode(Motor.RunMode.RawPower);
        backIntake.setRunMode(Motor.RunMode.RawPower);
        frontIntake.setRunMode(Motor.RunMode.RawPower);

        //Set Directions
        leftFront.setInverted(true);
        rightFront.setInverted(false);
        leftBack.setInverted(true);
        rightBack.setInverted(false);

        shooter.setInverted(true);
        backIntake.setInverted(true);
        frontIntake.setInverted(true);

        //Initialize Servo Positions
        kicker.setPosition(kickerInit);
        shootFlap.setPosition(flapAngleGoal);

        //Hardware Map IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);

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
    }

    //Play loop
    @Override
    public void loop() {
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
        leftFront.set(leftFrontPower);
        rightFront.set(rightFrontPower);
        leftBack.set(leftBackPower);
        rightBack.set(rightBackPower);

        //Vibrate Basket

        if (gamepad1.right_bumper) {
            g1rightbumperpressed = true;
            //Check if 200 ms has passed
            if (vibrateTime + 75 < runtime.milliseconds()) {
                //If 200ms has passed
                vibrateTime = runtime.milliseconds();

                //vibrated true means up oscillation
                if (vibrated == true) {
                    //Oscillate up
                    leftLift.setPosition(1 - (leftLiftUp + 0.02));
                    rightLift.setPosition(rightLiftUp + 0.02);
                } else {
                    //Oscillate Down
                    leftLift.setPosition(1 - (leftLiftUp - 0.02));
                    rightLift.setPosition(rightLiftUp - 0.02);
                }

                //Switch vibrated variable
                if (vibrated == true) {
                    vibrated = false;
                } else {
                    vibrated = true;
                }
            }
        } else if (!gamepad1.right_bumper && g1rightbumperpressed == true) {
            //When release right bumper, return lift to up position (trigger once)
            leftLift.setPosition(1 - leftLiftUp);
            rightLift.setPosition(rightLiftUp);
            g1rightbumperpressed = false;
        }



        /////////////
        //GAMEPAD 2//
        /////////////

        //Intake

        if (gamepad2.left_stick_y > 0.1) {
            //In
            frontIntake.set(1);
            backIntake.set(1);
        } else if (gamepad2.left_stick_y < -0.1 ) {
            //Out
            frontIntake.set(-1);
            backIntake.set(-1);
        } else {
            //Stop
            frontIntake.set(0);
            backIntake.set(0);
        }

        //If intake is active, bring lift down
        if(gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1) {
            leftLift.setPosition(1 - leftLiftDown);
            rightLift.setPosition(rightLiftDown);
        }


        //Shooter
        shooter.set(gamepad2.right_trigger * 1.00);

        //Measure RPM
        if (shooterTime + 100 < runtime.milliseconds()) {
            shooterTime = runtime.milliseconds();
            shooterRPM = (shooter.getCurrentPosition() - shooterPosition) / 28 * 10 * 60;
            shooterPosition = shooter.getCurrentPosition();
        }

        //Lift
        if (gamepad2.dpad_up) {
            //Up
            leftLift.setPosition(1 - leftLiftUp);
            rightLift.setPosition((rightLiftUp));
        } else if (gamepad2.dpad_down) {
            //Down
            leftLift.setPosition(1 - leftLiftDown);
            rightLift.setPosition(rightLiftDown);
        }

        //Kicker
        if (gamepad2.a && !kickerHasRun && !gamepad2.start) {
            currentTime = runtime.milliseconds();
            kicker.setPosition(kickerTo);
            kickerHasRun = true;
        }

        if (runtime.milliseconds() > currentTime + 150) {
            kicker.setPosition(kickerInit);
            kickerMethodRun = true;
        }

        if (kickerMethodRun && !gamepad2.a) {
            kickerMethodRun = false;
            kickerHasRun = false;
        }

        //shootFlap
        if (gamepad2.y) {
            //Power Shot (Lower)
            shootFlap.setPosition(flapAnglePowerShot);
        } else if (gamepad2.b) {
            //Goal (Higher)
            shootFlap.setPosition(flapAngleGoal);
        }

        //ClawArm
        if (gamepad2.left_bumper) {
            //Up
            clawArm.setPower(0.3);
        } else  if (gamepad2.right_bumper){
            //Down
            clawArm.setPower(-0.3);
        } else {
            clawArm.setPower(0);
        }

        //Claw Servo
        if (gamepad2.x) {
            clawServo.setPosition(clawOpen);
        } else {
            clawServo.setPosition(clawClose);
        }

        //IMU
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("1 imu heading", lastAngles.firstAngle);


        telemetry.addData("Shooter RPM", (int) shooterRPM);


    }

    //Stop code
    @Override
    public void stop() {
    }

}
