package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "rrAuto", group = "Autonomous")
//@Disabled
public class rrAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor shooter = null;
    private Servo kicker = null;
    double kickerInit = 0.2;
    double kickerTo = 0.56;
    private Servo shootFlap;
    double flapAngle = 0.06; //Higher = Steeper
    private Servo leftLift = null;
    private Servo rightLift = null;
    double leftLiftUp = 1 - 0.92; //0 Top
    double rightLiftUp = 0.89; //1 Top


    @Override
    public void runOpMode() {

        shooter = hardwareMap.get(DcMotor.class, "shooter");
        kicker = hardwareMap.get(Servo.class, "kicker");
        kicker.setPosition(kickerInit);
        shootFlap = hardwareMap.get(Servo.class, "shootFlap");
        shootFlap.setPosition(flapAngle);

        leftLift = hardwareMap.get(Servo.class, "leftLift");
        rightLift = hardwareMap.get(Servo.class, "rightLift");

        leftLift.setPosition(leftLiftUp);
        rightLift.setPosition(rightLiftUp);

        shooter.setDirection(DcMotor.Direction.REVERSE);

        //Initialized
        telemetry.addData("Status", "Initialized");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(5)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(myTrajectory);

    }

}

