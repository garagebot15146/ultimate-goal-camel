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

@TeleOp(name="servoTester", group="Iterative Opmode")
//@Disabled
public class servoTester extends OpMode
{
    //Declare runtime variable
    private ElapsedTime runtime = new ElapsedTime();
    double currentTime;

    //Declare servo object
    SimpleServo kicker;

    double servoInit = 0.5;

    //Initialize
    @Override
    public void init() {
        //Define servo hardware maps
        kicker = new SimpleServo(hardwareMap, "kicker") ;

        //Initialized
        telemetry.addData("Status", "Initialized");
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

        /////////////
        //GAMEPAD 1//
        /////////////

        //Use dpad to move servo
        if(gamepad1.a) {
            //Fast
            if(gamepad1.dpad_up) {
                servoInit = servoInit + 0.001;
            } else if (gamepad1.dpad_down) {
                servoInit = servoInit - 0.001;
            }
        } else {
            //Slow
            if(gamepad1.dpad_up) {
                servoInit = servoInit + 0.0001;
            } else if (gamepad1.dpad_down) {
                servoInit = servoInit - 0.0001;
            }
        }

        //Correct for out of bounds numbers
        if (servoInit < 0) {
            servoInit = 0;
        } else if (servoInit > 1) {
            servoInit = 1;
        }

        telemetry.addData("Servo Position", servoInit);

        //CHANGE THIS if using a different servo
        kicker.setPosition(servoInit);

    }

    //Stop code
    @Override
    public void stop() {
    }

}
