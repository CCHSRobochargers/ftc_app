/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop", group="4507")  // @Autonomous(...) is the other common choice
//@Disabled
public class TeleOp4507 extends OpMode {

    DcMotor lDrive;
    DcMotor rDrive;
    DcMotor sweeper;
//    DcMotor kicker;
    Servo beaconIn;
    Servo beaconOut;
    UltrasonicSensor bUltra;
    ColorSensor bColor;
    boolean sweeperOn = false;

    @Override
    public void init() {
        lDrive = hardwareMap.dcMotor.get("lD");
        rDrive = hardwareMap.dcMotor.get("rD");
        sweeper = hardwareMap.dcMotor.get("sweep");
//        kicker = hardwareMap.dcMotor.get("kick");
        lDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        lDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        kicker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        beaconIn = hardwareMap.servo.get("bI");
        beaconOut = hardwareMap.servo.get("bO");
        beaconIn.setPosition(.23);
        beaconOut.setPosition(.97);
        bUltra = hardwareMap.ultrasonicSensor.get("bU");
        bColor = hardwareMap.colorSensor.get("bC");
    }

    @Override
    public void loop() {
        double lSP = gamepad1.left_stick_y;
        double rSP = gamepad1.right_stick_y;


        lSP = Range.clip(lSP, -1.0, 1.0);
        rSP = Range.clip(rSP, -1.0, 1.0);

        if (gamepad2.a) {
            sweeperOn = true;
        } else if (gamepad2.b) {
            sweeperOn = false;
        }

        lDrive.setPower(lSP);
        rDrive.setPower(rSP);
        if (sweeperOn) {
            sweeper.setPower(1.0);
        } else if (!sweeperOn) {
            sweeper.setPower(0.0);
        }

//        if (gamepad2.x) {
//            kicker
//        }

        if (gamepad1.dpad_up) {
            beaconIn.setPosition(Range.clip(beaconIn.getPosition() + 0.01, 0.00, 1.00));
        } else if (gamepad1.dpad_down) {
            beaconIn.setPosition(Range.clip(beaconIn.getPosition() - 0.01, 0.00, 1.00));
        }

        if (gamepad2.dpad_up) {
            beaconOut.setPosition(Range.clip(beaconOut.getPosition() + 0.01, 0.00, 1.00));
        } else if (gamepad2.dpad_down) {
            beaconOut.setPosition(Range.clip(beaconOut.getPosition() - 0.01, 0.00, 1.00));
        }
        telemetry.addData("in", beaconIn.getPosition());
        telemetry.addData("out", beaconOut.getPosition());
        updateTelemetry(telemetry);
    }
}
