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

    DcMotor l1;
    DcMotor l2;
    DcMotor r1;
    DcMotor r2;
//
    @Override
    public void init() {
        l1 = hardwareMap.dcMotor.get("l1");
        l1.setDirection(DcMotorSimple.Direction.REVERSE);
        l2 = hardwareMap.dcMotor.get("l2");
        l2.setDirection(DcMotorSimple.Direction.REVERSE);
        r1 = hardwareMap.dcMotor.get("r1");
        r2 = hardwareMap.dcMotor.get("r2");
//        r2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double lSP = gamepad1.left_stick_y;
        double rSP = gamepad1.right_stick_y;


        lSP = Range.clip(lSP, -1.0, 1.0);
        rSP = Range.clip(rSP, -1.0, 1.0);

        if (gamepad1.dpad_up) {
            lSP = 0.08;
            rSP = 0.08;
        } else if (gamepad1.dpad_down) {
            lSP = -0.08;
            rSP = -0.08;
        }

//        if (gamepad2.a) {
//            sweeperOn = true;
//        } else if (gamepad2.b) {
//            sweeperOn = false;
//        }

        l1.setPower(lSP);
        l2.setPower(lSP);
        r1.setPower(rSP);
        r2.setPower(rSP);
//        if (sweeperOn) {
//            sweeper.setPower(1.0);
//        } else if (!sweeperOn) {
//            sweeper.setPower(0.0);
//        }

        telemetry.addData("left1", l1.getCurrentPosition());
        telemetry.addData("right1", r1.getCurrentPosition());
        updateTelemetry(telemetry);
    }
}
