/*
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor kicker;
    DcMotor sweeper;

    boolean sweeperOn = false;
    boolean kickerOn = false;

//
    @Override
    public void init() {
        leftDrive = hardwareMap.dcMotor.get("l1");
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive = hardwareMap.dcMotor.get("r1");
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        kicker = hardwareMap.dcMotor.get("kick");
        kicker.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sweeper = hardwareMap.dcMotor.get("sweep");
        sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        if (gamepad2.a) {
            sweeperOn = true;
        } else if (gamepad2.b) {
            sweeperOn = false;
        }
        if (gamepad2.x) {
            kickerOn = true;
        } else if (gamepad2.y) {
            kickerOn = false;
        }

        leftDrive.setPower(lSP);
        rightDrive.setPower(rSP);

        if (sweeperOn) {
            sweeper.setPower(1.0);
        } else if (!sweeperOn) {
            sweeper.setPower(0.0);
        }

        if (kickerOn) {
            if (!kicker.isBusy()) {
                kicker.setTargetPosition(kicker.getCurrentPosition() + 1120);
                kicker.setPower(1.0);
            }
        } else if (!kickerOn) {
            kicker.setPower(0.0);
        }

        telemetry.addData("left", leftDrive.getCurrentPosition());
        telemetry.addData("right", rightDrive.getCurrentPosition());
        updateTelemetry(telemetry);
    }
}
