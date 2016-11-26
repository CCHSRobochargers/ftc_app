/*
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
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
    enum KickIndex {DELAYSTART, DELAYEND, KICKSTART, KICKEND, INDEXSTART, INDEXEND, IDLE}

    KickIndex currentKI;
    KickIndex nextKI;
    KickIndex chooseKI;
    long now;
    long delayUntil;
    long delayTime;

    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor kicker;
    DcMotor sweeper;
    DcMotor capper;

    Servo indexer;
    Servo beaconPusher;


    TouchSensor kickerStop;

    boolean sweepButtonLockOut = false;
    boolean sweeperOn = false;
    boolean kickerOn = false;

//
    @Override
    public void init() {
        currentKI = KickIndex.IDLE;
        nextKI = KickIndex.IDLE;
        chooseKI = KickIndex.IDLE;

        leftDrive = hardwareMap.dcMotor.get("l");
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive = hardwareMap.dcMotor.get("r");
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        kicker = hardwareMap.dcMotor.get("kick");
        kicker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sweeper = hardwareMap.dcMotor.get("sweep");
        sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        capper = hardwareMap.dcMotor.get("cap");
        capper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        indexer = hardwareMap.servo.get("ind");
        indexer.setPosition(0.8);
        beaconPusher = hardwareMap.servo.get("bPu");
        beaconPusher.setPosition(0.5);

        kickerStop = hardwareMap.touchSensor.get("kT");
    }

    @Override
    public void loop() {
        double lSP = -gamepad1.left_stick_y;
        double rSP = -gamepad1.right_stick_y;
        double sw = gamepad2.left_stick_y;
        double cap = gamepad2.right_stick_y;


        lSP = Range.clip(lSP, -1.0, 1.0);
        rSP = Range.clip(rSP, -1.0, 1.0);
        sw = Range.clip(sw, -1.0, 1.0);
        cap = Range.clip(cap, -1.0, 1.0);

        if (gamepad1.dpad_up) {
            lSP = -1.0;
            rSP = -1.0;
        } else if (gamepad1.dpad_down) {
            lSP = 1.0;
            rSP = 1.0;
        }

        if (gamepad2.a) {
            sweeper.setPower(sw);
        }

        if (gamepad2.x && currentKI == KickIndex.IDLE) {
            chooseKI = KickIndex.INDEXSTART;
        }
        if (gamepad1.a && currentKI == KickIndex.IDLE && !gamepad2.x) {
            chooseKI = KickIndex.KICKSTART;
        }

        leftDrive.setPower(lSP);
        rightDrive.setPower(rSP);
        capper.setPower(cap);

        switch (currentKI) {
            case DELAYSTART:
                now = System.currentTimeMillis();
                delayUntil = now + delayTime;
                currentKI = currentKI.DELAYEND;
                break;

            case DELAYEND:
                if (System.currentTimeMillis() >= delayUntil) {
                    currentKI = nextKI;
                }
                break;

            case IDLE:
                currentKI = chooseKI;
                break;

            case KICKSTART:
                kicker.setPower(-1.0);
                currentKI = KickIndex.KICKEND;
                break;

            case KICKEND:
                if (kickerStop.isPressed()) {
                    kicker.setPower(0.0);
                    currentKI = KickIndex.INDEXSTART;
                }
                break;

            case INDEXSTART:
                indexer.setPosition(0.63);
                currentKI = KickIndex.DELAYSTART;
                nextKI = KickIndex.INDEXEND;
                delayTime = 300;
                break;

            case INDEXEND:
                indexer.setPosition(0.8);
                currentKI = KickIndex.DELAYSTART;
                nextKI = KickIndex.IDLE;
                chooseKI = KickIndex.IDLE;
                delayTime = 500;
                break;
        }




        telemetry.addData("left", leftDrive.getCurrentPosition());
        telemetry.addData("right", rightDrive.getCurrentPosition());
        telemetry.addData("indexer", indexer.getPosition());
        updateTelemetry(telemetry);
    }
}
