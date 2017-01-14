package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Iterator;

@TeleOp(name="Teleop", group="4507")
//@Disabled
public class TeleOp4507 extends OpMode {
    enum Kick {DELAYSTART, DELAYEND, KICKSTART, KICKEND, IDLE}
    enum Index {DELAYSTART, DELAYEND, INDEXSTART, INDEXEND, IDLE}

    Kick currentK;
    Kick nextK;
    Kick chooseK;
    long nowK;
    long delayUntilK;
    long delayTimeK;

    Index currentI;
    Index nextI;
    Index chooseI;
    long nowI;
    long delayUntilI;
    long delayTimeI;


    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor kicker;
    DcMotor sweeper;
    DcMotor capper;

    Servo indexer;
    Servo beaconPusher;
    Servo capBallLock;

    TouchSensor kickerStop;
    TouchSensor indexStart;

    static final int NUM_MOTOR_CONTROLLERS = 3;
    double[] minV = new double[NUM_MOTOR_CONTROLLERS];
//    double[] maxV = new double[NUM_MOTOR_CONTROLLERS];

    @Override
    public void init() {
        currentK = Kick.IDLE;
        nextK = Kick.IDLE;
        chooseK = Kick.IDLE;

        currentI = Index.IDLE;
        nextI = Index.IDLE;
        chooseI = Index.IDLE;

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
        indexer.setPosition(0.35);
        beaconPusher = hardwareMap.servo.get("bPu");
        beaconPusher.setPosition(0.5);
        capBallLock = hardwareMap.servo.get("cBL");
        capBallLock.setPosition(0.92);

        kickerStop = hardwareMap.touchSensor.get("kT");
        indexStart = hardwareMap.touchSensor.get("iT");

        Iterator<VoltageSensor> vSensorItr = hardwareMap.voltageSensor.iterator();
        for (int j = 0; j < NUM_MOTOR_CONTROLLERS && vSensorItr.hasNext(); j++) {
            VoltageSensor v = vSensorItr.next();
            minV[j] = v.getVoltage();
            Log.e("4507", String.format("minV[%d] %f", j, v.getVoltage()));
        }
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
            lSP = 0.2;
            rSP = 0.2;
        } else if (gamepad1.dpad_down) {
            lSP = -0.2;
            rSP = -0.2;
        }

        if (gamepad2.a) {
            sweeper.setPower(sw);
        }

        if (gamepad2.x && (currentK == Kick.IDLE) && (currentI == Index.IDLE) && !gamepad1.a) {
            chooseI = Index.INDEXSTART;
        }
        if (gamepad1.a && (currentK == Kick.IDLE) && (currentI == Index.IDLE) && !gamepad2.x) {
            chooseK = Kick.KICKSTART;
        }

        if (gamepad2.dpad_up) {
            capBallLock.setPosition(0.0);
        } else if (gamepad2.dpad_down) {
            capBallLock.setPosition(1.0);
        }

        if (gamepad2.dpad_left) {
            beaconPusher.setPosition(0.0);
        } else if (gamepad2.dpad_right){
            beaconPusher.setPosition(1.0);
        } else {
            beaconPusher.setPosition(0.5);
        }

        leftDrive.setPower(lSP);
        rightDrive.setPower(rSP);
        capper.setPower(cap);

        switch (currentK) {
            case DELAYSTART:
                nowK = System.currentTimeMillis();
                delayUntilK = nowK + delayTimeK;
                currentK = Kick.DELAYEND;
                break;

            case DELAYEND:
                if (System.currentTimeMillis() >= delayUntilK) {
                    currentK = nextK;
                }
                break;

            case IDLE:
                currentK = chooseK;
                break;

            case KICKSTART:
                kicker.setPower(-1.0);
                if (kickerStop.isPressed()) {
                    currentK = Kick.KICKEND;
                }
                break;

            case KICKEND:
                if (!kickerStop.isPressed()) {
                    kicker.setPower(0.0);
                    currentK = Kick.IDLE;
                    chooseK = Kick.IDLE;
                }
                break;
        }

        switch (currentI) {
            case DELAYSTART:
                nowI = System.currentTimeMillis();
                delayUntilI = nowI + delayTimeI;
                currentI = Index.DELAYEND;
                break;

            case DELAYEND:
                if (System.currentTimeMillis() >= delayUntilI) {
                    currentI = nextI;
                }
                break;

            case IDLE:
                if (indexStart.isPressed()) {
                    chooseI = Index.INDEXSTART;
                }
                currentI = chooseI;
                break;

            case INDEXSTART:
                indexer.setPosition(0.18);
                currentI = Index.DELAYSTART;
                nextI = Index.INDEXEND;
                delayTimeI = 300;
                break;

            case INDEXEND:
                indexer.setPosition(0.35);
                currentI = Index.IDLE;
                chooseI = Index.IDLE;
                break;
        }

        Iterator<VoltageSensor> vSensorItr = hardwareMap.voltageSensor.iterator();
        for (int j = 0; j < NUM_MOTOR_CONTROLLERS && vSensorItr.hasNext(); j++) {
            VoltageSensor v = vSensorItr.next();
            if (v.getVoltage() < minV[j]) {
                minV[j] = v.getVoltage();
                Log.e("4507", String.format("minV[%d] %f", j, v.getVoltage()));
            }
            telemetry.addData("minV", minV[j]);
        }

//        telemetry.addData("left", leftDrive.getCurrentPosition());
//        telemetry.addData("right", rightDrive.getCurrentPosition());
//        telemetry.addData("kick", currentK.toString());
//        telemetry.addData("index", currentI.toString());
//        telemetry.addData("indexer", indexer.getPosition());
        updateTelemetry(telemetry);
    }

    public void stop() {
    }
}
