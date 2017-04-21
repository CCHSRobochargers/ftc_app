package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Go Baby Go!")
public class GoBabyGo extends OpMode {
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor steer;

    TouchSensor leftSteerStop;
    TouchSensor rightSteerStop;

    SpeedMonitor monitor;

    boolean active = false;

    public void init() {
        leftDrive = hardwareMap.dcMotor.get("lD");
        rightDrive = hardwareMap.dcMotor.get("rD");
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        steer = hardwareMap.dcMotor.get("steer");

        leftSteerStop = hardwareMap.touchSensor.get("lT");
        rightSteerStop = hardwareMap.touchSensor.get("rT");

        active = true;
        monitor = new SpeedMonitor(leftDrive, rightDrive, 3000);
        monitor.start();
    }

    public void loop() {
        double speed = -gamepad1.right_stick_y;
        double steer = gamepad1.right_stick_x;

        speed = Range.clip(speed, -1.0, 1.0);
        if (rightSteerStop.isPressed()) {
            steer = Range.clip(steer, -1.0, 0.0);
        } else if (leftSteerStop.isPressed()) {
            steer = Range.clip(steer, 0.0, 1.0);
        } else {
            steer = Range.clip(steer, -1.0, 1.0);
        }

        monitor.setPower(speed);
        this.steer.setPower(steer);

    }

    public void stop() {
        active = false;
    }

    private class SpeedMonitor implements Runnable {
        DcMotor lD;
        DcMotor rD;
        Thread t;
        double currSpeed = 0.0;
        double targetSpeed = 0.0;
        double diff = 0.0;
        long zeroTo60time;

        public SpeedMonitor(DcMotor lD, DcMotor rD, long zeroTo60timeMillisecondsEvenThoughMrVBDoesntWantItThatWay) {
            this.lD = lD;
            this.rD = rD;
            this.lD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            this.rD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            this.zeroTo60time = zeroTo60timeMillisecondsEvenThoughMrVBDoesntWantItThatWay;
        }

        public void start() {
            if (t == null) {
                t = new Thread(this, "SpeedMonitorThread");
                t.start();
                Log.i("Thread status", String.valueOf(t.isAlive()));
            }
        }

        public void run() {
            while (active) {
                double otherDiff = Math.abs(getTargetSpeed() - currSpeed);
                if (otherDiff > 0.01) {
                    diff = Math.abs(getTargetSpeed() - currSpeed);
                    if (Math.abs(getTargetSpeed()) < 0.05) {
                        lD.setPower(0.0);
                        rD.setPower(0.0);
                        currSpeed = 0.0;
                    } else {
                        if (getTargetSpeed() < currSpeed) {
                            currSpeed -= 0.01;
                            lD.setPower(currSpeed);
                            rD.setPower(currSpeed);
                        } else {
                            currSpeed += 0.01;
                            lD.setPower(currSpeed);
                            rD.setPower(currSpeed);
                        }
                    }
                }
                try {
                    Thread.sleep(zeroTo60time / 100);
                } catch (InterruptedException e) {

                }
                Log.i("target", String.valueOf(getTargetSpeed()));
                Log.i("current", String.valueOf(currSpeed));
            }
            Log.i("hi", "tag");
        }

        public synchronized double getTargetSpeed() {
            return targetSpeed;
        }

        public synchronized void setPower(double speed) {
            targetSpeed = speed;
        }
    }
}
