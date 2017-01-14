/**
 * Created by 4507 Robochargers on
 *          9/ 10/ 16
 *
 */
package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;


/**
 * Created by 4507 Robocahrgers on 9/10/16.
 */

@Autonomous(name="Autonomous", group="4507")  // @TeleOp(...) is the other common choice
//@Disabled
public class Autonomous4507 extends LinearOpMode {

    // DcMotor
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor kicker;
    DcMotor sweeper;
    // Servos
    Servo indexer;
    Servo beaconPusher;
    Servo capBallLock;
    // Sensors
    ModernRoboticsI2cGyro gyro;
    int supposedToBeHeading;
    ColorSensor bColor;
    TouchSensor kickStop;
    ModernRoboticsI2cRangeSensor range;
//    //Switches
    DigitalChannel tileSw;
    DigitalChannel colorSw;
    DigitalChannel beaconSw;
    DigitalChannel shootSw;
//    // Elapsed Time
//    ElapsedTime elapsedTime;

    // Global State Vaiables
    int countsPerYard = 2867;
    int countsPer4Donuts = 17000;
    // These variables are for autonomous.
    boolean red;
    boolean blue;
    boolean diagTile;
    boolean straightTile;
    boolean beaconY;
    boolean beaconN;
    boolean shootY;
    boolean shootN;
    boolean endMove = false;

    void delay(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Log.e("InterruptedException", e.toString());
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Device Interface Module
//        dim = hardwareMap.deviceInterfaceModule.get("dim");
//        dim.setLED(RED_LED_CHANNEL, true);
//        dim.setLED(BLUE_LED_CHANNEL, false);
        // DcMotor
        leftDrive = hardwareMap.dcMotor.get("l");
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive = hardwareMap.dcMotor.get("r");
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        kicker = hardwareMap.dcMotor.get("kick");
        kicker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sweeper = hardwareMap.dcMotor.get("sweep");
        sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Servos
        indexer = hardwareMap.servo.get("ind");
        indexer.setPosition(.35);
        beaconPusher = hardwareMap.servo.get("bPu");
        beaconPusher.setPosition(0.5);
        capBallLock = hardwareMap.servo.get("cBL");
        capBallLock.setPosition(0.92);
        // Sensors
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        bColor = hardwareMap.colorSensor.get("cS");
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
//        beaconStopTouch = hardwareMap.touchSensor.get("bST");
        kickStop = hardwareMap.touchSensor.get("kT");
//        //Switches
        tileSw = hardwareMap.digitalChannel.get("tSw");
        colorSw = hardwareMap.digitalChannel.get("cSw");
        beaconSw = hardwareMap.digitalChannel.get("bSw");
        shootSw = hardwareMap.digitalChannel.get("sSw");
//        if(tileSw.getState()) { //d0
//            diagTile = true;
//            straightTile = false;
//        } else if (!tileSw.getState()) {
//            diagTile = false;
//            straightTile = true;
//        }
        if(colorSw.getState()) { //d1
            red = false;
            blue = true;
        } else if (!colorSw.getState()) {
            red = true;
            blue = false;
        }
        if (beaconSw.getState()) { //d2
            beaconY = true;
            beaconN = false;
        } else if (!beaconSw.getState()) {
            beaconY = false;
            beaconN = true;
        }
        if (shootSw.getState()) { //d3
            shootY = false;
            shootN = true;
        } else if (!shootSw.getState()) {
            shootY = true;
            shootN = false;
        }
//        // Elapsed Time
//        elapsedTime = new ElapsedTime();
        gyro.calibrate();
        while (gyro.isCalibrating() && !isStarted()) {
            idle();
        }
        if (!gyro.isCalibrating()) {
//            dim.setLED(RED_LED_CHANNEL, false);
//            dim.setLED(BLUE_LED_CHANNEL, true);
        }

        // Wait for the robot to start
        waitForStart();


        while (gyro.isCalibrating() && opModeIsActive()) {
            idle();
        }
        if (!gyro.isCalibrating()) {
//            dim.setLED(RED_LED_CHANNEL, false);
//            dim.setLED(BLUE_LED_CHANNEL, true);
        }

        supposedToBeHeading = gyro.getHeading();
        sweep(true);
        capBallLock.setPosition(0.5);
//        delay(10000);
//        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftDrive.setPower(1.0);
//        rightDrive.setPower(1.0);
//        delay(4000);
//        rightDrive.setPower(0.0);
//        leftDrive.setPower(0.0);
        driveStraight(24, 1.0, 450);
        /**/
        shoot(shootY ? 2 : 0);
        if (beaconY) {
            driveTurn(red ? -55 : 55, 0.75, 200);
            driveStraight(red ? 22 : 32, 1.0, 100);
            driveTurn(red ? -45 : 40, 0.75, 200);
            driveStraight(red ? 28 : 30, red ? 8 : 8, 1.0, 100);
            driveTurn(90, 0.75, 100);
            driveTurnWithGyro(supposedToBeHeading);
            if (red) {
                driveTurn(5, 0.5, 200);
            }
            beacon(red ? 0.50 : -1.0, false, 500);
            beacon(red ? 12.0 : -12.0, red ? 1.0 : -1.0, false, 100);
        } else if (!beaconY) {
            driveTurn(red ? 10 : -10, 1.0, 200);
//            driveTurn(red ? -45 : 45, 0.75, 200);
//            driveStraight(25, 1.0, 100);
//            driveTurn(red ? 80 : -80, 0.75, 200);
//            driveStraight(17, 1.0, 100);
//            driveTurn(red ? 100 : -100, 0.75, 200);
//            driveStraight(12, 1.0, 100);
//            driveTurn(red ? 90 : -90, 0.75, 100);
//            driveStraight(60, 1.0, 100);
            driveStraight(24, 1.0, 100);
            driveTurn(red ? 55 : -55, 1.0, 200);
            driveStraight(-72, 1.0, 100);
        }
        sweep(false);
    }









    public void driveStraight(double approxInches, double rangeInches, double speed, long delayMillis) throws InterruptedException {
        boolean endMove = false;
        int lTarget = leftDrive.getCurrentPosition() - (int)(-approxInches * (countsPerYard / 36.0) - 6);
        int rTarget = rightDrive.getCurrentPosition() - (int)(-approxInches * (countsPerYard / 36.0) - 6);
        double[] distances = new double[9];

        leftDrive.setTargetPosition(lTarget);
        rightDrive.setTargetPosition(rTarget);

        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        while ((leftDrive.isBusy() && rightDrive.isBusy()) && opModeIsActive() && !endMove) {
            for (int j = 0; j < distances.length; j++) {
                distances[j] = range.getDistance(DistanceUnit.INCH);
                sleep(75);
            }
            Arrays.sort(distances);
            rangeInches = distances[4];
            idle();
            telemetry.addData("range", rangeInches);
            updateTelemetry(telemetry);
        }

        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
        sleep(delayMillis);
    }

    /**
     * Created on 9/10/16
     * @param inches is how far to go
     * @param speed is how fast to go
     * @param delayMillis is how long to delay
     * @throws InterruptedException allows the sleep() method to be used
     */

    public void driveStraight(double inches, double speed, long delayMillis) throws InterruptedException {
        int lTarget = leftDrive.getCurrentPosition() - (int)(-inches * (countsPerYard / 36.0));
        int rTarget = rightDrive.getCurrentPosition() - (int)(-inches * (countsPerYard / 36.0));

        leftDrive.setTargetPosition(lTarget);
        rightDrive.setTargetPosition(rTarget);

        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        while ((leftDrive.isBusy() && rightDrive.isBusy()) && opModeIsActive()) {
            idle();
        }

        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
        sleep(delayMillis);
    }

    /**
     *
     * @param degrees is how many degrees to turn; when negated turns clockwise, and vice versa
     * @param speed is how fast to turn
     * @param delayMillis is how long to delay
     * @throws InterruptedException allows the sleep() method to be used
     */

    public void driveTurn(int degrees, double speed, long delayMillis) throws InterruptedException {
        int desiredHeading;
        supposedToBeHeading = supposedToBeHeading + degrees;
        if (supposedToBeHeading > 359) {
            supposedToBeHeading = supposedToBeHeading - 360;
        } else if (supposedToBeHeading < 0) {
            supposedToBeHeading = supposedToBeHeading + 360;
        }

        driveTurnWithEncoders(degrees, speed);
        driveTurnWithGyro(supposedToBeHeading);

        sleep(delayMillis);
    }

    /**
     *
     * @param degrees is how many degrees to turn; when negated turns clockwise, and vice versa
     * @param speed is how fast to turn
     */

    public void driveTurnWithEncoders(int degrees, double speed) {
        int desiredHeading = gyro.getHeading() + degrees;
        if (desiredHeading > 359) {
            desiredHeading = desiredHeading - 360;
        } else if (desiredHeading < 0) {
            desiredHeading = desiredHeading + 360;
        }
        int lTarget = leftDrive.getCurrentPosition() + (degrees * (countsPer4Donuts / 1440));
        int rTarget = rightDrive.getCurrentPosition() - (degrees * (countsPer4Donuts / 1440));

        leftDrive.setTargetPosition(lTarget);
        rightDrive.setTargetPosition(rTarget);

        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        while ((leftDrive.isBusy() && rightDrive.isBusy()) && opModeIsActive()) {
//            gyroError = gyro.getHeading() - desiredHeading;
//            if (gyroError > 180) {
//                gyroError = 360 - gyroError;
//            }
//            if (gyroError < -180) {
//                gyroError = 360 + gyroError;
//            }
//            leftDrive.setPower(Range.clip(0 - (gyroError * 0.03), -1, 1));
//            rightDrive.setPower(Range.clip(0 + (gyroError * 0.03), -1, 1));
            idle();
        }

    }

    /**
     *
     * @param desiredHeading is how many degrees to turn; when negated turns clockwise, and vice versa
     */

    public void driveTurnWithGyro (int desiredHeading) {
        int gyroError;
        boolean exitTurn = false;
        long pre = System.currentTimeMillis();

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (!exitTurn && opModeIsActive()) {
            long now = System.currentTimeMillis();
            gyroError = gyro.getHeading() - desiredHeading;
            if (gyroError > 180) {
                gyroError = 360 - gyroError;
            }
            if (gyroError < -180) {
                gyroError = 360 + gyroError;
            }
            if (gyroError == 0) {
                exitTurn = true;
            }
            if (pre + 1000 < now) {
                exitTurn = true;
            }
            leftDrive.setPower(Range.clip(0 - (0.04 + (gyroError * 0.03)), -0.25, 0.25));
            rightDrive.setPower(Range.clip(0 + (0.04 + (gyroError * 0.03)), -0.25, 0.25));
            idle();
        }
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * TODO: This is going to be an absolute pain in the butt
     * @param speed is the speed to go
     * @param colorRed is whether or not we are looking for red
     */

    public void beacon(double speed, boolean colorRed, long delay) {
        boolean stop = false;

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        while (!stop && opModeIsActive()) {
            if (red) {
                if (bColor.red() > bColor.blue()) {
                    Log.i("Stopped at red", "Yay");
                    try {
                        driveStraight(-3, 0.5, 200);
                    } catch (InterruptedException e) {
                        Log.e("ColorError", e.toString());
                    }
                    leftDrive.setPower(0.0);
                    rightDrive.setPower(0.0);
                    stop = true;
                }
            } else if (!red) {
                if (bColor.blue() > bColor.red()) {
                    Log.i("Stopped at blue", "Yay");
                    leftDrive.setPower(0.0);
                    rightDrive.setPower(0.0);
                    stop = true;
                }
            }
            telemetry.addData("BlueVal", bColor.blue());
            telemetry.addData("RedVal", bColor.red());
            telemetry.addData("ColorVal", bColor.argb());
            updateTelemetry(telemetry);
            Log.i("ColorVal", String.valueOf(bColor.argb()));
            idle();
        }
        beaconPusher.setPosition(0.0);
        sleep(2500);
        beaconPusher.setPosition(1.0);
        sleep(2500);
        beaconPusher.setPosition(0.5);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(delay);
    }

    public void beacon(double distB4Press, double speed, boolean colorRed, long delay) {
        boolean stop = false;
        int target = leftDrive.getCurrentPosition() - (int)(-distB4Press * countsPerYard / 36);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        while (!stop && opModeIsActive()) {
            if (red ? leftDrive.getCurrentPosition() > target : leftDrive.getCurrentPosition() < target) {
                if (red) {
                    if (bColor.red() > bColor.blue()) {
                        Log.i("Stopped at red", "Yay");
                        try {
                            driveStraight(-3, 0.5, 200);
                        } catch (InterruptedException e) {
                            Log.e("ColorError", e.toString());
                        }
                        leftDrive.setPower(0.0);
                        rightDrive.setPower(0.0);
                        stop = true;
                    }
                } else if (!red) {
                    if (bColor.blue() > bColor.red()) {
                        Log.i("Stopped at blue", "Yay");
                        leftDrive.setPower(0.0);
                        rightDrive.setPower(0.0);
                        stop = true;
                    }
                }
            }
            telemetry.addData("BlueVal", bColor.blue());
            telemetry.addData("RedVal", bColor.red());
            telemetry.addData("ColorVal", bColor.argb());
            updateTelemetry(telemetry);
            Log.i("ColorVal", String.valueOf(bColor.argb()));
            idle();
        }
        beaconPusher.setPosition(0.0);
        sleep(2500);
        beaconPusher.setPosition(1.0);
        sleep(2500);
        beaconPusher.setPosition(0.5);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(delay);
    }

    /**
     *
     * @param times is the number of times to shoot
     */

    public void shoot(int times) {
        for (int c = 0; c < times; c++) {
            kick();
            index();
            sleep(500);
        }
    }

    /**
     * kicks a particle once
     */

    public void kick() {
        kicker.setPower(-1.0);
        while (opModeIsActive() && !kickStop.isPressed()) { idle(); }
        while (opModeIsActive() && kickStop.isPressed()) { idle(); }
        kicker.setPower(0.0);
    }

    /**
     * indexes a particle once
     */

    public void index() {
        indexer.setPosition(0.63);
        sleep(500);
    }

    /**
     * @param onOff is a boolean to determine whether to stop or start the sweeper
     */

    public void sweep(boolean onOff) {
        if (onOff) {
            sweeper.setPower(-1.0);
        } else if (!onOff) {
            sweeper.setPower(0.0);
        }
    }
}
