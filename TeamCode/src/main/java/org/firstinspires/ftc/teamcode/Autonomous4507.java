/**
 * Created by 4507 Robochargers on
 *          9/ 10/ 16
 *
 *9/10/16:
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



/**
 * Created by 4507 Robocahrgers on 9/10/16.
 */

@Autonomous(name="Autonomous", group="4507")  // @TeleOp(...) is the other common choice
//@Disabled
public class Autonomous4507 extends LinearOpMode {

    // TODO: Need to test everything new
    // TODO: (Which is practically everything)
    // TODO: Look and fix code at other "TODO:" statements

    // Device Interface Module
//    DeviceInterfaceModule dim;
//    final int BLUE_LED_CHANNEL = 0;
//    final int RED_LED_CHANNEL = 1;
    // DcMotor
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor kicker;
    DcMotor sweeper;
    // Servos
    Servo indexer;
    Servo beaconPusher;
    // Sensors
    ModernRoboticsI2cGyro gyro;
    int supposedToBeHeading;
    ColorSensor bColor;
    TouchSensor kickStop;
    ModernRoboticsI2cRangeSensor range;
//    //Switches
//    DigitalChannel tileSw;
//    DigitalChannel colorSw;
//    DigitalChannel beaconSw;
//    // Elapsed Time
//    ElapsedTime elapsedTime;

    // Global State Vaiables
    int countsPerYard = 2867;
    int countsPer4Donuts = 17000;
    double fastSpeed;
    double mediumSpeed;
    double slowSpeed;
    long fastSpDelay;
    long slowSpDelay;

    // These variables are for autonomous.
    boolean red = false;
    boolean blue;
    boolean diagTile;
    boolean straightTile;
    boolean beaconY;
    boolean beaconN;
    boolean endMove = false;

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
        indexer.setPosition(0.8);
        beaconPusher = hardwareMap.servo.get("bPu");
        beaconPusher.setPosition(0.5);
        // Sensors
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        bColor = hardwareMap.colorSensor.get("cS");
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
//        beaconStopTouch = hardwareMap.touchSensor.get("bST");
        kickStop = hardwareMap.touchSensor.get("kT");
//        //Switches
//        tileSw = hardwareMap.digitalChannel.get("tSw");
//        colorSw = hardwareMap.digitalChannel.get("cSw");
//        beaconSw = hardwareMap.digitalChannel.get("bSw");
//        if(tileSw.getState()) {
//            diagTile = true;
//            straightTile = false;
//        } else if (!tileSw.getState()) {
//            diagTile = false;
//            straightTile = true;
//        }
//        if(colorSw.getState()) {
//            red = true;
//            blue = false;
//        } else if (!colorSw.getState()) {
//            red = false;
//            blue = true;
//        }
//        if (beaconSw.getState()) {
//            beaconY = true;
//            beaconN = true;
//        } else if (!beaconSw.getState()) {
//            beaconY = false;
//            beaconN = true;
//        }
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


//        if (straightTile) {
//            driveStraight(move1a + move1b, fastSpeed, fastSpDelay);
//        } else if (diagTile) {
//            driveStraight(move0a, fastSpeed, fastSpDelay);
//            driveTurn(turn0a, mediumSpeed, slowSpDelay);
//            driveStraight(move0b, fastSpeed, fastSpDelay);
//            driveTurn(turn0b, mediumSpeed, slowSpDelay);
//            driveStraight(move1b, fastSpeed, fastSpDelay);
//        }
//        if (beaconY) {
//            driveTurn(turn1, mediumSpeed, slowSpDelay);
//            driveStraight(move2, fastSpeed, fastSpDelay);
//            driveTurn(turn2, mediumSpeed, slowSpDelay);
//            driveStraight(move3, slowSpeed,slowSpDelay);
//            // add button code
//            driveStraight(move4a, fastSpeed, fastSpDelay);
//            driveStraight(move4b, slowSpeed, slowSpDelay);
//            // add button code
//            driveStraight(move5, fastSpeed, fastSpDelay);
//            driveTurn(turn3, mediumSpeed, slowSpDelay);
//            driveStraight(move6, fastSpeed, fastSpDelay);
//        }

        while (gyro.isCalibrating() && opModeIsActive()) {
            idle();
        }
        if (!gyro.isCalibrating()) {
//            dim.setLED(RED_LED_CHANNEL, false);
//            dim.setLED(BLUE_LED_CHANNEL, true);
        }
        supposedToBeHeading = gyro.getHeading();
        sweep(true);
        driveStraight(24, 1.0, 400);
        shoot(2);
        driveTurn(45, 0.75, 100);
        driveStraight(32, 1.0, 100);
        driveTurn(45, 0.75, 1000);
        driveStraight(20, 8, 1.0, 100);
        driveTurn(90, 0.75, 100);
        driveTurnWithGyro(supposedToBeHeading);
        beacon(red ? 1.0 : -1.0, false, 500);
        driveStraight(red ? 12 : -12, 1.0, 100);
        beacon(red ? 1.0 : -1.0, false, 100);
        sweep(false);
    }









    public void driveStraight(double approxInches, int rangeInches, double speed, long delayMillis) throws InterruptedException {
        boolean endMove = false;
        int lTarget = leftDrive.getCurrentPosition() - (int)(-approxInches * (countsPerYard / 36.0) - 6);
        int rTarget = rightDrive.getCurrentPosition() - (int)(-approxInches * (countsPerYard / 36.0) - 6);

        leftDrive.setTargetPosition(lTarget);
        rightDrive.setTargetPosition(rTarget);

        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        while ((leftDrive.isBusy() && rightDrive.isBusy()) && opModeIsActive() && !endMove) {
            if (range.getDistance(DistanceUnit.INCH) < rangeInches) {
                endMove = true;
                leftDrive.setPower(0.0);
                rightDrive.setPower(0.0);
            }
            idle();
            telemetry.addData("range", range.getDistance(DistanceUnit.INCH));
            updateTelemetry(telemetry);
        }

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

        desiredHeading = driveTurnWithEncodersRETURNSdesiredHeading(degrees, speed);
        driveTurnWithGyro(desiredHeading);

        sleep(delayMillis);
    }

    /**
     *
     * @param degrees is how many degrees to turn; when negated turns clockwise, and vice versa
     * @param speed is how fast to turn
     */

    public int driveTurnWithEncodersRETURNSdesiredHeading(int degrees, double speed) {
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
        return desiredHeading;
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
            if (colorRed) {
                if (bColor.red() > bColor.blue()) {
                    leftDrive.setPower(0.0);
                    rightDrive.setPower(0.0);
                    stop = true;
                }
            } else if (!colorRed) {
                if (bColor.blue() > bColor.red()) {
                    leftDrive.setPower(0.0);
                    rightDrive.setPower(0.0);
                    stop = true;
                }
            }
        }
        beaconPusher.setPosition(0.0);
        sleep(3000);
        beaconPusher.setPosition(1.0);
        sleep(3000);
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
