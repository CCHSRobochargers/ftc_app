package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.AutoConfig;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by CAD1 on 1/28/2017.
 */
@Autonomous(name = "Auto4507", group = "4507")
@Disabled
public class Auto4507 extends LinearOpMode {

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
    ColorSensor bColor;
    TouchSensor kickStop;
    ModernRoboticsI2cRangeSensor range;
    ModernRoboticsI2cRangeSensor sideRange;
    ModernRoboticsAnalogOpticalDistanceSensor redODS;
    ModernRoboticsAnalogOpticalDistanceSensor blueODS;
    double redODSGrayVal;
    double blueODSGrayVal;
    //Switch Variables
    boolean red;
    boolean beaconY;
    boolean shoot2;
    boolean vortexTile;
    boolean hitCapBall;
    //Global State Variables
    int currentHeading;
    int desiredHeading;
    int countsPerYard = 2867;
    int countsPer4Donuts = 18186;
    int countsPerDonut = countsPer4Donuts / 4;
    int countEndDelta = countsPerYard / 144;
    static double           HEADING_THRESHOLD       = 3.0;    // Degrees that is close enough
    static final double     P_TURN_COEFF            = 0.05;          //0.02;    // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.2;     // Larger is more responsive, but also less stable

    @Override
    public void runOpMode() throws InterruptedException {
        //Switch Variables
        red = AutoConfig.redAlliance;
        Log.i("Red?", String.valueOf(AutoConfig.redAlliance));
        beaconY = AutoConfig.doBeacon;
        Log.i("Beacon?", String.valueOf(AutoConfig.doBeacon));
        shoot2 = AutoConfig.shootTwo;
        vortexTile = AutoConfig.vortexTile;
        hitCapBall = AutoConfig.hitCapBall;
        //DcMotors
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
        capBallLock.setPosition(1.0);        // Sensors
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        bColor = hardwareMap.colorSensor.get("cS");
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        sideRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sR");
        sideRange.setI2cAddress(I2cAddr.create7bit(0x42));
        kickStop = hardwareMap.touchSensor.get("kT");
        redODS = hardwareMap.get(ModernRoboticsAnalogOpticalDistanceSensor.class, "rO");
        blueODS = hardwareMap.get(ModernRoboticsAnalogOpticalDistanceSensor.class, "bO");
        redODS.enableLed(true);
        blueODS.enableLed(true);
        redODSGrayVal = redODS.readRawVoltage();
        blueODSGrayVal = blueODS.readRawVoltage();
        Log.i("redODSVal", String.valueOf(redODSGrayVal));
        Log.i("blueODSVal", String .valueOf(blueODSGrayVal));
        // Calibrate Gyro
        gyro.calibrate();
        Log.i("Gyro Status:", "Is Calibrating...");
        while (gyro.isCalibrating() && !isStarted() && opModeIsActive()) {
            idle();
        }
        if (!gyro.isCalibrating()) {
            Log.i("Gyro Status:", "Calibrated Successfully!");
        }

        //Wait for start
        waitForStart();

        //Check if gyro is still calibrating
        if (gyro.isCalibrating()) {
            while (gyro.isCalibrating()) idle();
            Log.i("Gyro Status:", "Calibrated Successfully!");
        }

        currentHeading = gyro.getIntegratedZValue();
        desiredHeading = gyro.getIntegratedZValue();
        Log.i("Gyro Status", "Current Heading is: " + String.valueOf(currentHeading));

        if (!beaconY) {
            sleep(1000 * AutoConfig.delay);
        }

        //Turn on sweeper
        sweeper.setPower(-1.0);

        //Flip up cap ball holder
        capBallLock.setPosition(0.5);

        //Drive straight to shoot
        if (opModeIsActive()) {
            if (!beaconY && !vortexTile) {
                driveStraight(currentHeading, 42, 1.0, 300);
            } else {
                driveStraight(currentHeading, 24, 1.0, 300);
            }
        }

        //Shoot twice
        shoot(shoot2 ? 2 : 1);

        sweeper.setPower(0.0);

        if (beaconY) {
            //Drive Turn parallel to tape lines
            if (opModeIsActive()) {
//                driveTurn(red ? 45 : -45, 0.75, 300);
                currentHeading += red ? 45: -45;
                gyroTurn(0.25, currentHeading);
            }

            //Drive forward
            if (opModeIsActive()) {
                driveStraight(currentHeading, 23, 1.0, 300);
            }

            //Turn to wall
            if(opModeIsActive()) {
//                driveTurn(red ? 45 : -45, 0.75, 300);
                currentHeading += red ? 45: -45;
                gyroTurn(0.25, currentHeading);
            }

            //Drive straight partially to wall
            if (opModeIsActive()) {
                driveStraight(currentHeading, 16, 1.0, 300);
            }

            //Drive straight to wall with range
            if (opModeIsActive()) {
                driveStraight(currentHeading, 12, 10, 0.4, 300);
            }

            //Turn to Beacon
            if (opModeIsActive()) {
//                driveTurn(red ? -90 : 90, 0.75, 300);
                currentHeading += - 90;
                gyroTurn(0.20, currentHeading);
            }

//            if (opModeIsActive()) {
//                gyroHold(0.2, 0, 0.5);
////                gyroFix(0, 300);
//            }

            //Drive to first white line
            if (opModeIsActive()) {
                driveToWhiteLine(red ? 0.3 : -0.3, 300);
            }

            if (opModeIsActive() && !red) {
                driveStraight(currentHeading, -3, 0.25, 200);
            }

            //Drive to first beacon
            if (opModeIsActive()) {
                beacon(red ? 0.15 : -0.15, 300);
            }

            //Wind out beacon pusher
            if (opModeIsActive()) {
                moveButtonPusherOut(3500);
            }

            //Wind in beacon pusher
            if (opModeIsActive()) {
                moveButtonPusherIn(1000);
            }

            //Drive forward between beacons
            if (opModeIsActive()) {
                driveStraight(currentHeading, red ? 32 : -32, 1.0, 300);
            }

            //Drive to second white line
            if (opModeIsActive()) {
                driveToWhiteLine(red ? 0.3 : -0.3, 300);
            }

            if (opModeIsActive() && !red) {
                driveStraight(currentHeading, -3, 0.25, 200);
            }

            //Drive to second beacon
            if (opModeIsActive()) {
                beacon(red ? 0.15 : -0.15, 300);
            }
//
            //Wind out beacon pusher
            if(opModeIsActive()) {
                moveButtonPusherOut(2500);
            }

            //Wind in beacon pusher
            if (opModeIsActive()) {
                moveButtonPusherIn(1000);
            }
        } else {
            if (vortexTile) {
                if (hitCapBall) {
                    // Turn
                    if (opModeIsActive()) {
                        currentHeading += red ? 10 : -10;
                        gyroTurn(0.75, currentHeading);
                    }

                    // Drive and push cap ball and park
                    if (opModeIsActive()) {
                        driveStraight(currentHeading, 36, 1.0, 100);
                    }
                } else {
                    //Backup
                    if (opModeIsActive()) {
                        driveStraight(currentHeading, -24, 1.0, 300);
                    }
                }
            } else {
                if (hitCapBall) {
                    if (opModeIsActive()) {
                        driveStraight(currentHeading, 24, 1.0, 300);
                    }
                } else {
                    if (opModeIsActive()) {
                        driveStraight(currentHeading, -42, 1.0, 300);
                    }
                }
            }

        }

    }

    public void driveStraight(double currentHeading, double approxInches, double rangeInches, double speed, long delayMillis) throws InterruptedException {
        boolean endMove = false;
        double rngInches;
        double prevRngInches = 0.0;
        int lTarget = leftDrive.getCurrentPosition() - (int)(-approxInches * (countsPerYard / 36.0) - 6);
        int rTarget = rightDrive.getCurrentPosition() - (int)(-approxInches * (countsPerYard / 36.0) - 6);
        int gyroError;
        double error;

        leftDrive.setTargetPosition(lTarget);
        rightDrive.setTargetPosition(rTarget);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        Log.i("gyroStraightHeading", String.valueOf(currentHeading));

        rngInches = range.getDistance(DistanceUnit.INCH);
        while ((leftDrive.isBusy() && rightDrive.isBusy()) && opModeIsActive() && !endMove) {

            if (prevRngInches != rngInches) {
                prevRngInches = rngInches;
            }
            rngInches = range.getDistance(DistanceUnit.INCH);
            Log.i("range", Double.toString(rngInches));
            updateTelemetry(telemetry);
            if ((rngInches > 3.0) && (rngInches <= rangeInches)) { //&& (prevRngInches <= rangeInches)) {
                endMove = true;
                leftDrive.setPower(0.0);
                rightDrive.setPower(0.0);
            }

//            error = currentHeading - gyro.getIntegratedZValue();
//            while (error > 180)  error -= 360;
//            while (error <= -180) error += 360;
//
//            Log.i("Straight", String.valueOf(gyro.getIntegratedZValue()));
//
//            leftDrive.setPower(Range.clip(speed - (error * P_DRIVE_COEFF), -1.0, 1.0));
//            rightDrive.setPower(Range.clip(speed + (error * P_DRIVE_COEFF), -1.0, 1.0));
            idle();
        }
        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
        sleep(delayMillis);
    }

    public void driveStraight(double currentHeading, double inches, double speed, long delayMillis) throws InterruptedException {
        int lTarget = leftDrive.getCurrentPosition() - (int)(-inches * (countsPerYard / 36.0));
        int rTarget = rightDrive.getCurrentPosition() - (int)(-inches * (countsPerYard / 36.0));
        double error;

        Log.i("gyroStraightHeading", String.valueOf(currentHeading));

        leftDrive.setTargetPosition(lTarget);
        rightDrive.setTargetPosition(rTarget);

        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        while ((leftDrive.isBusy() && rightDrive.isBusy()) && opModeIsActive()) {
//            error = currentHeading - gyro.getIntegratedZValue();
//            while (error > 180)  error -= 360;
//            while (error <= -180) error += 360;
//
//            Log.i("gyroStraight", String.valueOf(gyro.getIntegratedZValue()));
//
//            leftDrive.setPower(Range.clip(speed - (error * P_DRIVE_COEFF), -1.0, 1.0));
//            rightDrive.setPower(Range.clip(speed + (error * P_DRIVE_COEFF), -1.0, 1.0));
            idle();
        }

        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
        sleep(delayMillis);
    }

    public void gyroFix(int angleToGoTo, long delay) {
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (gyro.getIntegratedZValue() < angleToGoTo) {
            leftDrive.setPower(0.1);
            rightDrive.setPower(-0.1);
        } else if (gyro.getIntegratedZValue() > angleToGoTo) {
            leftDrive.setPower(-0.1);
            rightDrive.setPower(0.1);
        }
        while (opModeIsActive() && gyro.getIntegratedZValue() != angleToGoTo +- 1 && gyro.getIntegratedZValue() != angleToGoTo) {
            if (gyro.getIntegratedZValue() < angleToGoTo) {
                leftDrive.setPower(-0.1);
                rightDrive.setPower(0.1);
            } else if (gyro.getIntegratedZValue() > angleToGoTo) {
                leftDrive.setPower(0.1);
                rightDrive.setPower(-0.1);
            }
        }

        sleep(delay);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void gyroTurn (double speed, double angle) {

        Log.i("gyroTurn angle", String.valueOf(angle));
        Log.i("gyroTurn before", String.valueOf(gyro.getIntegratedZValue()));
        // keep looping while we are still active, and not on heading.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
        Log.i("gyroTurn after", String.valueOf(gyro.getIntegratedZValue()));

        double saveHeadingThreshold = HEADING_THRESHOLD;
        HEADING_THRESHOLD = 1;
        gyroHold(speed, angle, 0.75);
        HEADING_THRESHOLD = saveHeadingThreshold;
        Log.i("gyroTurn held", String.valueOf(gyro.getIntegratedZValue()));
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        error = angle - gyro.getIntegratedZValue();
        while (error > 180)  error -= 360;
        while (error <= -180) error += 360;

        // determine turn power based on +/- error

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = Range.clip(error * PCoeff, -1, 1);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        leftDrive.setPower(leftSpeed);
        rightDrive.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        sleep(250);
    }

//    public void driveTurn(int degrees, double speed, long delayMillis) throws InterruptedException {
//        int gyroError;
//        desiredHeading = desiredHeading - degrees;
//        if(desiredHeading >= 360) {
//            desiredHeading = desiredHeading - 360;
//        } else if (desiredHeading < 0) {
//            desiredHeading = desiredHeading + 360;
//        }
//        gyroError = gyro.getHeading() - desiredHeading;
//        if (gyroError > 180) {
//            gyroError = 360 - gyroError;
//        }
//        if (gyroError < -180) {
//            gyroError = 360 + gyroError;
//        }
//        Log.i("gyroError", String.valueOf(gyroError));
//        Log.i("desiredHeading", String.valueOf(desiredHeading));
//        Log.i("gyroVal", String.valueOf(gyro.getHeading()));
//        int leftTarget = leftDrive.getCurrentPosition() - (int)((countsPerDonut / 360.0) * gyroError);
//        int rightTarget = rightDrive.getCurrentPosition() + (int)((countsPerDonut / 360.0) * gyroError);
//        leftDrive.setTargetPosition(leftTarget);
//        rightDrive.setTargetPosition(rightTarget);
//        leftDrive.setPower(speed);
//        rightDrive.setPower(speed);
//        while (opModeIsActive() && (Math.abs(leftTarget -leftDrive.getCurrentPosition()) < countEndDelta) &&
//                (Math.abs(rightTarget - rightDrive.getCurrentPosition()) < countEndDelta)){
//            idle();
//        }
//        sleep(delayMillis);
//        Log.i("gyroValAfter", String.valueOf(gyro.getHeading()));
//    }

    public void driveTurn(int degrees, double speed, long delayMillis) throws InterruptedException {
        int gyroError;
        desiredHeading = desiredHeading + degrees;
        gyroError = desiredHeading - gyro.getIntegratedZValue();
        Log.i("gyroVal", String.valueOf(gyro.getIntegratedZValue()));
        Log.i("gyroDesiredHeading", String.valueOf(desiredHeading));
        Log.e("gyroError", String.valueOf(gyroError));
        int leftTarget = leftDrive.getCurrentPosition() - (int)((countsPerDonut / 360.0) * gyroError);
        int rightTarget = rightDrive.getCurrentPosition() + (int)((countsPerDonut / 360.0) * gyroError);
        leftDrive.setTargetPosition(leftTarget);
        rightDrive.setTargetPosition(rightTarget);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        while (opModeIsActive() && (Math.abs(leftTarget -leftDrive.getCurrentPosition()) < countEndDelta) &&
                (Math.abs(rightTarget - rightDrive.getCurrentPosition()) < countEndDelta)){
            idle();
        }
        sleep(delayMillis);
        Log.i("gyroValAfter", String.valueOf(gyro.getIntegratedZValue()));
    }

    public void beacon(double speed, long delay) {
        boolean stop = false;
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        while (!stop && opModeIsActive()) {
            int redVal = bColor.red();
            int blueVal = bColor.blue();
            if (red) {
                if (redVal > blueVal) {
                    Log.i("Stopped at red", "Yay");
                    leftDrive.setPower(0.0);
                    rightDrive.setPower(0.0);
                    stop = true;
                }
            } else {
                if (blueVal > redVal) {
                    Log.i("Stopped at blue", "Yay");
                    leftDrive.setPower(0.0);
                    rightDrive.setPower(0.0);
                    stop = true;
                }
            }
            telemetry.addData("BlueVal", blueVal);
            telemetry.addData("RedVal", redVal);
            updateTelemetry(telemetry);
            if ((redVal > 0) || (blueVal > 0)) {
                Log.i("RedVal", String.valueOf(redVal));
                Log.i("BlueVal", String.valueOf(blueVal));
            }
            idle();
        }
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(delay);

    }

    public void driveToWhiteLine(double speed, long delay) {
        boolean stop = false;
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        while (!stop && opModeIsActive()) {
            if (red) {
                if (redODS.readRawVoltage() > (redODSGrayVal + (redODSGrayVal / 4))) {
                    stop = true;
                }
            } else {
                if (blueODS.readRawVoltage() > (redODSGrayVal + (redODSGrayVal / 4))) {
                    stop = true;
                }
            }
            Log.i("sideRange", String.valueOf(sideRange.getDistance(DistanceUnit.INCH)));
            idle();
        }

        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(delay);
        Log.i("redODS", String.valueOf(redODS.readRawVoltage()));
        Log.i("blueODS", String.valueOf(blueODS.readRawVoltage()));
    }

    public void shoot(int times) {
        for (int c = 0; c < times; c++) {
            kick();
            index();
            sleep(500);
        }
    }

    public void kick() {
        kicker.setPower(-1.0);
        while (opModeIsActive() && !kickStop.isPressed()) idle();
        while (opModeIsActive() && kickStop.isPressed()) idle();
        kicker.setPower(0.0);
    }

    public void index() {
        indexer.setPosition(0.18);
        sleep(300);
    }

    public void moveButtonPusherOut(long delay) {
        beaconPusher.setPosition(1.0);
        sleep(delay);
        Log.i("Done Moving", "OUT");
        beaconPusher.setPosition(0.5);
    }

    public void moveButtonPusherIn(long delay) {
        beaconPusher.setPosition(0.0);
        sleep(3000);
        Log.i("Done Moving", "IN");
        beaconPusher.setPosition(0.5);
    }
}
