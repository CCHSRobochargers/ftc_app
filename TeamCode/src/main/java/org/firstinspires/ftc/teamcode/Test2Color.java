package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by CAD1 on 12/27/2016.
 */

public class Test2Color extends LinearOpMode {

    DcMotor leftDrive;
    DcMotor rightDrive;
    ColorSensor colorLeft;
    ColorSensor colorRight;
    int countsPerYard = 2867;

    @Override
    public void runOpMode() throws InterruptedException {
        leftDrive = hardwareMap.dcMotor.get("l");
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive = hardwareMap.dcMotor.get("r");
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        colorLeft = hardwareMap.colorSensor.get("cL");
        colorRight = hardwareMap.colorSensor.get("cR");

        waitForStart();

        driveStraight(36.0, 1.0, 100);
    }





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
}

