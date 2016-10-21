package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Robotics on 10/13/2016.
 */
@Autonomous(name = "FixAutonomous", group = "4507")
//@Disabled
public class AutoToMakeThingsActuallyWorkAndNOTLessBadByAlecAndGary extends LinearOpMode {

    DcMotor lD;
    DcMotor rD;

    @Override
    public void runOpMode() throws InterruptedException {
        lD = hardwareMap.dcMotor.get("l1");
        rD = hardwareMap.dcMotor.get("r1");

    }
}
