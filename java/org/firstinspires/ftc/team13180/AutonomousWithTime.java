package org.firstinspires.ftc.team13180;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Shivam Adeshara on 12/29/2017.
 * This is autonomous program with time
 */

@Autonomous(name="AutonomousTime", group="autonomusGroup1")
public class AutonomousWithTime extends LinearOpMode {
    private RobotNavigator robotNavigator;

    @Override
    public void runOpMode () {
        double forwardPower = 0.5;
        int forwardTime = 1000;
        double leftPower = 0.5;
        int leftTime = 1000;

        robotNavigator = new RobotNavigator();
        robotNavigator.init(hardwareMap);

        telemetry.addData("Status:", "initialized");
        telemetry.update();

        waitForStart();

        try {
            // Move robot Forward
            robotNavigator.moveForwardTime(forwardPower, forwardTime);

            telemetry.addData("Status:", "MoveForward");
            telemetry.update();

            // Move Robot Left
            robotNavigator.moveLeftTime(leftPower, leftTime);

            telemetry.addData("Status:", "MoveLeft");
            telemetry.update();

            // Stop Robot
            robotNavigator.stopMotor();

            telemetry.addData("Status:", "Stop");
            telemetry.update();

            // TODO:
            // Put the block with Arm
            // Open Arm

        } catch (Exception e) {
            telemetry.addData("Exception:", e);
            telemetry.update();
        }

    }
}
