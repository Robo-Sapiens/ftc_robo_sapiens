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
    private LoaderArm loaderArm;
    @Override
    public void runOpMode () {
        double forwardPower = 0.5;
        int forwardTime = 1000;
        double leftPower = 0.5;
        int leftTime = 1000;

        double armPower = 0.25;
        int armTime = 1000;

        robotNavigator = new RobotNavigator();
        robotNavigator.init(hardwareMap);

        loaderArm = new LoaderArm();
        loaderArm.init(hardwareMap);

        telemetry.addData("Status:", "initialized");
        telemetry.update();

        // Wait for 2 seconds
        try {
            Thread.sleep(2000);
        } catch(Exception e) {

        }
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

            //telemetry.addData("Status:", "Stop");
            //telemetry.update();

            // TODO:
            // Put the block with Arm
           loaderArm.moveUpArmTime(armPower, armTime);

           loaderArm.moveDownArmTime(armPower, armTime);

           // Open Arm
           // telemetry.addData("Servo PWM Status:", loaderArm.armServo.getController().getPwmStatus().toString());
           // telemetry.update();

            loaderArm.openArm();
            telemetry.addData("Servo Position:", loaderArm.armServo.getPosition());
            //telemetry.update();

            Thread.sleep(2000);
             loaderArm.closeArm();
             Thread.sleep(2000);
             telemetry.addData("Servo Position:", loaderArm.armServo.getPosition());
             loaderArm.openArm();
             telemetry.addData("Servo Position:", loaderArm.armServo.getPosition());
             telemetry.update();
             Thread.sleep(2000);

             // Move Robot Right
             robotNavigator.moveRightTime(leftPower, leftTime);

             // Move robot Backward
             robotNavigator.moveBackwardTime(forwardPower, forwardTime);

             telemetry.addData("Status:", "MoveForward");
             telemetry.update();


             //loaderArm.openArm(0.5); ;
             // loaderArm.closeArm();

        } catch (Exception e) {
            telemetry.addData("Exception:", e);
            telemetry.update();
        }

    }
}
