package org.firstinspires.ftc.team13180;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Shivam Adeshara on 12/29/2017.
 * This is autonomous program with time
 */

@Autonomous(name="AutonomousRedTeam", group="autonomusGroup1")
public class AutonomousRedTeam extends LinearOpMode {
    private RobotNavigator robotNavigator;
    private LoaderArm loaderArm;
    private JewelColorSensor jewelColorSensor;
    private JewelKnockoutArm jewelKnockoutArm;

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

        jewelColorSensor = new JewelColorSensor();
        jewelColorSensor.init(hardwareMap);

        jewelKnockoutArm = new JewelKnockoutArm();
        jewelKnockoutArm.init(hardwareMap);

        telemetry.addData("Status:", "initialized");
        telemetry.update();

        // Wait for 2 seconds
        try {
            Thread.sleep(2000);
        } catch(Exception e) {

        }

        waitForStart();

        //
        try {
            jewelKnockoutArm.jewelServo.getController().pwmEnable();
            Thread.sleep(2000);
            //Servo.Direction direction = jewelKnockoutArm.jewelServo.getDirection();
            //jewelKnockoutArm.jewelServo.setDirection(Servo.Direction.REVERSE);
            jewelKnockoutArm.jewelServo.setDirection(Servo.Direction.FORWARD);
            Thread.sleep(2000);
            telemetry.addData("Position:", jewelKnockoutArm.getJewelArmPosition());
            //    telemetry.update();
            // Move the Arm Down (180 Degree)
            jewelKnockoutArm.setJewelArmPosition(1.0);
            Thread.sleep(2000);

            if(jewelColorSensor.isColorBlue()) {
                // Move robot Forward
                robotNavigator.moveForwardTime(0.50, 250);
                // Move robot backward
                //robotNavigator.moveBackwardTime(0.25, 250);
            } else if(jewelColorSensor.isColorRed()) {
                // Move robot backward
                robotNavigator.moveBackwardTime(0.50, 100);
                // Move robot Forward
                // robotNavigator.moveForwardTime(0.25, 100);
            }

            telemetry.addData("Position:", jewelKnockoutArm.getJewelArmPosition());
            //    telemetry.update();

            Thread.sleep(2000);
            // mOVE THE ARM uP (90 DEGReE)
            jewelKnockoutArm.setJewelArmPosition(0.5);
            Thread.sleep(2000);

            // Move the Arm up
            //jewelKnockoutArm.setJewelArmPosition(0.0);

            telemetry.addData("Position:", jewelKnockoutArm.getJewelArmPosition());
            telemetry.update();

            //jewelKnockoutArm.jewelServo.setDirection(direction);

        } catch (Exception e) {
            telemetry.addData("Exception:", e);
            telemetry.update();
        }

        // Wait for 2 seconds
        try {
            Thread.sleep(2000);
        } catch(Exception e) {

        }

        /*
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
        */
    }
}
