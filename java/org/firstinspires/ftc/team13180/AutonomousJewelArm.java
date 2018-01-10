package org.firstinspires.ftc.team13180;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Shivam Adeshara on 12/29/2017.
 * This is autonomous program with time
 */

@Autonomous(name="AutonomousJewelArm", group="autonomusGroup1")
public class AutonomousJewelArm extends LinearOpMode {
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
        //loaderArm.init(hardwareMap);

        jewelColorSensor = new JewelColorSensor();
        jewelColorSensor.init(hardwareMap);

        jewelKnockoutArm = new JewelKnockoutArm();
        jewelKnockoutArm.init(hardwareMap);

        telemetry.addData("Status:", "initialized");
        telemetry.update();

        //jewelKnockoutArm.setJewelArmPosition(0.0);

        // Wait for 2 seconds
        try {
            Thread.sleep(2000);
        } catch(Exception e) {

        }

        waitForStart();

        try {
            jewelKnockoutArm.jewelServo.getController().pwmEnable();
            Thread.sleep(2000);
            Servo.Direction direction = jewelKnockoutArm.jewelServo.getDirection();
            //jewelKnockoutArm.jewelServo.setDirection(Servo.Direction.REVERSE);
            jewelKnockoutArm.jewelServo.setDirection(Servo.Direction.FORWARD);
            Thread.sleep(2000);
            telemetry.addData("Position:", jewelKnockoutArm.getJewelArmPosition());
        //    telemetry.update();
            // Move the Arm Down
            jewelKnockoutArm.setJewelArmPosition(1.0);
            Thread.sleep(2000);

            telemetry.addData("redColor", jewelColorSensor.colorSensor.red());
            telemetry.addData("blueColor", jewelColorSensor.colorSensor.blue());
            telemetry.addData("greenColor", jewelColorSensor.colorSensor.green());

            if(jewelColorSensor.isColorBlue()) {
                //Move robot Forward
                robotNavigator.moveForwardTime(0.25, 250);
                // Move robot backward
                //robotNavigator.moveBackwardTime(0.25, 250);

                telemetry.addData("redColor", "false");
                telemetry.addData("blueColor", "true");

            } else if(jewelColorSensor.isColorRed()) {
                // Move robot backward
                robotNavigator.moveBackwardTime(0.25, 100);
                // Move robot Forward
                //robotNavigator.moveForwardTime(0.25, 100);
                telemetry.addData("redColor", "true");
                telemetry.addData("blueColor", "false");
            }


            telemetry.addData("redColor", jewelColorSensor.colorSensor.red());
            telemetry.addData("blueColor", jewelColorSensor.colorSensor.blue());
            telemetry.addData("greenColor", jewelColorSensor.colorSensor.green());

            telemetry.addData("Position:", jewelKnockoutArm.getJewelArmPosition());
            telemetry.update();
            Thread.sleep(2000);
        //    telemetry.update();

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
            Thread.sleep(3000);
        } catch(Exception e) {

        }
    }
}
