package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team13180.JewelKnockoutArm;

/**
 * Created by manish on 11/2/2017.
 */

@TeleOp
public class MyFIRSTJavaOpModeShivam extends LinearOpMode {
    //private Gyroscope imu;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor elevator;
    private DcMotor motorTest4;
    private DigitalChannel digitalTouch;
    private DistanceSensor sensorColorRange;
    private Servo armServo;
    private Servo servoTest2;
    private JewelKnockoutArm jewelKnockoutArm;


    @Override
    public void runOpMode() {
        //imu = hardwareMap.get(Gyroscope.class, "imu");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        //motorTest4 = hardwareMap.get(DcMotor.class, "motorTest4");
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        //sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        armServo = hardwareMap.get(Servo.class, "armServo");
        //servoTest2 = hardwareMap.get(Servo.class, "servoTest2");

        // TODO: If we comment this line, then we need to change
        // leftMote set[power to + instead of -
        // Do after Saturday game
        // One more direction is set to REVERSE
        // leftMotor.setDirection(DcMotor.Direction.REVERSE);

        jewelKnockoutArm = new JewelKnockoutArm();
        jewelKnockoutArm.init(hardwareMap);

        jewelKnockoutArm.getJewelServo().getController().pwmEnable();
        try {
            Thread.sleep(1000);
            jewelKnockoutArm.getJewelServo().setDirection(Servo.Direction.FORWARD);
            Thread.sleep(1000);
            // Move the Arm up (90 Degree)
            jewelKnockoutArm.setJewelArmPosition(0.5);
            Thread.sleep(1000);
        } catch (Exception e) {
            telemetry.addData("Status", "Exception");
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // set the digital channel to input mode
        //digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        /*while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
        } */

        // run until the end of the match (driver presses STOP)
        /* double tgtPower = 0;
        while (opModeIsActive()) {
            tgtPower = -this.gamepad1.left_stick_y;
            motorTest1.setPower(tgtPower);
            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Motor Power", motorTest.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();

        } */

        // run until the end of the match (driver presses STOP)
        double tgtPower1 = 0;
        double tgtPower2 = 0;
        double multiPlier1 = 0.50;
        double multiPlier2 = 0.25;
        while (opModeIsActive()) {
            // Gamepad 1
            // MotorTest1
            tgtPower1 =  - multiPlier1 * this.gamepad1.left_stick_y;
            leftMotor.setPower(tgtPower1);
            // MotorTest2
            tgtPower1 =  multiPlier1 * this.gamepad1.right_stick_y;
            //tgtPower1 = - multiPlier1 * this.gamepad1.right_stick_y;
            rightMotor.setPower(tgtPower1);
            // Gamepad 2
            // Gamepad 2 is used to control Motor 3 and 4
            tgtPower2 = - multiPlier2 * this.gamepad2.left_stick_y;
            elevator.setPower(tgtPower2);
            //tgtPower2 = - multiPlier2 * this.gamepad2.right_stick_y;
            //motorTest4.setPower(tgtPower2);
            // check to see if we need to move the servo.
            // Gamepad 2 is used to control Servo Motor 1 and 2
            if(gamepad2.y) {
                // move to 0 degrees.
                armServo.setPosition(0);
                //servoTest2.setPosition(1);
            } else if (gamepad2.x || gamepad2.b) {
                // move to 90 degrees.
                //servoTest1.setPosition(0.25);
                //servoTest1.setPosition(0.5);
                //servoTest2.setPosition(0.5);
            } else if (gamepad2.a) {
                // move to 180 degrees.
                armServo.setPosition(0.75);
                //servoTest1.setPosition(1);
                //servoTest2.setPosition(0);
            }

            telemetry.addData("Servo1 Position", armServo.getPosition());
            //telemetry.addData("Servo2 Position", servoTest2.getPosition());
            telemetry.addData("Target Power1", tgtPower1);
            telemetry.addData("Target Power2", tgtPower2);
            telemetry.addData("Motor1 Power", leftMotor.getPower());
            telemetry.addData("Motor2 Power", rightMotor.getPower());
            telemetry.addData("elevator Power", elevator.getPower());
            //telemetry.addData("Motor4 Power", motorTest4.getPower());
            // Following line is for Touch Sensor
            //telemetry.addData("Distance (cm)", sensorColorRange.getDistance(DistanceUnit.CM));
            telemetry.addData("Status Dec 02",  "Running");
            telemetry.update();

        }
    }
}
