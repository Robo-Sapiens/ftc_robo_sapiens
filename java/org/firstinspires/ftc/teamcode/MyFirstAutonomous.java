package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Robo... on 12/3/2017.
 */

@Autonomous(name="My Autonomous")
public class MyFirstAutonomous extends LinearOpMode {

    private Gyroscope imu;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor motorTest3;
    private DcMotor motorTest4;
    private DigitalChannel digitalTouch;
    private DistanceSensor sensorColorRange;
    private Servo servoTest1;
    private Servo servoTest2;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        //motorTest3 = hardwareMap.get(DcMotor.class, "motorTest3");
        //motorTest4 = hardwareMap.get(DcMotor.class, "motorTest4");
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        //sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        //servoTest1 = hardwareMap.get(Servo.class, "servoTest1");
        //servoTest2 = hardwareMap.get(Servo.class, "servoTest2");

        // One more direction is set to REVERSE
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // set the digital channel to input mode
        //digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        try {
            driveForwardTime(0.5, 2500);
            turnLeftTime(0.5, 1000);
            //turnRightTime(0.5, 4000);
            stopDriving();
        } catch(Exception e) {

        }

    }

    public void driveForwardTime(double power, long time) throws InterruptedException {
        driveForward(power);
        Thread.sleep(time);
    }

    public void driveForward(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        //motorLeft.setPower(power);
        //motorRight.setPower(power);
    }

    public void driveBackwardTime(double power, long time) throws InterruptedException {
        driveForward(-power);
        Thread.sleep(time);
    }

    public void driveBackward(double power) {
        driveForward(-power);
    }

    public void turnLeft(double power) {
        leftMotor.setPower(-power);
        rightMotor.setPower(power);
        //motorLeft.setPower(-power);
        //motorRight.setPower(power);
    }


    public void turnLeftTime(double power, long time) throws InterruptedException {
        turnLeft(power);
        Thread.sleep(time);
    }

    public void turnRight(double power) {
        turnLeft(-power);
    }

    public void turnRightTime(double power, long time) throws InterruptedException {
        turnRight(power);
        Thread.sleep(time);
    }

    public void stopDriving() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        //motorLeft.setPower(0);
        //motorRight.setPower(0);
    }

}
