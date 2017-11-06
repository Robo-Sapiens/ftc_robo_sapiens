package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by manish on 11/2/2017.
 */

@TeleOp
// @Autonomous
public class MyFIRSTJavaOpModeShivam extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor motorTest1;
    private DcMotor motorTest2;
    private DcMotor motorTest3;
    private DcMotor motorTest4;
    private DigitalChannel digitalTouch;
    private DistanceSensor sensorColorRange;
    private Servo servoTest1;
    private Servo servoTest2;


    @Override
    public void runOpMode() {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        motorTest1 = hardwareMap.get(DcMotor.class, "motorTest1");
        motorTest2 = hardwareMap.get(DcMotor.class, "motorTest2");
        motorTest3 = hardwareMap.get(DcMotor.class, "motorTest3");
        motorTest4 = hardwareMap.get(DcMotor.class, "motorTest4");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        servoTest1 = hardwareMap.get(Servo.class, "servoTest1");
        servoTest2 = hardwareMap.get(Servo.class, "servoTest2");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // set the digital channel to input mode
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
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
        double tgtPower2 = 0;;
        double multiPlier = 0.25;
        while (opModeIsActive()) {
            // Gamepad 1
            tgtPower1 =  - multiPlier * this.gamepad1.left_stick_y;
            motorTest1.setPower(tgtPower1);
            tgtPower1 = - multiPlier * this.gamepad1.right_stick_y;
            motorTest2.setPower(tgtPower1);
            // GamePad 2
            // Gamepad 2 is used to control Motor 3 and 4
            tgtPower2 = - multiPlier * this.gamepad2.left_stick_y;
            motorTest3.setPower(tgtPower2);
            tgtPower2 = - multiPlier * this.gamepad2.right_stick_y;
            motorTest4.setPower(tgtPower2);
            // check to see if we need to move the servo.
            // Gamepad 2 is used to control Servo Motor 1 and 2
            if(gamepad2.y) {
                // move to 0 degrees.
                servoTest1.setPosition(0);
                servoTest2.setPosition(1);
            } else if (gamepad2.x || gamepad2.b) {
                // move to 90 degrees.
                servoTest1.setPosition(0.5);
                servoTest2.setPosition(0.5);
            } else if (gamepad2.a) {
               // move to 180 degrees.
                servoTest1.setPosition(1);
                servoTest2.setPosition(0);
            }

            telemetry.addData("Servo1 Position", servoTest1.getPosition());
            telemetry.addData("Servo2 Position", servoTest2.getPosition());
            telemetry.addData("Target Power1", tgtPower1);
            telemetry.addData("Target Power2", tgtPower2);
            telemetry.addData("Motor1 Power", motorTest1.getPower());
            telemetry.addData("Motor2 Power", motorTest2.getPower());
            telemetry.addData("Motor3 Power", motorTest3.getPower());
            telemetry.addData("Motor4 Power", motorTest4.getPower());
            // Following line is for Touch Sensor
            telemetry.addData("Distance (cm)", sensorColorRange.getDistance(DistanceUnit.CM));
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}
