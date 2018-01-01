package org.firstinspires.ftc.team13180;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Shivam Adeshara on 12/24/2017.
 */

public class RobotNavigator {
    private DcMotor leftMotor;
private DcMotor rightMotor;

    public boolean init(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        return true;
    }

    public void moveForward(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public void moveBackward(double power) {
        leftMotor.setPower(-power);
        rightMotor.setPower(-power);
    }

    public void moveLeft (double power){
        leftMotor.setPower(power);
        rightMotor.setPower(-power);
    }

    public void moveRight (double power){
        leftMotor.setPower (-power);
        rightMotor.setPower(power);
    }

    public void stopMotor() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void moveForwardTime(double power, long time) throws InterruptedException {
        moveForward(power);
        Thread.sleep(time);
        stopMotor();
    }

    public void moveBackwardTime(double power, long time) throws InterruptedException {
        moveBackward(power);
        Thread.sleep(time);
        stopMotor();
    }

    public void moveRightTime(double power, long time) throws InterruptedException {
        moveRight(power);
        Thread.sleep(time);
        stopMotor();
    }

    public void moveLeftTime(double power, long time) throws InterruptedException {
        moveLeft(power);
        Thread.sleep(time);
        stopMotor();
    }

}

