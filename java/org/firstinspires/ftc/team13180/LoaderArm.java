package org.firstinspires.ftc.team13180;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Shivam Adeshara on 12/24/2017.
 */

public class LoaderArm {
    private DcMotor elevator;
    public Servo armServo;

    public void init(HardwareMap hardwareMap) {
        armServo = hardwareMap.get(Servo.class, "armServo");
        //armServo.scaleRange(0.0, 1.0);
        armServo.getController().pwmEnable();
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        elevator.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO : Check this
        closeArm();
    }

    public void moveUpArm (double power){
        elevator.setPower(power);
    }

    public void moveDownArm (double power){
        elevator.setPower (-power);
    }

    public void moveUpArmTime(double power, long time) throws InterruptedException {
        moveUpArm(power);
        Thread.sleep(time);
        stopArm();
    }

    public void moveDownArmTime(double power, long time) throws InterruptedException {
        moveDownArm(power);
        Thread.sleep(time);
        stopArm();
    }


    public void openArm () {
        armServo.setPosition(0.5);
    }

    public void closeArm (){
        armServo.setPosition(0.2);

    }

    public void stopArm(){
        elevator.setPower(0);
    }


}
