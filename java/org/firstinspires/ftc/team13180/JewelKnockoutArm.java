package org.firstinspires.ftc.team13180;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Shivam Adeshara on 12/24/2017.
 */

public class JewelKnockoutArm {
    public Servo jewelServo;

    public void init(HardwareMap hardwareMap) {
        jewelServo = hardwareMap.get(Servo.class, "jewelServo");
        //jewelServo.scaleRange(0.0, 1.0);
        jewelServo.getController().pwmEnable();
        //jewelServo.setDirection(Servo.Direction.REVERSE);
    }

    public void upJewelArm () {
        jewelServo.setPosition(0.5);
    }

    public void downJewelArm (){
        jewelServo.setPosition(0.2);
    }

    public void stopJewelArm() {
        jewelServo.setPosition(0.0);
    }

    public void setJewelArmPosition(double position) {
        jewelServo.setPosition(position);
    }

    public double getJewelArmPosition() {
        return jewelServo.getPosition();
    }




}
