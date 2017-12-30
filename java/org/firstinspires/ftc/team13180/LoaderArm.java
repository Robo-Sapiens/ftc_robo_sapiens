package org.firstinspires.ftc.team13180;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Shivam Adeshara on 12/24/2017.
 */

public class LoaderArm {
    private DcMotor elevator;
    private Servo armServo;

    public void init(HardwareMap hardwareMap) {
        armServo = hardwareMap.get(Servo.class, "armServo");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
    }
    public void moveUpArm (double power){
        elevator.setPower(power);
    }

    public void moveDownArm (double power){
        elevator.setPower (-power);
    }

    public void openArm (double position) {
        armServo.setPosition(position);
    }

    public void closeArm (double position){
        armServo.setPosition(-position);

    }

    public void stopMotor(){
        elevator.setPower(0);
    }


}
