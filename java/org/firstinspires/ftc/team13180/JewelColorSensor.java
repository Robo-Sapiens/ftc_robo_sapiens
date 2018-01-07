package org.firstinspires.ftc.team13180;

import com.qualcomm.robotcore.hardware.HardwareMap;
import android.graphics.Color;
import com.qualcomm.robotcore.hardware.ColorSensor;
/**
 * Created by Shivam Adeshara on 1/6/2018.
 */



/**
 * Created by Shivam Adeshara on 12/24/2017.
 */

public class JewelColorSensor {
    private ColorSensor colorSensor;

    boolean blueColor = false;
    boolean redColor = false;

    public void init(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    }

    public boolean isColorRed() {
        redColor = false;
        blueColor = false;

        if(colorSensor.red() > 100 && colorSensor.green() < 100 &&  colorSensor.blue() < 100) {
            redColor = true;
            blueColor = false;
            return true;
        }
        return false;
    }

    public boolean isColorBlue() {
        redColor = false;
        blueColor = false;

        if(colorSensor.red() < 100 && colorSensor.green() < 100 &&  colorSensor.blue() > 100) {
            redColor = false;
            blueColor = true;
            return true;
        }
        return false;
    }
}





