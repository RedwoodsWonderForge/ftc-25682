package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import java.util.HashMap;


public class RGBIndicator {

    private HashMap<String, Double> colors;

    private String currentColor;

    private Servo rgbIndicator;

    public RGBIndicator(Servo rgbIndicator){
        this.rgbIndicator = rgbIndicator;
        this.colors = new HashMap<>();
        this.colors.put("off", 0.0);
        this.colors.put("red", 0.28);
        this.colors.put("orange",0.333);
        this.colors.put("yellow",0.388);
        this.colors.put("green",0.500);
        this.colors.put("blue",0.611);
        this.colors.put("violet", 0.722);
        this.colors.put("white", 1.0);
        this.currentColor = "off";
    }

    public String getColor () {
        return this.currentColor;
    }

    public void setColor(String color) {
        Double colorPWM = this.colors.get(color);
        colorPWM = colorPWM == null ? 0.0 : colorPWM;
        rgbIndicator.setPosition(colorPWM);
        this.currentColor = color;
    }
}