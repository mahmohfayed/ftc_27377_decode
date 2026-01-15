package org.firstinspires.ftc.teamcode.decode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HoodServo {
    private Servo hoodservo;

    public void init(HardwareMap hardwareMap){
        hoodservo = hardwareMap.get(Servo.class, "hoodServo");
    }
    public void setHoodservo(double angle){
        hoodservo.setPosition(angle);
    }
}
