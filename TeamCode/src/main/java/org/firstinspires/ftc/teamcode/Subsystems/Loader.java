package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

//single motor that loads balls
public class Loader {

    private DcMotor loaderMotor;

    public void init (HardwareMap hardwareMap){
        loaderMotor = hardwareMap.get(DcMotor.class,"loaderMotor");
        loaderMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void setLoaderMotor (double power) {
        loaderMotor.setPower(power);
    }
    public void stop(){loaderMotor.setPower(0);}

    public void setLoaderMotor() {
    }
}
