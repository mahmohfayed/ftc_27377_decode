package org.firstinspires.ftc.teamcode.decode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

//single motor that loads balls
public class Loader {

    private DcMotor loaderMotor;

    public Loader(HardwareMap hardwareMap){
        loaderMotor = hardwareMap.get(DcMotor.class,"loaderMotor");
        loaderMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void setLoaderMotor (double power) {
        loaderMotor.setPower(power);
    }
    public void stop(){loaderMotor.setPower(0);}
    public void loadArtifacts(){
        loaderMotor.setPower(-0.7);
    }
    public void unloadArtifacts(){
        loaderMotor.setPower(-0.5);
    }

}
