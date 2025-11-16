package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

// The intake is a single motor
public class Intake {
    private DcMotor intakeMotor;


    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

    }
    public void intakeArtifacts(double power) {
        intakeMotor.setPower(power);
    }
//
  public void intake() {
        intakeMotor.setPower(1);
    }
    public void unloadArtifacts() {
        intakeMotor.setPower(-1);
    }
    public void stop() {
        intakeMotor.setPower(0);
    }
}
// left bumper to .25
// right bumper to defualt(0.8)
//dpad_up increase speed .25
// dpad_down decrease by .25
//+ 8
