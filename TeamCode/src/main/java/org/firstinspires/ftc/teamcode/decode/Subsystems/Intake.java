package org.firstinspires.ftc.teamcode.decode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private DcMotor intakeMotor;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    // FIXED: Now actually uses the power you pass in
    public void intakeArtifacts(double power) {
        intakeMotor.setPower(power);
    }

    public void unloadArtifacts() {
        intakeMotor.setPower(-1);
    }

    public void stop() {
        intakeMotor.setPower(0);
    }
}
