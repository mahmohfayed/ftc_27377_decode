package org.firstinspires.ftc.teamcode.decode.testing;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class AprilTagLimelight extends OpMode {

    private Limelight3A limelight;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);// april tag piline
    }

    @Override
    public void start(){
    limelight.start();
    }

    @Override
    public void loop() {

    }
}
