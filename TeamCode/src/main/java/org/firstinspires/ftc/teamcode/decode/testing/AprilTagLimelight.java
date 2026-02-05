package org.firstinspires.ftc.teamcode.decode.testing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Limelight;

public class AprilTagLimelight extends OpMode {

    //private Limelight3A limelight;
    boolean isRed = true;
    Limelight limelight = new Limelight(hardwareMap);
    Drivetrain drivetrain = new Drivetrain();

    private Follower follower;
    private boolean following = false;
    @Override
    public void init() {
        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
       // limelight.pipelineSwitch(0);// april tag piline
        drivetrain.init(hardwareMap);
    }

    @Override
    public void start(){
    limelight.start();
    limelight.getDistanceTarget(isRed, telemetry);

       double heading = limelight.getRotation();
       Pose dpose = drivetrain.getPose();

    telemetry.addData( " pose estimate", drivetrain.getPose() );
        telemetry.addData( " heading", heading);


    }

    @Override
    public void loop() {

    }
}
