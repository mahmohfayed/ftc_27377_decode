package org.firstinspires.ftc.teamcode.testing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.decode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Limelight;

public abstract class AutoAlignRed extends LinearOpMode {
    private Limelight limelight = new Limelight(hardwareMap);
    private final boolean IS_RED = true;

    public static Pose LAST_REMEMBERED_POSE = new Pose(0, 0, 0);

    private Drivetrain drivetrain = new Drivetrain();

    public void runOpMode() throws InterruptedException{

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            if (IS_RED) {
                limelight.setRedGoalPipeline();
            } else {
                limelight.setBlueGoalPipeline();
            }


            if (IS_RED) {
                drivetrain.setStartingPose(LAST_REMEMBERED_POSE);
            } else {
                if (LAST_REMEMBERED_POSE.getHeading() == 0) {
                    LAST_REMEMBERED_POSE = LAST_REMEMBERED_POSE.setHeading(Math.toRadians(180));
                }
                drivetrain.setStartingPose(LAST_REMEMBERED_POSE);
            }
        }
    }


}
