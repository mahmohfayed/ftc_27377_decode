package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;

//
public class Robot {
    public final Drivetrain drivetrain;

    public final Intake intake;
    public final Loader loader;
    public final Shooter shooter;
    public final C70 camera;

    // Wrist wrist = new Wrist();

    //public final BulkReader bulkReader;

    public Robot() {
        //drivetrain = new MecanumDrive(hardwareMap, startPose);
        loader = new Loader();
        intake = new Intake();
        shooter = new Shooter();
        camera = new C70();
        drivetrain = new Drivetrain();

    }

    public void init(HardwareMap hardwareMap) {
        drivetrain.init(hardwareMap);
        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        loader.init(hardwareMap);
        camera.init(hardwareMap);

    }

}
