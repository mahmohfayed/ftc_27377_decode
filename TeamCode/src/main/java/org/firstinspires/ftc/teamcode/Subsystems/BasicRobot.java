package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.HardwareMap;

//
public class BasicRobot {

    double x, y, rx;
    public Drivetrain drivetrain;

    public  Intake intake;
    public  Loader loader;
    public  Shooter shooter;
    public  C70 camera;

    // Wrist wrist = new Wrist();

    //public final BulkReader bulkReader;

    public BasicRobot() {
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
    public void score(){


    }
    public void intakeArtifacts(){
        intake.intake();
    }
    public void shootArtifacts(){
        shooter.shootArtifacts();
    }
    public void loadArtifacts(){
        loader.loadArtifacts();
    }
    public void unloadArtifacts(){
        intake.unloadArtifacts();
    }
    public void stop(){
        drivetrain.stop();
        intake.stop();
        shooter.stop();
        loader.stop();
        camera.stop();
    }

    public void drive(){
        double x = gamepad1.right_stick_x;
        double y = -gamepad1.left_stick_y;
        double rx = -gamepad1.left_stick_y;    // forward/backward
        drivetrain.driveFieldRelative(x, y, rx);

    }

}
