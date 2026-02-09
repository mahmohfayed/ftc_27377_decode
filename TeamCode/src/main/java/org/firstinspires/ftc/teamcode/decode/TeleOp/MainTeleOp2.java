package org.firstinspires.ftc.teamcode.decode.TeleOp;

import static org.firstinspires.ftc.teamcode.decode.Subsystems.Common.isRed;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad; // Standard Gamepad import
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.control.controllers.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Common;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.decode.Subsystems.HoodServo;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Robot;

@Configurable
@TeleOp(name = "AlignTeleOpFar", group = "Main")
public class MainTeleOp2 extends LinearOpMode {

    private Robot robot;

    private Drivetrain drivetrain;
    private IMU imu;

    // Changed from GamepadEx to standard Gamepad
    //private GamepadEx gp1;

    private PIDController pidController = new PIDController();
    //    private HoodServo hood = new HoodServo();
    public static PIDGains pidGains = new PIDGains(
            4.3, 0.00000001 ,0.1
    );
    private Pose relocalizationfarPose = new Pose(
            5,   // x
            8,   // y
            90    // heading
    );

    private boolean isFirst = true;
    private Pose goal = new Pose(136, 136);

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRight = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor flyWheelMotor = hardwareMap.dcMotor.get("rightShooterMotor");
        DcMotor followerWheelMotor = hardwareMap.dcMotor.get("leftShooterMotor");
        DcMotor intake = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor loader = hardwareMap.dcMotor.get("loaderMotor");
        Servo servo = hardwareMap.servo.get("leftServo");

        // Assigning standard gamepad1 directly
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        followerWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is a
        // ssumed to be logo up / USB forward
        imu.initialize(parameters);
        double flyPower = 0;

        //hood.setHoodservo(0.45);

        //gp1 = new GamepadEx(gamepad1);
        servo.setPosition(0.3);

        robot = new Robot(hardwareMap);
        robot.drivetrain.setStartingPose(Common.AUTO_END_POSE);
        pidController.setGains(pidGains);
//        hood.init(hardwareMap); // Initialize your HoodServo subsystem

        robot.drivetrain.update();
        robot.drivetrain.startTeleOpDrive(true);

        if(!isRed) goal = goal.mirror();
        if (!isRed) relocalizationfarPose = relocalizationfarPose.mirror();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_stick_button) {
                imu.resetYaw();
            }
            //gp1.readButtons();

            if (gamepad1.left_bumper && isFirst)  {
                pidController.setTarget(new State(Math.atan2(goal.getY() - robot.drivetrain.getPose().getY(), goal.getX() -  robot.drivetrain.getPose().getX())));
                isFirst = false;
            }

            if (!isFirst) {
                robot.drivetrain.setTeleOpDrive(
                        0, 0, pidController.calculate(new State(robot.drivetrain.getHeading())), true
                );
                if (pidController.isInTolerance(new State(robot.drivetrain.getHeading()), Math.toRadians(3))) isFirst = true;
            }
            else {

                // Standard gamepad access uses field names (e.g., .left_stick_y)
                double y = -gamepad1.left_stick_y;   // forward = up
                double x = -gamepad1.left_stick_x;    // right = right
                double rx = -gamepad1.right_stick_x;  // rotate right = right

                robot.drivetrain.setTeleOpDrive(y, x, rx);



            }


            flyPower += gamepad1.right_trigger * 0.05;
            flyPower -= gamepad1.left_trigger * 0.05;
            flyPower = Math.max(0, Math.min(1, flyPower));
            double output = flyPower;
            flyWheelMotor.setPower(flyPower);
            followerWheelMotor.setPower(output);


            if (gamepad1.y) {
                loader.setPower(1);
                intake.setPower(0);

            } else if (gamepad1.a) {
                loader.setPower(-1);
                intake.setPower(0);

            } else if (gamepad1.dpad_up) {
                loader.setPower(1);
                intake.setPower(1);

            } else {
                loader.setPower(0);
                intake.setPower(0);
            }

            if (gamepad1.right_bumper) {
                intake.setPower(1);

            }

            if (gamepad1.x) {
                servo.setPosition(0.6);
            } else if (gamepad1.b) {
                servo.setPosition(0.5);
            } else if (gamepad1.dpad_down) {
                servo.setPosition(0.3);
            }
            if (gamepad1.right_stick_button) {
                robot.drivetrain.setPose(relocalizationfarPose);
            }

            robot.run();

            telemetry.addData("Shooter Power", flyPower);
            telemetry.update();
        }
    }
}
//decrease p if overshoots
//increase p if it undershoots
//decrease d if it jitters to much
//increase d if it doesnt push p enough (not enough precision)
//increase i if there is lots of small error over TIME
//decrease i if it is higher than d or p, or jittering
//
//P SHOULD BE BIGGEST, d is smaller, and I is smallest
//
//tune p first, then d, then i
//
//start from 0.001 for p, 0.0001 for d, 0.000001 for i
//change 1 in decimal to 5, if not move up decimal place up; same if going down

