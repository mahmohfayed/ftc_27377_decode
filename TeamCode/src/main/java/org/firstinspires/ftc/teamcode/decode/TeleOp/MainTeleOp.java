package org.firstinspires.ftc.teamcode.decode.TeleOp;

import static org.firstinspires.ftc.teamcode.decode.Subsystems.Common.isRed;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.control.controllers.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Common;
import org.firstinspires.ftc.teamcode.decode.Subsystems.LimelightEx;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Robot;

@Configurable
@TeleOp(name = "AlignTeleOpClose", group = "Main")
public class MainTeleOp extends LinearOpMode {

    private Robot robot;
    private IMU imu;
    private Limelight3A limelight;

    // ── Heading PID (auto align) ───────────────────────────────────────────
    private PIDController headingPID = new PIDController();
    public static PIDGains headingGains = new PIDGains(2.3, 0.0, 0.1);

    // ── Shooter velocity PID ───────────────────────────────────────────────
    public static double SHOOTER_P = 50.0;
    public static double SHOOTER_I = 0.0;
    public static double SHOOTER_D = 0.0;
    public static double SHOOTER_F = 12.0;

    public static double SHOOTER_VELOCITY_IDLE  = 1000;
    public static double SHOOTER_VELOCITY_CLOSE = 1350;
    public static double SHOOTER_VELOCITY_MID   = 2200;
    public static double SHOOTER_VELOCITY_FAR   = 2750;

    // ── Hood tuning ────────────────────────────────────────────────────────
    public static double HOOD_MIN_POSITION = 0.82;
    public static double HOOD_MID_POSITION = 0.63;
    public static double HOOD_MAX_POSITION = 0.73;
    public static double MIN_DISTANCE      = 20.0;
    public static double MID_DISTANCE      = 60.0;
    public static double MAX_DISTANCE      = 180.0;

    private static final double HOOD_DEADBAND = 0.005;

    // ── Limelight relocalization ───────────────────────────────────────────
    public static double LIMELIGHT_MAX_JUMP          = 24.0;
    public static double AUTO_RELOCALIZE_THRESHOLD   = 3.0;
    public static long   AUTO_RELOCALIZE_INTERVAL_MS = 500;
    public static long   STALENESS_LIMIT_MS          = 100;
    public static double INCHES_PER_METER            = 39.3701;
    public static double HEADING_OFFSET              = 270;

    // ── Poses ──────────────────────────────────────────────────────────────
    private Pose goal               = new Pose(136, 136);
    private boolean isFirst         = true;

    // ── Hardware ───────────────────────────────────────────────────────────
    private DcMotorEx flyWheelMotor;
    private DcMotorEx followerWheelMotor;

    // ── Internal state ─────────────────────────────────────────────────────
    private double  lastHoodPosition       = -1;
    private boolean shooterRunning         = false;
    private long    lastAutoRelocalizeTime = 0;
    private boolean limelightAvailable     = false;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeft  = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeft   = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRight  = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor intake     = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor loader     = hardwareMap.dcMotor.get("loaderMotor");

        flyWheelMotor      = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        followerWheelMotor = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        followerWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        flyWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        followerWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        updateShooterPIDF();

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        // ── Limelight init ─────────────────────────────────────────────────
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.start();
            limelightAvailable = true;
        } catch (Exception e) {
            limelightAvailable = false;
        }

        robot = new Robot(hardwareMap);
        headingPID.setGains(headingGains);
        robot.drivetrain.update();
        robot.drivetrain.startTeleOpDrive(true);

        if (!isRed) goal               = goal.mirror();

        waitForStart();

        // ── Starting pose: use auto end pose if auto was run, otherwise
        //    wait up to 2 seconds for limelight to get a fix, otherwise
        //    fall back to a known default position ────────────────────────
        boolean autoWasRun = Common.AUTO_END_POSE.getX() != 0
                || Common.AUTO_END_POSE.getY() != 0;

        if (autoWasRun) {
            // Auto ran — trust where it ended
            robot.drivetrain.setPose(Common.AUTO_END_POSE);
        } else if (limelightAvailable) {
            // No auto — try to get a limelight fix on startup
            Pose startPose = null;
            long startTime = System.currentTimeMillis();
            while (startPose == null && System.currentTimeMillis() - startTime < 2000 && opModeIsActive()) {
                double headingDeg = Math.toDegrees(robot.drivetrain.getHeading()) - HEADING_OFFSET;
                limelight.updateRobotOrientation(headingDeg);
                LLResult initResult = limelight.getLatestResult();
                if (initResult != null && initResult.isValid() && initResult.getStaleness() < STALENESS_LIMIT_MS) {
                    Pose3D pose3D = initResult.getBotpose_MT2();
                    if (pose3D != null) {
                        double x = (pose3D.getPosition().y * INCHES_PER_METER) + 72;
                        double y = 72 - (pose3D.getPosition().x * INCHES_PER_METER);
                        startPose = new Pose(x, y, robot.drivetrain.getHeading());
                    }
                }
                robot.drivetrain.update();
                telemetry.addLine("Waiting for Limelight fix...");
                telemetry.update();
            }
            if (startPose != null) {
                robot.drivetrain.setPose(startPose);
            }
            // else: limelight timed out - pose stays at 0,0,0 until tag seen
        }
        // else: no auto, no limelight - will correct once tag is seen

        robot.drivetrain.update();
        robot.hoodServo.setHoodServo(HOOD_MIN_POSITION);
        lastHoodPosition = HOOD_MIN_POSITION;

        while (opModeIsActive()) {

            // ── Limelight update ───────────────────────────────────────────
            // ALWAYS send heading first every loop so MegaTag2 works
            Pose limelightPose = null;
            if (limelightAvailable) {
                double headingDeg = Math.toDegrees(robot.drivetrain.getHeading()) - HEADING_OFFSET;
                limelight.updateRobotOrientation(headingDeg);

                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid() && result.getStaleness() < STALENESS_LIMIT_MS) {
                    Pose3D pose3D = result.getBotpose_MT2();
                    if (pose3D != null) {
                        double x = (pose3D.getPosition().y * INCHES_PER_METER) + 72;
                        double y = 72 - (pose3D.getPosition().x * INCHES_PER_METER);
                        limelightPose = new Pose(x, y, robot.drivetrain.getHeading());
                    }
                }
            }

            // ── Distance to goal ───────────────────────────────────────────
            double robotX   = robot.drivetrain.getPose().getX();
            double robotY   = robot.drivetrain.getPose().getY();
            double dx       = goal.getX() - robotX;
            double dy       = goal.getY() - robotY;
            double distance = Math.sqrt(dx * dx + dy * dy);

            // ── Auto hood ──────────────────────────────────────────────────
            double hoodPosition = distanceToHoodPosition(distance);
            if (Math.abs(hoodPosition - lastHoodPosition) > HOOD_DEADBAND) {
                robot.hoodServo.setHoodServo(hoodPosition);
                lastHoodPosition = hoodPosition;
            }

            // ── Shooter (Right Trigger) ────────────────────────────────────
            if (gamepad1.right_trigger > 0.1) {
                if (!shooterRunning) {
                    updateShooterPIDF();
                    shooterRunning = true;
                }
                double targetVelocity = distanceToShooterVelocity(distance);
                flyWheelMotor.setVelocity(targetVelocity);
                followerWheelMotor.setVelocity(targetVelocity);
            } else {
                if (!shooterRunning) {
                    updateShooterPIDF();
                    shooterRunning = true;
                }
                flyWheelMotor.setVelocity(SHOOTER_VELOCITY_IDLE);
                followerWheelMotor.setVelocity(SHOOTER_VELOCITY_IDLE);
            }

            // ── Feeder/Loader + Intake (Left Trigger) ──────────────────────
            double targetVelocity = distanceToShooterVelocity(distance);
            double actualVelocity = flyWheelMotor.getVelocity();
            double velocityError  = Math.abs(targetVelocity - actualVelocity);
            boolean shooterReady  = velocityError < 200;

            if (gamepad1.left_trigger > 0.1 && shooterReady) {
                loader.setPower(1);
                intake.setPower(1);
            } else if (gamepad1.left_trigger > 0.1 && !shooterReady) {
                intake.setPower(1);
                loader.setPower(0);
            } else {
                loader.setPower(0);
                intake.setPower(0);
            }

            // ── IMU reset ──────────────────────────────────────────────────
            if (gamepad1.left_stick_button) {
                imu.resetYaw();
            }

            // ── Auto align (stationary) ────────────────────────────────────
            if (gamepad1.left_bumper && isFirst) {
                headingPID.setTarget(new State(Math.atan2(dy, dx)));
                isFirst = false;
            }

            if (!isFirst) {
                robot.drivetrain.setTeleOpDrive(
                        0, 0,
                        headingPID.calculate(new State(robot.drivetrain.getHeading())),
                        true
                );
                if (headingPID.isInTolerance(
                        new State(robot.drivetrain.getHeading()), Math.toRadians(3))) {
                    isFirst = true;
                }
            } else {
                double y  = -gamepad1.left_stick_y;
                double x  = -gamepad1.left_stick_x;
                double rx = -gamepad1.right_stick_x;
                robot.drivetrain.setTeleOpDrive(y, x, rx);
            }

            // ── Loader manual controls ─────────────────────────────────────
            if (gamepad1.y && shooterReady) {
                loader.setPower(1);
            } else if (gamepad1.a) {
                loader.setPower(-1);
            } else if (gamepad1.dpad_up) {
                loader.setPower(1);
                intake.setPower(1);
            } else if (gamepad1.right_bumper) {
                intake.setPower(1);
            } else if (gamepad1.left_trigger <= 0.1) {
                loader.setPower(0);
                intake.setPower(0);
            }

            // ── Relocalize (D-pad Right) ───────────────────────────────────
            if (gamepad1.dpad_right) {
                // Only relocalize if limelight actually sees the tag
                if (limelightPose != null) {
                    double jumpDist = Math.hypot(
                            limelightPose.getX() - robotX,
                            limelightPose.getY() - robotY);
                    if (jumpDist < LIMELIGHT_MAX_JUMP) {
                        robot.drivetrain.setPose(limelightPose);
                    }
                    // else: bad reading, do nothing
                }
                // else: no tag visible, do nothing
            }

            // ── Auto-relocalize every 500ms when tag visible ───────────────
            if (limelightPose != null) {
                long now = System.currentTimeMillis();
                if (now - lastAutoRelocalizeTime > AUTO_RELOCALIZE_INTERVAL_MS) {
                    double drift = Math.hypot(
                            limelightPose.getX() - robotX,
                            limelightPose.getY() - robotY);
                    if (drift > AUTO_RELOCALIZE_THRESHOLD && drift < LIMELIGHT_MAX_JUMP) {
                        robot.drivetrain.setPose(limelightPose);
                    }
                    lastAutoRelocalizeTime = now;
                }
            }

            robot.run();

            // ── Telemetry ──────────────────────────────────────────────────
            telemetry.addLine("=== SHOOTER ===");
            telemetry.addData("Running",         shooterRunning);
            telemetry.addData("Ready to Shoot",  shooterReady ? "YES" : "NO");
            telemetry.addData("Target Velocity", "%.0f t/s", distanceToShooterVelocity(distance));
            telemetry.addData("Actual Velocity", "%.0f t/s", flyWheelMotor.getVelocity());
            telemetry.addData("Velocity Error",  "%.0f t/s",
                    distanceToShooterVelocity(distance) - flyWheelMotor.getVelocity());
            telemetry.addLine("=== HOOD ===");
            telemetry.addData("Distance to Goal", "%.1f in", distance);
            telemetry.addData("Hood Target",      "%.3f",    hoodPosition);
            telemetry.addData("Hood Actual",      "%.3f",    robot.hoodServo.getPosition());
            telemetry.addLine("=== HEADING ===");
            telemetry.addData("Heading",       "%.1f deg",
                    Math.toDegrees(robot.drivetrain.getHeading()));
            telemetry.addData("Auto Aligning", !isFirst);
            telemetry.addLine("=== LIMELIGHT ===");
            telemetry.addData("Available",   limelightAvailable);
            telemetry.addData("Tag Visible", limelightPose != null);
            if (limelightPose != null) {
                telemetry.addData("LL Pose X", "%.1f", limelightPose.getX());
                telemetry.addData("LL Pose Y", "%.1f", limelightPose.getY());
                telemetry.addData("Drift",     "%.2f in", Math.hypot(
                        limelightPose.getX() - robotX,
                        limelightPose.getY() - robotY));
            }
            telemetry.update();
        }
    }

    // ── Shooter PIDF ───────────────────────────────────────────────────────
    private void updateShooterPIDF() {
        PIDFCoefficients pidf = new PIDFCoefficients(
                SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        flyWheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        followerWheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    // ── Velocity mapping ───────────────────────────────────────────────────
    private double distanceToShooterVelocity(double distance) {
        if (distance <= MID_DISTANCE) {
            double t = (distance - MIN_DISTANCE) / (MID_DISTANCE - MIN_DISTANCE);
            t = Math.max(0.0, Math.min(1.0, t));
            return SHOOTER_VELOCITY_CLOSE + t * (SHOOTER_VELOCITY_MID - SHOOTER_VELOCITY_CLOSE);
        } else {
            double t = (distance - MID_DISTANCE) / (MAX_DISTANCE - MID_DISTANCE);
            t = Math.max(0.0, Math.min(1.0, t));
            return SHOOTER_VELOCITY_MID + t * (SHOOTER_VELOCITY_FAR - SHOOTER_VELOCITY_MID);
        }
    }

    // ── Hood mapping ───────────────────────────────────────────────────────
    private double distanceToHoodPosition(double distance) {
        if (distance <= MID_DISTANCE) {
            double t = (distance - MIN_DISTANCE) / (MID_DISTANCE - MIN_DISTANCE);
            t = Math.max(0.0, Math.min(1.0, t));
            return HOOD_MAX_POSITION - t * (HOOD_MAX_POSITION - HOOD_MID_POSITION);
        } else {
            double t = (distance - MID_DISTANCE) / (MAX_DISTANCE - MID_DISTANCE);
            t = Math.max(0.0, Math.min(1.0, t));
            return HOOD_MID_POSITION - t * (HOOD_MID_POSITION - HOOD_MIN_POSITION);
        }
    }
}
