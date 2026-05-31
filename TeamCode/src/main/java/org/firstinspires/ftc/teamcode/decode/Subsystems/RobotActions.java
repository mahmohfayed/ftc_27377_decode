package org.firstinspires.ftc.teamcode.decode.Subsystems;

import static org.firstinspires.ftc.teamcode.decode.Subsystems.Common.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.geometry.Pose;

public class RobotActions {

    // ── Goal position for distance calculations ────────────────────────────
    private static Pose goal = new Pose(136, 136);

    public static void setGoal(Pose goalPose) {
        goal = goalPose;
    }

    // ── Distance / velocity / hood constants ──────────────────────────────
    // Keep in sync with MainTeleOp
    public static double MIN_DISTANCE           = 20.0;
    public static double MID_DISTANCE           = 60.0;
    public static double MAX_DISTANCE           = 180.0;
    public static double SHOOTER_VELOCITY_CLOSE = 1700;
    public static double SHOOTER_VELOCITY_MID   = 2200;
    public static double SHOOTER_VELOCITY_FAR   = 2750;
    public static double HOOD_MIN_POSITION      = 0.82;
    public static double HOOD_MID_POSITION      = 0.63;
    public static double HOOD_MAX_POSITION      = 0.73;

    // ── Helper: current distance to goal ──────────────────────────────────
    public static double getDistanceToGoal() {
        double robotX = robot.drivetrain.getPose().getX();
        double robotY = robot.drivetrain.getPose().getY();
        double dx = goal.getX() - robotX;
        double dy = goal.getY() - robotY;
        return Math.sqrt(dx * dx + dy * dy);
    }

    // ── Velocity mapping (identical to TeleOp) ────────────────────────────
    public static double distanceToShooterVelocity(double distance) {
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

    // ── Hood mapping (identical to TeleOp) ───────────────────────────────
    public static double distanceToHoodPosition(double distance) {
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

    // ── INTAKE ────────────────────────────────────────────────────────────
    public static Action intakeAction(double power, double timeSeconds) {
        return new SequentialAction(
                new InstantAction(() -> robot.intake.intakeArtifacts(power)),
                new SleepAction(timeSeconds),
                new InstantAction(() -> robot.intake.stop())
        );
    }

    // ── LOADER ────────────────────────────────────────────────────────────
    public static Action loaderAction(double power, double timeSeconds) {
        return new SequentialAction(
                new InstantAction(() -> robot.loader.setLoaderMotor(power)),
                new SleepAction(timeSeconds),
                new InstantAction(() -> robot.loader.stop())
        );
    }

    // ── SHOOTER ───────────────────────────────────────────────────────────

    /**
     * Spin up shooter + set hood using same distance-based logic as TeleOp.
     * Called once via callback at X% into the shoot path.
     * timeSeconds = how long to hold before the action finishes
     * (set this to the remaining path travel time so it stays running until arrival)
     */
    public static Action startShooter(double timeSeconds) {
        return new SequentialAction(
                new InstantAction(() -> {
                    double dist = getDistanceToGoal();
                    // Same functions as TeleOp - velocity AND hood set at same time
                    robot.shooter.setVelocity(distanceToShooterVelocity(dist));
                    robot.hoodServo.setHoodServo(distanceToHoodPosition(dist));
                }),
                new SleepAction(timeSeconds)
        );
    }

    // ── HOOD SERVO ────────────────────────────────────────────────────────
    public static Action setHoodServo(double position) {
        return new InstantAction(() -> robot.hoodServo.setHoodServo(position));
    }

    // ── STOP EVERYTHING ───────────────────────────────────────────────────
    public static Action stopAll() {
        return new InstantAction(() -> {
            robot.intake.stop();
            robot.loader.stop();
            robot.shooter.stop();
        });
    }
}