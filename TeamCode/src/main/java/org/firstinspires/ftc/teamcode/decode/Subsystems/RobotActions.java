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
    public static double SHOOTER_VELOCITY_CLOSE = 1900; //1350
    public static double SHOOTER_VELOCITY_MID   = 2400;  //2200
    public static double SHOOTER_VELOCITY_FAR   = 2600; //2750
    public static double targetVelo = 1500;
//    public static double HOOD_MIN_POSITION      = 0.99; //.02
//    public static double HOOD_MID_POSITION      = 0.89;  //63
//    public static double HOOD_MAX_POSITION      = 0.71;  //73





    public static double MIN_DISTANCEFAR          = 20.0;
    public static double MID_DISTANCEFAR           = 60.0;
    public static double MAX_DISTANCEFAR           = 180.0;
    public static double SHOOTER_VELOCITY_CLOSEFAR = 2150;    //1700
    public static double SHOOTER_VELOCITY_MIDFAR   = 2200;    //2200
    public static double SHOOTER_VELOCITY_FARFAR  = 2300;    //2750
//    public static double HOOD_MIN_POSITIONFAR      = 0.99; //.02
//    public static double HOOD_MID_POSITIONFAR      = 0.96;  //63
//    public static double HOOD_MAX_POSITIONFAR      = 0.75;  //73

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
//    public static double distanceToHoodPosition(double distance) {
//        if (distance <= MID_DISTANCE) {
//            double t = (distance - MIN_DISTANCE) / (MID_DISTANCE - MIN_DISTANCE);
//            t = Math.max(0.0, Math.min(1.0, t));
//            return HOOD_MAX_POSITION - t * (HOOD_MAX_POSITION - HOOD_MID_POSITION);
//        } else {
//            double t = (distance - MID_DISTANCE) / (MAX_DISTANCE - MID_DISTANCE);
//            t = Math.max(0.0, Math.min(1.0, t));
//            return HOOD_MID_POSITION - t * (HOOD_MID_POSITION - HOOD_MIN_POSITION);
//        }
//    }
    public static double distanceToShooterVelocityFar(double distance) {
        if (distance <= MID_DISTANCEFAR) {
            double t = (distance - MIN_DISTANCEFAR) / (MID_DISTANCEFAR - MIN_DISTANCEFAR);
            t = Math.max(0.0, Math.min(1.0, t));
            return SHOOTER_VELOCITY_CLOSEFAR + t * (SHOOTER_VELOCITY_MIDFAR - SHOOTER_VELOCITY_CLOSEFAR);
        } else {
            double t = (distance - MID_DISTANCEFAR) / (MAX_DISTANCEFAR - MID_DISTANCEFAR);
            t = Math.max(0.0, Math.min(1.0, t));
            return SHOOTER_VELOCITY_MIDFAR + t * (SHOOTER_VELOCITY_FARFAR - SHOOTER_VELOCITY_MIDFAR);
        }
    }

//    // ── Hood mapping (identical to TeleOp) ───────────────────────────────
//    public static double distanceToHoodPositionFar(double distance) {
//        if (distance <= MID_DISTANCEFAR) {
//            double t = (distance - MIN_DISTANCEFAR) / (MID_DISTANCEFAR - MIN_DISTANCEFAR);
//            t = Math.max(0.0, Math.min(1.0, t));
//            return HOOD_MAX_POSITIONFAR - t * (HOOD_MAX_POSITIONFAR - HOOD_MID_POSITIONFAR);
//        } else {
//            double t = (distance - MID_DISTANCEFAR) / (MAX_DISTANCEFAR - MID_DISTANCEFAR);
//            t = Math.max(0.0, Math.min(1.0, t));
//            return HOOD_MID_POSITIONFAR - t * (HOOD_MID_POSITIONFAR - HOOD_MIN_POSITIONFAR);
//        }
//    }
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
        if(true) {
            return new SequentialAction(
                    new InstantAction(() -> robot.loader.setLoaderMotor(power)),
                    new SleepAction(timeSeconds),
                    new InstantAction(() -> robot.loader.stop())
            );
        }
        return new SequentialAction();
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
                    targetVelo = distanceToShooterVelocity(dist);
                }),

               new SleepAction(timeSeconds)
        );
    }
    public static Action startShooterfar(double timeSeconds) {
        return new SequentialAction(
                new InstantAction(() -> {
                    double dist = getDistanceToGoal();
                    // Same functions as TeleOp - velocity AND hood set at same time
                    robot.shooter.setVelocity(distanceToShooterVelocityFar(dist));
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