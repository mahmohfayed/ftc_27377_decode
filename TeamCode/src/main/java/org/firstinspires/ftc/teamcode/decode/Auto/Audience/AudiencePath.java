package org.firstinspires.ftc.teamcode.decode.Auto.Audience;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class AudiencePath {

    private final Follower f;

    public PathChain shootPreload;

    public PathChain intake3;
    public PathChain shoot3;

    public PathChain intake6;
    public PathChain shoot6;

    public PathChain unloadRamp;

    public PathChain intake9;
    public PathChain shoot9;

    public PathChain intakeExtra1;
    public PathChain shootExtra1;
    public PathChain intakeExtra2;
    public PathChain shootExtra2;
    public PathChain intakeExtra3;
    public PathChain shootExtra3;

    public PathChain leave;

    // =============================
    // POSES
    // =============================

    public static Pose P_START = new Pose(56.047, 8.103);
    public static Pose P_SHOOT = new Pose(58.000, 16.000);

    // Intake 3
    public static Pose P_I3_CP = new Pose(58.748, 33.088);
    public static Pose P_I3_END = new Pose(40.853, 35.451);
    public static Pose P_I3_WALL = new Pose(10.298, 35.958);

    // Intake 6
    public static Pose P_I6_CP = new Pose(57.060, 53.683);
    public static Pose P_I6_END = new Pose(39.165, 60.098);
    public static Pose P_I6_WALL = new Pose(9.623, 59.761);

    // Ramp
    public static Pose P_RAMP_CP = new Pose(45.749, 42.542);
    public static Pose P_RAMP_END = new Pose(50.982, 68.877);
    public static Pose P_RAMP_WALL = new Pose(13.505, 69.383);
    public static Pose P_RAMP_RET = new Pose(50.982, 68.539);

    // Intake 9
    public static Pose P_I9_CP = new Pose(58.410, 79.343);
    public static Pose P_I9_END = new Pose(47.606, 83.733);
    public static Pose P_I9_WALL = new Pose(14.518, 83.733);
    public static Pose P_I9_RETURN = new Pose(47.437, 83.902);

    // Extra balls
    public static Pose P_EX_CP1 = new Pose(56.891, 60.436);
    public static Pose P_EX_END1 = new Pose(11.648, 61.787);

    public static Pose P_EX_CP2 = new Pose(56.722, 60.774);
    public static Pose P_EX_END2 = new Pose(11.817, 61.787);

    public static Pose P_EX_CP3 = new Pose(57.397, 60.436);
    public static Pose P_EX_END3 = new Pose(11.817, 61.449);

    // Leave
    public static Pose P_LEAVE_END = new Pose(57.735, 34.101);

    public static double H_90  = Math.toRadians(0);
    public static double H_180  = Math.toRadians(90);
        public static double H_114 = Math.toRadians(16); // heading for far

    public static double H_150 = Math.toRadians(60);
    // =============================
    // MIRROR SUPPORT
    // ===================
    public AudiencePath(Follower follower) {
        f = follower;
    }
    private double mirrorAngleBlue(double angle) {
        return Math.PI - angle;
    }
    public static boolean isPathRed = false;

    public void audienceMirrorAll() {
        P_START = P_START.mirror();
        P_SHOOT = P_SHOOT.mirror();

        P_I3_CP = P_I3_CP.mirror();
        P_I3_END = P_I3_END.mirror();
        P_I3_WALL = P_I3_WALL.mirror();

        P_I6_CP = P_I6_CP.mirror();
        P_I6_END = P_I6_END.mirror();
        P_I6_WALL = P_I6_WALL.mirror();

        P_RAMP_CP = P_RAMP_CP.mirror();
        P_RAMP_END = P_RAMP_END.mirror();
        P_RAMP_WALL = P_RAMP_WALL.mirror();
        P_RAMP_RET = P_RAMP_RET.mirror();

        P_I9_CP = P_I9_CP.mirror();
        P_I9_END = P_I9_END.mirror();
        P_I9_WALL = P_I9_WALL.mirror();
        P_I9_RETURN = P_I9_RETURN.mirror();

        P_EX_CP1 = P_EX_CP1.mirror();
        P_EX_END1 = P_EX_END1.mirror();

        P_EX_CP2 = P_EX_CP2.mirror();
        P_EX_END2 = P_EX_END2.mirror();

        P_EX_CP3 = P_EX_CP3.mirror();
        P_EX_END3 = P_EX_END3.mirror();

        P_LEAVE_END = P_LEAVE_END.mirror();

        H_180 = mirrorAngleBlue(H_180);
        H_114 = mirrorAngleBlue(H_114);
        H_90 = mirrorAngleBlue(H_90);
        H_150 = mirrorAngleBlue(H_150);
    }

    // =============================
    // CONSTRUCTOR
    // =============================


    public void goal3Build() {

        // ---------- Preload ----------
        shootPreload = f.pathBuilder()
                .addPath(new BezierLine(P_START, P_SHOOT))
                .setLinearHeadingInterpolation(H_90,H_114)
                .build();

        // ---------- Leave ----------
        leave = f.pathBuilder()
                .addPath(new BezierLine(P_SHOOT, P_LEAVE_END))
                .setConstantHeadingInterpolation(H_114)
                .build();
    }
    public void goal6Build() {

        // ---------- Preload ----------
        shootPreload = f.pathBuilder()
                .addPath(new BezierLine(P_START, P_SHOOT))
                .setLinearHeadingInterpolation(H_90,H_114)
                .build();

        // ---------- Intake + Shoot 3 ----------
        intake3 = f.pathBuilder()
                .addPath(new BezierCurve(P_SHOOT, P_I3_CP, P_I3_END))
                .setLinearHeadingInterpolation(H_114,H_180)
                .addPath(new BezierLine(P_I3_END, P_I3_WALL))
                .setConstantHeadingInterpolation(H_180)
                .addPath(new BezierLine(P_I3_WALL, P_I3_END))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        shoot3 = f.pathBuilder()
                .addPath(new BezierCurve(P_I3_END, P_I3_CP, P_SHOOT))
                .setLinearHeadingInterpolation(H_180,H_114)
                .build();

        // ---------- Leave ----------
        leave = f.pathBuilder()
                .addPath(new BezierLine(P_SHOOT, P_LEAVE_END))
                .setConstantHeadingInterpolation(H_114)
                .build();
    }
    public void goal9Build() {

        // ---------- Preload ----------
        shootPreload = f.pathBuilder()
                .addPath(new BezierLine(P_START, P_SHOOT))
                .setLinearHeadingInterpolation(H_90,H_114)
                .build();

        // ---------- Intake + Shoot 3 ----------
        intake3 = f.pathBuilder()
                .addPath(new BezierCurve(P_SHOOT, P_I3_CP, P_I3_END))
                .setLinearHeadingInterpolation(H_114,H_180)
                .addPath(new BezierLine(P_I3_END, P_I3_WALL))
                .setConstantHeadingInterpolation(H_180)
                .addPath(new BezierLine(P_I3_WALL, P_I3_END))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        shoot3 = f.pathBuilder()
                .addPath(new BezierCurve(P_I3_END, P_I3_CP, P_SHOOT))
                .setLinearHeadingInterpolation(H_180,H_114)
                .build();

        // ---------- Intake + Shoot 6 ----------
        intake6 = f.pathBuilder()
                .addPath(new BezierCurve(P_SHOOT, P_I6_CP, P_I6_END))
                .setLinearHeadingInterpolation(H_114,H_180)
                .addPath(new BezierLine(P_I6_END, P_I6_WALL))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(P_I6_WALL, P_I6_END))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        shoot6 = f.pathBuilder()
                .addPath(new BezierCurve(P_I6_END, P_I6_CP, P_SHOOT))
                .setLinearHeadingInterpolation(H_180, H_114)
                .build();

        // ---------- Leave ----------
        leave = f.pathBuilder()
                .addPath(new BezierLine(P_SHOOT, P_LEAVE_END))
                .setConstantHeadingInterpolation(H_114)
                .build();
    }
    public void goal12Build() {

        // ---------- Preload ----------
        shootPreload = f.pathBuilder()
                .addPath(new BezierLine(P_START, P_SHOOT))
                .setLinearHeadingInterpolation(H_90,H_114)
                .build();

        // ---------- Intake + Shoot 3 ----------
        intake3 = f.pathBuilder()
                .addPath(new BezierCurve(P_SHOOT, P_I3_CP, P_I3_END))
                .setLinearHeadingInterpolation(H_114,H_180)
                .addPath(new BezierLine(P_I3_END, P_I3_WALL))
                .setConstantHeadingInterpolation(H_180)
                .addPath(new BezierLine(P_I3_WALL, P_I3_END))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        shoot3 = f.pathBuilder()
                .addPath(new BezierCurve(P_I3_END, P_I3_CP, P_SHOOT))
                .setLinearHeadingInterpolation(H_180,H_114)
                .build();

        // ---------- Intake + Shoot 6 ----------
        intake6 = f.pathBuilder()
                .addPath(new BezierCurve(P_SHOOT, P_I6_CP, P_I6_END))
                .setLinearHeadingInterpolation(H_114,H_180)
                .addPath(new BezierLine(P_I6_END, P_I6_WALL))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(P_I6_WALL, P_I6_END))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        shoot6 = f.pathBuilder()
                .addPath(new BezierCurve(P_I6_END, P_I6_CP, P_SHOOT))
                .setLinearHeadingInterpolation(H_180, H_114)
                .build();

        // ---------- Intake + Shoot 9 ----------
        intake9 = f.pathBuilder()
                .addPath(new BezierCurve(P_RAMP_RET, P_I9_CP, P_I9_END))
                .setConstantHeadingInterpolation(H_180)
                .addPath(new BezierLine(P_I9_END, P_I9_WALL))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(P_I9_WALL, P_I9_RETURN))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        shoot9 = f.pathBuilder()
                .addPath(new BezierCurve(P_I9_RETURN, P_RAMP_CP, P_SHOOT))
                .setLinearHeadingInterpolation(H_180,H_114)
                .build();

        // ---------- Leave ----------
        leave = f.pathBuilder()
                .addPath(new BezierLine(P_SHOOT, P_LEAVE_END))
                .setConstantHeadingInterpolation(H_114)
                .build();
    }


    public void goal21Build() {

        shootPreload = f.pathBuilder()
                .addPath(new BezierLine(P_START, P_SHOOT))
                .setLinearHeadingInterpolation(H_90,H_114)
                .build();

        intake3 = f.pathBuilder()
                .addPath(new BezierCurve(P_SHOOT, P_I3_CP, P_I3_END))
                .setLinearHeadingInterpolation(H_114,H_180)
                .addPath(new BezierLine(P_I3_END, P_I3_WALL))
                .setConstantHeadingInterpolation(H_180)
                .addPath(new BezierLine(P_I3_WALL, P_I3_END))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        shoot3 = f.pathBuilder()
                .addPath(new BezierCurve(P_I3_END, P_I3_CP, P_SHOOT))
                .setLinearHeadingInterpolation(H_180,H_114)
                .build();

        intake6 = f.pathBuilder()
                .addPath(new BezierCurve(P_SHOOT, P_I6_CP, P_I6_END))
                .setLinearHeadingInterpolation(H_114,H_180)
                .addPath(new BezierLine(P_I6_END, P_I6_WALL))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(P_I6_WALL, P_I6_END))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        shoot6 = f.pathBuilder()
                .addPath(new BezierCurve(P_I6_END, P_I6_CP, P_SHOOT))
                .setLinearHeadingInterpolation(H_180, H_114)
                .build();

        unloadRamp = f.pathBuilder()
                .addPath(new BezierCurve(P_SHOOT, P_RAMP_CP, P_RAMP_END))
                .setLinearHeadingInterpolation(H_114,H_180)
                .addPath(new BezierLine(P_RAMP_END, P_RAMP_WALL))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(P_RAMP_WALL, P_RAMP_RET))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intake9 = f.pathBuilder()
                .addPath(new BezierCurve(P_RAMP_RET, P_I9_CP, P_I9_END))
                .setConstantHeadingInterpolation(H_180)
                .addPath(new BezierLine(P_I9_END, P_I9_WALL))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(P_I9_WALL, P_I9_RETURN))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        shoot9 = f.pathBuilder()
                .addPath(new BezierCurve(P_I9_RETURN, P_RAMP_CP, P_SHOOT))
                .setLinearHeadingInterpolation(H_180,H_114)
                .build();

        intakeExtra1 = f.pathBuilder()
                .addPath(new BezierCurve(P_SHOOT, P_EX_CP1, P_EX_END1))
                .setLinearHeadingInterpolation(H_114, H_150)
                .build();

        shootExtra1 = f.pathBuilder()
                .addPath(new BezierCurve(P_EX_END1, P_I6_END, P_SHOOT))
                .setLinearHeadingInterpolation(H_150, H_114)
                .build();

        intakeExtra2 = f.pathBuilder()
                .addPath(new BezierCurve(P_SHOOT, P_EX_CP2, P_EX_END2))
                .setLinearHeadingInterpolation(H_114, H_150)
                .build();

        shootExtra2 = f.pathBuilder()
                .addPath(new BezierCurve(P_EX_END2, P_I6_END, P_SHOOT))
                .setLinearHeadingInterpolation(H_150,H_114)
                .build();

        intakeExtra3 = f.pathBuilder()
                .addPath(new BezierCurve(P_SHOOT, P_EX_CP3, P_EX_END3))
                .setLinearHeadingInterpolation(H_114, H_150)
                .build();

        shootExtra3 = f.pathBuilder()
                .addPath(new BezierCurve(P_EX_END3, P_I6_END, P_SHOOT))
                .setLinearHeadingInterpolation(H_150,H_114)
                .build();

        leave = f.pathBuilder()
                .addPath(new BezierLine(P_SHOOT, P_LEAVE_END))
                .setConstantHeadingInterpolation(H_114)
                .build();
    }
}