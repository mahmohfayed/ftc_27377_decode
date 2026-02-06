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
    public PathChain intakeHuman;
    public PathChain shootHuman;


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

    public static Pose P_START = new Pose(56.047, 8.103, Math.toRadians(90));
    public static Pose P_SHOOT = new Pose(58.000, 16.000);

    // Intake 3
    public static Pose P_I3_CP = new Pose(58.748, 31.2);
    public static Pose P_I3_END = new Pose(40.853, 31.2);
    public static Pose P_I3_WALL = new Pose(26, 31.2);

    // Intake 6
    public static Pose P_I6_CP = new Pose(57.060, 57);
    public static Pose P_I6_END = new Pose(39.165, 57);
    public static Pose P_I6_WALL = new Pose(25, 57);
    int x= 0; //test
    // Ramp
    public static Pose P_RAMP_CP = new Pose(45.749, 68.877);
    public static Pose P_RAMP_END = new Pose(50.982, 69.383);
    public static Pose P_RAMP_WALL = new Pose(23, 69.383);
    public static Pose P_RAMP_RET = new Pose(50.982, 69.383);

    // Intake 9
    public static Pose P_I9_CP = new Pose(58.410, 82);
    public static Pose P_I9_END = new Pose(47.606, 82);
    public static Pose P_I9_WALL = new Pose(22, 82);
    public static Pose P_I9_RETURN = new Pose(47.437, 82);

    public static Pose P_HP_START = new Pose(58.000, 16.000);

    public static Pose P_HP_CP1 = new Pose(36.696, 57.580);
    public static Pose P_HP_END = new Pose(6.005, 25.960);

    public static Pose P_HP_WALL = new Pose(7.5, 14.3);
    public static Pose P_HP_RETURN = new Pose(6.036, 26.155);


    // Extra balls
    public static Pose P_EX_CP1 = new Pose(56.891, 60.436);
    public static Pose P_EX_END1 = new Pose(11.648, 61.787);

    public static Pose P_EX_CP2 = new Pose(56.722, 60.774);
    public static Pose P_EX_END2 = new Pose(11.817, 61.787);

    public static Pose P_EX_CP3 = new Pose(57.397, 60.436);
    public static Pose P_EX_END3 = new Pose(11.817, 61.449);

    // Leave
    public static Pose P_LEAVE_END = new Pose(57.735, 34.101);

    public static double H_90  = Math.toRadians(90);
    public static double H_180  = Math.toRadians(180);
    public static double H_114 = Math.toRadians(115); // heading for far

    public static double H_150 = Math.toRadians(150);
     public static double H_270 = Math.toRadians(270);
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

        P_HP_START = P_HP_START.mirror();

        P_HP_CP1 = P_HP_CP1.mirror();
        P_HP_END = P_HP_END.mirror();

        P_HP_WALL = P_HP_WALL.mirror();
        P_HP_RETURN = P_HP_RETURN.mirror();

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
        H_270 = mirrorAngleBlue(H_270);
    }

    // =============================
    // CONSTRUCTOR
    // =============================

public void human6Build() {
    shootPreload = f.pathBuilder()
            .addPath(new BezierLine(P_START, P_SHOOT))
            .setLinearHeadingInterpolation(H_90,H_114)
            .build();
    intakeHuman = f.pathBuilder()
            .addPath(new BezierCurve(
                    P_SHOOT, P_HP_CP1, P_HP_END))
            .setLinearHeadingInterpolation(H_114, H_270)
            .addPath(new BezierLine(P_HP_END, P_HP_WALL))
            .setTangentHeadingInterpolation()
            .addPath(new BezierLine(P_HP_WALL, P_HP_RETURN))
            .setTangentHeadingInterpolation()
            .setReversed()
            .build();
    shootHuman = f.pathBuilder()
            .addPath(new BezierCurve(P_HP_RETURN, P_HP_CP1, P_SHOOT))
            .setLinearHeadingInterpolation(H_270, H_114)
            .build();
    intakeHuman = f.pathBuilder()
            .addPath(new BezierCurve(
                    P_SHOOT, P_HP_CP1, P_HP_END))
            .setLinearHeadingInterpolation(H_114, H_270)
            .addPath(new BezierLine(P_HP_END, P_HP_WALL))
            .setTangentHeadingInterpolation()
            .addPath(new BezierLine(P_HP_WALL, P_HP_RETURN))
            .setTangentHeadingInterpolation()
            .setReversed()
            .build();
    shootHuman = f.pathBuilder()
            .addPath(new BezierCurve(P_HP_RETURN, P_HP_CP1, P_SHOOT))
            .setLinearHeadingInterpolation(H_270, H_114)
            .build();


}
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
        intakeHuman = f.pathBuilder()
                .addPath(new BezierCurve(
                        P_SHOOT, P_HP_CP1, P_HP_END))
                .setLinearHeadingInterpolation(H_114, H_270)
                .addPath(new BezierLine(P_HP_END, P_HP_WALL))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(P_HP_WALL, P_HP_RETURN))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        shootHuman = f.pathBuilder()
                .addPath(new BezierCurve(P_HP_RETURN, P_HP_CP1, P_SHOOT))
                .setLinearHeadingInterpolation(H_270, H_114)
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