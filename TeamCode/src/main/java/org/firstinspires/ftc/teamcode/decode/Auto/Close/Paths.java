package org.firstinspires.ftc.teamcode.decode.Auto.Close;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {

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
    // POSES (ALL CENTRALIZED)
    // =============================

    public static Pose P_START = new Pose(112.094, 138.091);
    public static Pose P_SHOOT = new Pose(89.000, 102.000);

    // Intake 3
    public static Pose P_I3_CP        = new Pose(79.512, 88.291);
    public static Pose P_I3_END       = new Pose(95.550, 90);
    public static Pose P_I3_WALL      = new Pose(118 ,85.5);

    // Intake 6
    public static Pose P_I6_CP        = new Pose(73.435, 79.006);
    public static Pose P_I6_END       = new Pose(96.563, 68);
    public static Pose P_I6_WALL      = new Pose(132, 68);

    // Ramp
    public static Pose P_RAMP_CP      = new Pose(92.680, 75.123);
    public static Pose P_RAMP_END     = new Pose(130, 70.227);

    // Intake 9
    public static Pose P_I9_CP        = new Pose(99.939, 45);
    public static Pose P_I9_END       = new Pose(99.264, 45);
    public static Pose P_I9_WALL      = new Pose(125, 45);
    public static Pose P_I9_RETURN    = new Pose(99.433, 35.3);

    // Extra balls
    public static Pose P_EX_CP1       = new Pose(93.862, 66.851);
    public static Pose P_EX_END1      = new Pose(133.027, 60.774);

    public static Pose P_EX_CP2       = new Pose(93.862, 66.682);
    public static Pose P_EX_END2      = new Pose(133.027, 60.774);

    public static Pose P_EX_CP3       = new Pose(94.368, 66.513);
    public static Pose P_EX_END3      = new Pose(133.027, 60.605);

    // Leave
    public static Pose P_LEAVE_END    = new Pose(89.135, 52.164);
    public static double H_38  = Math.toRadians(38);
    public static double H_0  = Math.toRadians(0);
    public static double H_43  = Math.toRadians(43);
    public Paths(Follower follower) {
        f = follower;
    }
    public double mirrorAngleRad(double angle) {
        return Math.PI - angle;
    }
    public static boolean isPathRed = true;

    public void mirrorAll() {
        P_START     = P_START.mirror();
        P_SHOOT     = P_SHOOT.mirror();

        P_I3_CP     = P_I3_CP.mirror();
        P_I3_END    = P_I3_END.mirror();
        P_I3_WALL   = P_I3_WALL.mirror();

        P_I6_CP     = P_I6_CP.mirror();
        P_I6_END    = P_I6_END.mirror();
        P_I6_WALL   = P_I6_WALL.mirror();

        P_RAMP_CP   = P_RAMP_CP.mirror();
        P_RAMP_END  = P_RAMP_END.mirror();

        P_I9_CP     = P_I9_CP.mirror();
        P_I9_END    = P_I9_END.mirror();
        P_I9_WALL   = P_I9_WALL.mirror();
        P_I9_RETURN = P_I9_RETURN.mirror();

        P_EX_CP1    = P_EX_CP1.mirror();
        P_EX_END1   = P_EX_END1.mirror();

        P_EX_CP2    = P_EX_CP2.mirror();
        P_EX_END2   = P_EX_END2.mirror();

        P_EX_CP3    = P_EX_CP3.mirror();
        P_EX_END3   = P_EX_END3.mirror();

        P_LEAVE_END = P_LEAVE_END.mirror();

        H_0 =  mirrorAngleRad(H_0);
        H_38 =  mirrorAngleRad(H_38);
        H_43 =  mirrorAngleRad(H_43);
    }


    // =============================
    // BUILD ALL PATHS
    // =============================

    public void goal21Build() {

        // ---------- Preload ----------
        shootPreload = f.pathBuilder()
                .addPath(new BezierLine(P_START, P_SHOOT))
                .setLinearHeadingInterpolation(H_0,H_38)
                .build();

        // ---------- Intake + Shoot 3 ----------
        intake3 = f.pathBuilder()
                .addPath(new BezierCurve(P_SHOOT, P_I3_CP, P_I3_END))
                .setLinearHeadingInterpolation(H_38,H_0)
                .addPath(new BezierLine(P_I3_END, P_I3_WALL))
                .setConstantHeadingInterpolation(H_0)
                .addPath(new BezierLine(P_I3_WALL, P_I3_END))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        shoot3 = f.pathBuilder()
                .addPath(new BezierCurve(P_I3_END, P_I3_CP, P_SHOOT))
                .setLinearHeadingInterpolation(H_0,H_38)
                .build();

        // ---------- Intake + Shoot 6 ----------
        intake6 = f.pathBuilder()
                .addPath(new BezierCurve(P_SHOOT, P_I6_CP, P_I6_END))
                .setLinearHeadingInterpolation(H_38, H_0)
                .addPath(new BezierLine(P_I6_END, P_I6_WALL))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(P_I6_WALL, P_I6_END))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        shoot6 = f.pathBuilder()
                .addPath(new BezierCurve(P_I6_END, P_I6_CP, P_SHOOT))
                .setLinearHeadingInterpolation(H_0, H_38)
                .build();

        // ---------- Ramp ----------
        unloadRamp = f.pathBuilder()
                .addPath(new BezierCurve(P_SHOOT, P_RAMP_CP, P_RAMP_END))
                .setLinearHeadingInterpolation(H_38,H_0)
                .build();

        // ---------- Intake + Shoot 9 ----------
        intake9 = f.pathBuilder()
                .addPath(new BezierCurve(P_RAMP_END, P_I9_CP, P_I9_END))
                .setConstantHeadingInterpolation(H_0)
                .addPath(new BezierLine(P_I9_END, P_I9_WALL))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(P_I9_WALL, P_I9_RETURN))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        shoot9 = f.pathBuilder()
                .addPath(new BezierCurve(P_I9_RETURN, P_I9_CP, P_SHOOT))
                .setLinearHeadingInterpolation(H_0, H_38)
                .build();

        // ---------- Extra Balls ----------
        intakeExtra1 = f.pathBuilder()
                .addPath(new BezierCurve(P_SHOOT, P_EX_CP1, P_EX_END1))
                .setLinearHeadingInterpolation(H_38, H_43)
                .build();

        shootExtra1 = f.pathBuilder()
                .addPath(new BezierCurve(P_EX_END1, P_EX_CP1, P_SHOOT))
                .setLinearHeadingInterpolation(H_43, H_38)
                .build();

        intakeExtra2 = f.pathBuilder()
                .addPath(new BezierCurve(P_SHOOT, P_EX_CP2, P_EX_END2))
                .setLinearHeadingInterpolation(H_38, H_43)
                .build();

        shootExtra2 = f.pathBuilder()
                .addPath(new BezierCurve(P_EX_END2, P_EX_CP2, P_SHOOT))
                .setLinearHeadingInterpolation(H_43,H_38)
                .build();

        intakeExtra3 = f.pathBuilder()
                .addPath(new BezierCurve(P_SHOOT, P_EX_CP3, P_EX_END3))
                .setLinearHeadingInterpolation(H_38, H_43)
                .build();

        shootExtra3 = f.pathBuilder()
                .addPath(new BezierLine(P_EX_END3, P_SHOOT))
                .setLinearHeadingInterpolation(H_43, H_38)
                .build();

        // ---------- Leave ----------
        leave = f.pathBuilder()
                .addPath(new BezierLine(P_SHOOT, P_LEAVE_END))
                .setConstantHeadingInterpolation(H_38)
                .build();
    }
}
