package org.firstinspires.ftc.teamcode.decode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drivetrain {
    private Follower follower;
    private static DcMotorEx frontLeft;
    private static DcMotorEx frontRight;
    private static DcMotorEx backLeft;
    private static DcMotorEx backRight;

    public static final Pose RED_GOAL_POSE = new Pose(137.5, 143.2, 0);
    public static final Pose BLUE_GOAL_POSE = RED_GOAL_POSE.mirror();

    private IMU imu;

    public void init(HardwareMap hardwareMap){

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);



        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }


    private static void setPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
         double maxSpeed = 1.0;
         maxSpeed = Math.max(maxSpeed, Math.abs(frontLeftPower));
         maxSpeed = Math.max(maxSpeed, Math.abs(frontRightPower));
         maxSpeed = Math.max(maxSpeed, Math.abs(backLeftPower));
         maxSpeed = Math.max(maxSpeed, Math.abs(backRightPower));

         frontLeftPower /= maxSpeed;
         frontRightPower /= maxSpeed;
         backLeftPower /= maxSpeed;
         backRightPower /= maxSpeed;

        Drivetrain.setSafePower(frontLeft, frontLeftPower);
        Drivetrain.setSafePower(frontRight,frontRightPower);
        Drivetrain.setSafePower(backLeft, backLeftPower);
        Drivetrain.setSafePower(backRight, backRightPower);
    }

    // Thanks to FTC16072 for sharing this code!!
    public static void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backLeftPower = forward - right + rotate;
        double backRightPower = forward + right - rotate;

        setPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    public void stop(){
        setPowers(0,0,0,0);
    }

    public void driveFieldRelative (double x, double y, double rx){

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX *= 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        Drivetrain.setPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }
    static public void setSafePower(DcMotorEx motor, double targetPower){
        final double SLEW_RATE = 0.2;
        double currentPower = motor.getPower();
        double desiredChange = targetPower - currentPower;
        double limitedChange = Math.max(-SLEW_RATE, Math.min(desiredChange, SLEW_RATE));
        motor.setPower(currentPower += limitedChange);
    }



    public void printTelemetry() {
        telemetry.addData("Status", "Initialized");

        telemetry.update();
    }
//    }
//    public void drive(double x, double y, double rx){
//        driveFieldRelative(x,y,rx);
//    }
    public Pose getPose(){
        return follower.getPose();
    }

    public double getDistanceFromGoal(boolean red){
        Pose goalPose = RED_GOAL_POSE;
        Pose robotPose = getPose();

        return Math.sqrt(
                Math.pow(goalPose.getY() - robotPose.getY(), 2) + Math.pow(goalPose.getX() - robotPose.getX(), 2));
    }


    public double getAngleFromGoalDegrees(boolean red){
        Pose goalPose = RED_GOAL_POSE;
        Pose robotPose = getPose();

        return Math.toDegrees(Math.atan2(goalPose.getY() - robotPose.getY(),goalPose.getX() - robotPose.getX()));
    }

    public void setStartingPose(Pose pose){
        follower.setStartingPose(pose);
    }

}
