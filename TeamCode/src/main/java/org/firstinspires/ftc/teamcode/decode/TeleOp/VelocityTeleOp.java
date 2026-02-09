package org.firstinspires.ftc.teamcode.decode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
@Disabled

@TeleOp(name = "VelocityTeleOp")
public class VelocityTeleOp extends LinearOpMode {
    private IMU imu;
    public Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {

        servo = hardwareMap.servo.get("hoodServo");
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRight = hardwareMap.dcMotor.get("backRightMotor");

        // Use MotorEx for flywheel motors
        MotorEx flyWheelMotor = new MotorEx(hardwareMap, "rightShooterMotor");
        MotorEx followerWheelMotor = new MotorEx(hardwareMap, "leftShooterMotor");

        DcMotor intake = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor loader = hardwareMap.dcMotor.get("loaderMotor");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Setup shooter motors for velocity control
        flyWheelMotor.resetEncoder();
        followerWheelMotor.resetEncoder();
        flyWheelMotor.setRunMode(Motor.RunMode.VelocityControl);
        followerWheelMotor.setRunMode(Motor.RunMode.VelocityControl);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        double flyPower = 0;
        double targetVelocity = 2600; // Target velocity in ticks per second

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // --- mecanum Drive Control ---
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.left_stick_button) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            // Shooter velocity control with triggers
            flyPower += gamepad1.right_trigger * 0.05;
            flyPower -= gamepad1.left_trigger * 0.05;
            flyPower = Math.max(0, Math.min(1, flyPower));

            // Set velocity based on flyPower
            double currentTargetVelocity = flyPower * targetVelocity;
            flyWheelMotor.setVelocity(currentTargetVelocity);
            followerWheelMotor.setVelocity(currentTargetVelocity);

            if (gamepad1.right_bumper) {
                intake.setPower(1);
            } else if (gamepad1.left_bumper) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            if (gamepad1.y) {
                loader.setDirection(DcMotorSimple.Direction.FORWARD);
                loader.setPower(1);
            } else if (gamepad1.a) {
                loader.setPower(-1);
            } else if(gamepad1.dpad_up){
                loader.setPower(1);
                intake.setPower(1);
            } else {
                loader.setPower(0);
            }

            if(gamepad1.x){
                servo.setPosition(0.5);
            }
            else if(gamepad1.b){
                servo.setPosition(0.75);
            }
            else if(gamepad1.dpad_left){
                servo.setPosition(1);
            }
            else if(gamepad1.dpad_right){
                servo.setPosition(0.25);
            }

            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Back Left Power", backLeftPower);
            telemetry.addData("Back Right Power", backRightPower);
            telemetry.addData("intake Power", intake.getPower());
            telemetry.addData("Target Velocity", currentTargetVelocity);
            telemetry.addData("Flywheel Velocity", flyWheelMotor.getVelocity());
            telemetry.addData("Follower Velocity", followerWheelMotor.getVelocity());
            telemetry.update();
        }
    }
}