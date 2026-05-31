package org.firstinspires.ftc.teamcode.decode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.decode.Subsystems.HoodServo;

@TeleOp(name = "Maryam failing Ap Eng")
public class zainTeleOp1 extends LinearOpMode {

    private IMU imu;

    // Subsystems
    private HoodServo hoodServo;

    @Override
    public void runOpMode() throws InterruptedException {

        // -------- Motors --------
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRight = hardwareMap.dcMotor.get("backRightMotor");

        DcMotor flyWheelMotor = hardwareMap.dcMotor.get("rightShooterMotor");
        DcMotor followerWheelMotor = hardwareMap.dcMotor.get("leftShooterMotor");

        DcMotor intake = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor loader = hardwareMap.dcMotor.get("loaderMotor");

        // -------- Subsystem Init --------
        hoodServo = new HoodServo();
        hoodServo.init(hardwareMap);
        hoodServo.setHoodServo(0.3); // starting position

        // -------- Motor Directions --------
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        int text_x =0;//delete this
        // -------- IMU --------
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);

        double flyPower = 0;

        waitForStart();

        while (opModeIsActive()) {

            // -------- Field-Centric Mecanum --------
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.left_stick_button) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            frontLeft.setPower((rotY + rotX + rx) / denominator);
            backLeft.setPower((rotY - rotX + rx) / denominator);
            frontRight.setPower((rotY - rotX - rx) / denominator);
            backRight.setPower((rotY + rotX - rx) / denominator);

            // -------- Shooter --------
            flyPower += gamepad1.right_trigger * 0.05;
            flyPower -= gamepad1.left_trigger * 0.05;
            flyPower = Math.max(0, Math.min(1, flyPower));

            flyWheelMotor.setPower(flyPower);
            followerWheelMotor.setPower(flyPower);

            // -------- Intake --------
            if (gamepad1.right_bumper) {
                intake.setPower(1);
            } else if (gamepad1.left_bumper) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            // -------- Loader --------
            if (gamepad1.y) {
                loader.setPower(1);
            } else if (gamepad1.a) {
                loader.setPower(-1);
            } else if (gamepad1.dpad_up) {
                loader.setPower(1);
                intake.setPower(1);
            } else {
                loader.setPower(0);
            }

            // -------- Hood (2 Servos) --------
            if (gamepad1.x) {
                hoodServo.setHoodServo(0.4);
            } else if (gamepad1.b) {
                hoodServo.setHoodServo(0.45);
            } else if (gamepad1.dpad_down) {
                hoodServo.setHoodServo(0.5);
            }

            // -------- Telemetry --------
            telemetry.addData("Shooter Power", flyPower);
            telemetry.addData("Hood Position", hoodServo.getPosition());
            telemetry.update();
        }
    }
}
