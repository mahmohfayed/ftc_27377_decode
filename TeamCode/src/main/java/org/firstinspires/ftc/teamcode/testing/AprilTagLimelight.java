package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@TeleOp

public class AprilTagLimelight extends LinearOpMode {

    private Limelight3A limelight;
    public IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight"); // init limelight
//            imu = hardwareMap.get(IMU.class, "imu");
//            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                    RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
//                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
//            imu.initialize(parameters);

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);// the pipline your using

        /*
         * Starts polling for data.
         */
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());


        }


        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));


        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("ta", result.getTa());
                //telemetry.addData("Botpose", botpose.toString());
                Pose3D botpose_mt2 = result.getBotpose_MT2();

                if (botpose_mt2 != null) {
                    double x = botpose_mt2.getPosition().x;
                    double y = botpose_mt2.getPosition().y;
                    telemetry.addData("MT2 Location:", "(" + x + "," + y + ")");
                }
            }
        } else {
            telemetry.addData("Limelight", "No Targets");

        }
        telemetry.update();
        limelight.stop();


    }
}
