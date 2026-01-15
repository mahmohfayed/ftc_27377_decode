package org.firstinspires.ftc.teamcode.decode.Subsystems;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;
import java.util.concurrent.TimeUnit;

// This the subsystem for the Logitech c70 camera
public class C70 {
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag;

    public void init(HardwareMap hardwareMap) { // Initialize the camera

        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);


        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    public void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) {
            return;
        }

        // Wait for the camera to be open
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            // This loop will run in init() and wait for the camera.
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }

    public void getDetections(boolean targetFound, AprilTagDetection desiredTag, int DESIRED_TAG_ID) {


        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    targetFound = true;
                    desiredTag = detection;

                    break; // Found the tag, so exit the loop
                }
            }
        }
    }

    public void stop() {
        // Close the vision portal to free up resources
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}


