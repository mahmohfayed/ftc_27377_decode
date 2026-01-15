//package org.firstinspires.ftc.teamcode.testing;
//
//// ... (imports are unchanged)
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.Subsystems.BasicRobot;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import java.util.List;
//@Disabled
//
//@Autonomous(name = "April Tag with PP test", group = "Opmode")
////@Configurable
//@SuppressWarnings("FieldCanBeLocal")
//public class AprilTagPatternAuto extends LinearOpMode {
//    // ... (Pose and PathChain definitions are unchanged)
//    private final ElapsedTime runtime = new ElapsedTime();
//    private final Pose startPose = new Pose(72, 120, Math.toRadians(90));
//    private final Pose scorePose = new Pose(72, 20, Math.toRadians(115));
//    private final Pose PPGPose = new Pose(100, 83.5, Math.toRadians(0));
//    private final Pose PGPPose = new Pose(100, 59.5, Math.toRadians(0));
//    private final Pose GPPPose = new Pose(100, 35.5, Math.toRadians(0));
//    private PathChain grabPPG, scorePPG, grabPGP, scorePGP, grabGPP, scoreGPP;
//
//
//    //set April Tag values to specific patterns
//    private static final int PPG_TAG_ID = 23;
//    private static final int PGP_TAG_ID = 22;
//    private static final int GPP_TAG_ID = 21;
//    private static final boolean USE_WEBCAM = true;
//    private VisionPortal visionPortal;
//    private AprilTagProcessor aprilTag;
//    private AprilTagDetection desiredTag = null;
//
//    public BasicRobot robot = new BasicRobot();
//
//
//
//    // Other variables
//    private Pose currentPose;
//    private Follower follower;
//    private TelemetryManager panelsTelemetry;
//    private int pathStatePPG, pathStatePGP, pathStateGPP;
//
//    // --- BUG FIX 1: Use a single variable to lock in the detected path ---
//    // Use an enum for clarity or integers if you prefer. 0 = none, 1 = PPG, 2 = PGP, 3 = GPP
//    private int detectedPath = 0;
//
//    // ... (log, intakeArtifacts, shootArtifacts methods are unchanged)
//    private void log(String caption, Object... text) {
//        if (text.length == 1) {
//            telemetry.addData(caption, text[0]);
//            panelsTelemetry.debug(caption + ": " + text[0]);
//        } else if (text.length >= 2) {
//            StringBuilder message = new StringBuilder();
//            for (int i = 0; i < text.length; i++) {
//                message.append(text[i]);
//                if (i < text.length - 1) message.append(" ");
//            }
//            telemetry.addData(caption, message.toString());
//            panelsTelemetry.debug(caption + ": " + message);
//        }
//    }
//    public void intakeArtifacts(){
//        robot.intakeArtifacts();
//        // intake.artifactIntake();
//
//    }
//    public void shootArtifacts() {
//        robot.shootArtifacts();
//
//
//    }
//
//
//    @Override
//    public void runOpMode() {
//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startPose);
//        initAprilTag();
//
//        if (USE_WEBCAM) {
//            setManualExposure(6, 250);
//        }
//
//        log("Status", "Initialized. Waiting for AprilTag detection...");
//        robot.init(hardwareMap);
//
//        // --- BUG FIX 2: Detect the AprilTag during the init phase ---
//        while (opModeInInit()) {
//            robot.init(hardwareMap);
//
//            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//            for (AprilTagDetection detection : currentDetections) {
//                if (detection.metadata != null) {
//                    if (detection.id == PPG_TAG_ID) {
//                        detectedPath = 1; // 1 for PPG
//                        desiredTag = detection;
//                        break;
//                    } else if (detection.id == PGP_TAG_ID) {
//                        detectedPath = 2; // 2 for PGP
//                        desiredTag = detection;
//                        break;
//                    } else if (detection.id == GPP_TAG_ID) {
//                        detectedPath = 3; // 3 for GPP
//                        desiredTag = detection;
//                        break;
//                    }
//                }
//            }
//
//            if (desiredTag != null) {
//                log("Tag Found", "ID: %d, Path %d selected. Ready to start!", desiredTag.id, detectedPath);
//            } else {
//                log("Status", "Searching for tags...");
//            }
//            telemetry.update();
//            sleep(20); // Small delay to avoid spamming the CPU
//        }
//
//        // --- BUG FIX 3: Build only the required path ONCE after start ---
//        if (detectedPath == 1) {
//            buildPathsPPG();
//            setpathStatePPG(0);
//        } else if (detectedPath == 2) {
//            buildPathsPGP();
//            setpathStatePGP(0);
//        } else if (detectedPath == 3) {
//            buildPathsGPP();
//            setpathStateGPP(0);
//        } else {
//            // No tag was found, so do nothing.
//            log("Error", "No AprilTag was detected during init.");
//            telemetry.update();
//        }
//
//        runtime.reset();
//        while (opModeIsActive()) {
//
//            follower.update();
//            panelsTelemetry.update();
//            currentPose = follower.getPose();
//
//            // --- BUG FIX 4: Run only the selected state machine ---
//            if (detectedPath == 1) {
//                updateStateMachinePPG();
//            } else if (detectedPath == 2) {
//                updateStateMachinePGP();
//            } else if (detectedPath == 3) {
//                updateStateMachineGPP();
//            }
//            // No need for an else block here, as it will just do nothing if no path was selected.
//
//            log("Elapsed", runtime.toString());
//            log("X", currentPose.getX());
//            log("Y", currentPose.getY());
//            log("Heading", currentPose.getHeading());
//            telemetry.update();
//        }
//    }
//
//    // ... (buildPaths, updateStateMachine, and setter methods are unchanged)
//    public void buildPathsPPG() {
//        grabPPG = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, PPGPose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), PPGPose.getHeading())
//                .build();
//        scorePPG = follower.pathBuilder()
//                .addPath(new BezierLine(PPGPose, scorePose))
//                .setLinearHeadingInterpolation(PPGPose.getHeading(), scorePose.getHeading())
//                .build();
//    }
//
//    public void buildPathsPGP() {
//        grabPGP = follower
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(56.000, 8.000), new Pose(55.920, 59.812))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
//                .addPath(
//                        new BezierLine(new Pose(55.920, 59.812), new Pose(35.437, 60.222))
//                )
//                .setTangentHeadingInterpolation()
//                .addPath(
//                        new BezierCurve(
//                                new Pose(35.437, 60.222),
//                                new Pose(0.615, 63.294),
//                                new Pose(0.000, 56.125),
//                                new Pose(27.448, 43.425)
//                        )
//                )
//                .setTangentHeadingInterpolation()
//                .addPath(
//                        new BezierCurve(
//                                new Pose(27.448, 43.425),
//                                new Pose(51.004, 34.208),
//                                new Pose(71.283, 42.606)
//                        )
//                )
//                .setTangentHeadingInterpolation()
//                .build();
//        scorePGP = follower.pathBuilder()
//                .addPath(new BezierLine(PGPPose, scorePose))
//                .setLinearHeadingInterpolation(PGPPose.getHeading(), scorePose.getHeading())
//                .build();
//    }
//
//    public void buildPathsGPP() {
//        grabGPP = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, GPPPose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), GPPPose.getHeading())
//                .build();
//        scoreGPP = follower.pathBuilder()
//                .addPath(new BezierLine(GPPPose, scorePose))
//                .setLinearHeadingInterpolation(GPPPose.getHeading(), scorePose.getHeading())
//                .build();
//    }
//
//    public void updateStateMachinePPG() {
//        switch (pathStatePPG) {
//            case 0:
//                follower.followPath(grabPPG);
//                setpathStatePPG(1);
//                break;
//            case 1:
//                if (!follower.isBusy()) {
//                    follower.followPath(scorePPG);
//                    setpathStatePPG(-1);
//                }
//                break;
//        }
//    }
//
//    public void updateStateMachinePGP() {
//        switch (pathStatePGP) {
//            case 0:
//                follower.followPath(grabPGP);
//                setpathStatePGP(1);
//                break;
//            case 1:
//                if (!follower.isBusy()) {
//                    follower.followPath(scorePGP);
//                    setpathStatePGP(-1);
//                }
//                break;
//        }
//    }
//
//    public void updateStateMachineGPP() {
//        switch (pathStateGPP) {
//            case 0:
//                follower.followPath(grabGPP);
//                setpathStateGPP(1);
//                break;
//            case 1:
//                if (!follower.isBusy()) {
//                    follower.followPath(scoreGPP);
//                    setpathStateGPP(-1);
//                }
//                break;
//        }
//    }
//
//    void setpathStatePPG(int newPathState) { this.pathStatePPG = newPathState; }
//    void setpathStatePGP(int newPathState) { this.pathStatePGP = newPathState; }
//    void setpathStateGPP(int newPathState) { this.pathStateGPP = newPathState; }
//
//
//    // ... (initAprilTag and setManualExposure methods are unchanged)
//    private void initAprilTag() {
//        aprilTag = new AprilTagProcessor.Builder().build();
//        aprilTag.setDecimation(2);
//        if (USE_WEBCAM) {
//            visionPortal = new VisionPortal.Builder()
//                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                    .addProcessor(aprilTag)
//                    .build();
//        } else {
//            visionPortal = new VisionPortal.Builder()
//                    .setCamera(BuiltinCameraDirection.FRONT)
//                    .addProcessor(aprilTag)
//                    .build();
//        }
//    }
////
//    private void setManualExposure(int exposureMS, int gain) {
//        if (visionPortal == null) return;
//        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            telemetry.addData("Camera", "Waiting");
//            telemetry.update();
//            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
//                sleep(20);
//            }
//            telemetry.addData("Camera", "Ready");
//            telemetry.update();
//        }
//        // ... (the rest of the method is unchanged)
//    }
//}
