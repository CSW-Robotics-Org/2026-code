package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

/**
 * Subsystem wrapper for interacting with a Limelight camera.
 * Provides helper methods for changing pipelines and retrieving
 * vision data such as robot pose and target pose.
 */
public class LimeLight extends SubsystemBase {

    /** Name of the Limelight as defined in the Limelight web interface */
    private String limelightName;

    // Calibration state
    private boolean isCalibrating = false;
    private boolean exposureLocked = false;
    private double calibrationStartTime = 0;
    private double lockedExposure = 20;

    private NetworkTable table;

    private boolean timerStarted = false;

    /**
     * Creates a new Limelight subsystem.
     *
     * @param name The network name of the Limelight (ex: "limelight")
     */
    public LimeLight(String name) {
        this.limelightName = name;

        this.table = NetworkTableInstance.getDefault().getTable(name);
    }

    /**
     * Sets the Limelight pipeline to the AprilTag detection pipeline.
     * This pipeline should be configured in the Limelight dashboard.
     */
    public void setAprilTag() {
        LimelightHelpers.setPipelineIndex(limelightName, 0);
    }

    /**
     * Sets the Limelight pipeline to the reflective target pipeline.
     * Useful for detecting retroreflective tape targets.
     */
    public void setReflective() {
        LimelightHelpers.setPipelineIndex(limelightName, 1);
    }

    /**
     * Checks whether the Limelight currently detects a valid target.
     *
     * @return true if a target is visible, false otherwise
     */
    public boolean hasTarget() {
        return LimelightHelpers.getTV(limelightName);
    }

    /**
     * Gets the robot's estimated field pose from the Limelight.
     * This is typically used for vision-based pose estimation.
     *
     * The returned array contains:
     * [X, Y, Z, roll, pitch, yaw]
     *
     * @return array containing the robot pose in field coordinates
     */
    public double[] getBotPose() {
        return LimelightHelpers.getBotPose(limelightName);
    }

    /**
     * Gets the pose of the detected target relative to the camera.
     *
     * The returned array contains:
     * [X, Y, Z, roll, pitch, yaw]
     *
     * @return array representing the target pose in camera space
     */
    public double[] getTargetPoseCameraSpace() {
        return LimelightHelpers.getTargetPose_CameraSpace(limelightName);
    }

    /**
     * Gets the pose of the detected target relative to the robot.
     *
     * The returned array contains:
     * [X, Y, Z, roll, pitch, yaw]
     *
     * @return array representing the target pose in robot space
     */
    public double[] getTargetPoseRobotSpace() {
        return LimelightHelpers.getTargetPose_RobotSpace(limelightName);
    }

    /**
     * Gets the robot's pose relative to the detected target.
     * This is often used for calculating distance and angle to a target.
     *
     * The returned array contains:
     * [X, Y, Z, roll, pitch, yaw]
     *
     * @return array representing the robot pose in target space
     */
    public double[] getRobotPoseTargetSpace() {
        return LimelightHelpers.getBotPose_TargetSpace(limelightName);
    }

    /**
     * Call this in robotInit() or disabledInit()
     * while pointed at an AprilTag.
     */
    public void initVisionCalibration() {
        setAprilTag();

        // Turn LEDs on
        LimelightHelpers.setLEDMode_ForceOn(limelightName);

        // Enable auto exposure
        table.getEntry("ae_mode").setNumber(1);

        calibrationStartTime = Timer.getFPGATimestamp();
        isCalibrating = true;
        exposureLocked = false;
    }

    private void handleCalibration() {
        if (!isCalibrating || exposureLocked) return;

        // Wait until we actually see a tag
        if (hasTarget() && !timerStarted) {
            calibrationStartTime = Timer.getFPGATimestamp();
            timerStarted = true;
        }

        // If we haven't seen a tag yet, do nothing
        if (!timerStarted) return;

        double timeElapsed = Timer.getFPGATimestamp() - calibrationStartTime;

        // Only lock after seeing tag + delay
        if (hasTarget() && timeElapsed > 2.0) {

            lockedExposure = table.getEntry("exposure").getDouble(20);

            lockedExposure = Math.max(5, Math.min(50, lockedExposure));

            table.getEntry("ae_mode").setNumber(0);
            table.getEntry("exposure").setNumber(lockedExposure);

            exposureLocked = true;
            isCalibrating = false;
        }
    }

    /**
     * Runs every robot loop (~20ms).
     * Sends useful Limelight data to SmartDashboard for debugging.
     */
    @Override
    public void periodic() {
        handleCalibration();

        SmartDashboard.putBoolean("LL Target", hasTarget());
        SmartDashboard.putBoolean("LL Calibrating", isCalibrating);
        SmartDashboard.putBoolean("LL Exposure Locked", exposureLocked);
        SmartDashboard.putNumber("LL Exposure", lockedExposure);
    }

    /**
     * Runs periodically during simulation.
     * Currently unused.
     */
    @Override
    public void simulationPeriodic() {}
}