package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimeLight extends SubsystemBase {

    private String limelightName;

    // offsets
    public double xoffset;
    public double zoffset;
    public double rotoffset;

    public LimeLight(String name, double xoffset, double zoffset, double rotoffset) {

        this.limelightName = name;

        this.xoffset = xoffset;
        this.zoffset = zoffset;
        this.rotoffset = rotoffset;

    }

    public void setAprilTag() {
        LimelightHelpers.setPipelineIndex(limelightName, 0);
    }

    public void setReflective() {
        LimelightHelpers.setPipelineIndex(limelightName, 1);
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(limelightName);
    }

    public double[] getBotPose() {
        return LimelightHelpers.getBotPose(limelightName);
    }

    public double[] getTargetPoseCameraSpace() {
        return LimelightHelpers.getTargetPose_CameraSpace(limelightName);
    }

    public double[] getTargetPoseRobotSpace() {
        return LimelightHelpers.getTargetPose_RobotSpace(limelightName);
    }

    public double[] getRobotPoseTargetSpace() {
        return LimelightHelpers.getBotPose_TargetSpace(limelightName);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("LL Target", hasTarget());
    }

    @Override
    public void simulationPeriodic() {}
}