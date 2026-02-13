
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight extends SubsystemBase {

    private String m_network_table_key = "limelight";

    // offsets for the camera
    public double xoffset;
    public double zoffset;
    public double rotoffset;

    private CommandSwerveDrivetrain drivetrain;
    private Turret turret;

    // testing commit protection
    // surely this works the second time

    public LimeLight(String network_table_key,double xoffset, double zoffset, double rotoffset, CommandSwerveDrivetrain drivetrain, Turret turret) {
        // sets the key for the limelight aka the name 
        m_network_table_key = network_table_key;

        // stores the offsets
        this.xoffset = xoffset;
        this.zoffset = zoffset;
        this.rotoffset = rotoffset;

        this.drivetrain = drivetrain;
        this.turret = turret;

    }

    // 3d AT data
    public double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDouble(0);

    public double[] targetPosCameraSpace = NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);

    public double[] botPos = NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("botpose").getDoubleArray(new double[6]);

    public double[] targetPoseRobotSpace = NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("targetpose_robotspace").getDoubleArray(new double[6]);

    public void setAprilTag() {
        NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("pipeline").setNumber(0);
    }

    public void setReflective() {
        NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("pipeline").setNumber(1);
    }

    /**
     * Updates the drivetrain Pose2d from a turret-mounted Limelight
     * that gives a 6-element field-relative robot pose:
     * [X, Y, Z, roll, pitch, yaw] in meters/radians.
     */
    public void updateRobotPoseFromLimelight(LimeLight limelight, 
                                            CommandSwerveDrivetrain drivetrain,
                                            double turretAngleRad) {
        if (limelight.tv != 1) return; // no target

        // Limelight field pose
        double fieldX = limelight.botPos[0]; // X in meters (lateral)
        double fieldZ = limelight.botPos[2]; // Z in meters (forward)
        double fieldYaw = limelight.botPos[5]; // yaw in radians

        // Camera offset relative to turret/robot center (forward, lateral, vertical)
        double camOffsetX = 0.3; // forward from robot center
        double camOffsetZ = 0;   // lateral offset
        // (vertical offset is ignored for Pose2d)

        // Rotate the offset by turret rotation
        double cos = Math.cos(turretAngleRad);
        double sin = Math.sin(turretAngleRad);

        double offsetX = camOffsetX * cos - camOffsetZ * sin;
        double offsetZ = camOffsetX * sin + camOffsetZ * cos;

        // Subtract offsets to get robot center position
        double robotX = fieldX - offsetX;
        double robotZ = fieldZ - offsetZ;

        // Update drivetrain Pose2d (ignoring vertical)
        drivetrain.resetPose(new Pose2d(robotX, robotZ, drivetrain.getState().Pose.getRotation()));
    }


    @Override
    public void periodic() {

        targetPosCameraSpace = NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);

        botPos = NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("botpose").getDoubleArray(new double[6]);

        targetPoseRobotSpace = NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("targetpose_robotspace").getDoubleArray(new double[6]);

        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDouble(0);

        this.updateRobotPoseFromLimelight(this,drivetrain,turret.getAngle());
        
        
    }

    @Override
    public void simulationPeriodic() {}
}