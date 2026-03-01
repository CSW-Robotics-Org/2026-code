
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class LimeLight extends SubsystemBase {

    private String m_network_table_key = "limelight";

    // offsets for the camera
    public double xoffset;
    public double zoffset;
    public double rotoffset;
    private CommandSwerveDrivetrain drivetrain;
    private Turret m_turret;

    // testing commit protection
    // surely this works the second time

    /**
     * Constructor
     * @param network_table_key (networktable key of the limelight)
     * @param xoffset (x offset on the robot, left and right plane)
     * @param zoffset (z offset on the robot, forward and backward plane)
     * @param rotoffset (yaw offset from the robot)
     * @param drive (drivetrain for possition estimation)
     * @param turret (turret for possition estimation)
     */
    public LimeLight(String network_table_key,double xoffset, double zoffset, double rotoffset, CommandSwerveDrivetrain drive, Turret turret) {
        // sets the key for the limelight aka the name 
        m_network_table_key = network_table_key;

        // stores the offsets
        this.xoffset = xoffset;
        this.zoffset = zoffset;
        this.rotoffset = rotoffset;
        drivetrain = drive;
        m_turret = turret;

    }

    // 3d AT data
    public double tv = NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("tv").getDouble(0);

    public double[] targetPosCameraSpace = NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);

    public double[] botPos = NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("botpose").getDoubleArray(new double[6]);

    public double[] targetPoseRobotSpace = NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("targetpose_robotspace").getDoubleArray(new double[6]);

    public void setAprilTag() {
        NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("pipeline").setNumber(0);
    }

    public void setReflective() {
        NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("pipeline").setNumber(1);
    }


    @Override
    public void periodic() {

        targetPosCameraSpace = NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);

        botPos = NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("botpose").getDoubleArray(new double[6]);

        targetPoseRobotSpace = NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("targetpose_robotspace").getDoubleArray(new double[6]);

        tv = NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("tv").getDouble(0);

        // System.out.println("X: " + targetPoseRobotSpace[0] + ", Y: " + targetPoseRobotSpace[1] + ", TV: " + tv);
        
    }

    @Override
    public void simulationPeriodic() {}
}