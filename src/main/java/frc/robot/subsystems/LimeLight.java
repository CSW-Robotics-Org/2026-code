
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

    /**
     * Updates the drivetrain Pose2d from a turret-mounted Limelight
     * that gives a 6-element field-relative robot pose:
     * [X, Y, Z, roll, pitch, yaw] in meters/radians.
     */
    public void updateRobotPoseFromLimelight(LimeLight limelight, 
                                            CommandSwerveDrivetrain drivetrain,
                                            double turretAngleRad) {
        // drivetrainPoseEstimator is a SwerveDrivePoseEstimator in your drivetrain subsystem
        // turretAngleRad is your current turret angle relative to robot front
        // limelight.botPos is [X, Y, Z, roll, pitch, yaw] in field coordinates

        if (limelight.tv == 1) {
            double limelightX = limelight.botPos[0];
            double limelightY = limelight.botPos[2]; // forward is Z in Limelight, X is lateral

            // Offset the camera based on turret rotation
            double camOffsetX = 0; // forward
            double camOffsetY = 0;   // lateral

            double cos = Math.cos(turretAngleRad);
            double sin = Math.sin(turretAngleRad);

            double offsetX = camOffsetX * cos - camOffsetY * sin;
            double offsetY = camOffsetX * sin + camOffsetY * cos;

            Pose2d visionPose = new Pose2d(
                limelightX - offsetX,
                limelightY - offsetY,
                drivetrain.getState().Pose.getRotation() // or use limelightYaw if you trust it
            );

            drivetrain.addVisionMeasurement(visionPose, Timer.getFPGATimestamp());
        }
    }


    @Override
    public void periodic() {

        targetPosCameraSpace = NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);

        botPos = NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("botpose").getDoubleArray(new double[6]);

        targetPoseRobotSpace = NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("targetpose_robotspace").getDoubleArray(new double[6]);

        tv = NetworkTableInstance.getDefault().getTable(m_network_table_key).getEntry("tv").getDouble(0);
        
        // updates the robot possition from the limelight
        this.updateRobotPoseFromLimelight(this, drivetrain, m_turret.getAngle());

        System.out.println("X: " + targetPoseRobotSpace[0] + ", Y: " + targetPoseRobotSpace[1] + ", TV: " + tv);
        
    }

    @Override
    public void simulationPeriodic() {}
}