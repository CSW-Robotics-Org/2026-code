package frc.robot.commands;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TurretPowerCommand extends Command {

    private final Turret turret;
    private final LimeLight limelight;
    private final CommandSwerveDrivetrain drivetrain;
    public double shooterPowerOffset = 0;

    // Hub position (meters)
    private final double HUB_X = 8.27;   // lateral
    private final double HUB_Z = 4.03;   // forward

    public Translation3d targetPos;
    public double targetDistance = 0;
    public Transform3d tagToBoxCenter = new Transform3d(
        new Translation3d(-0.5969, 0, 0),
        new Rotation3d()
    );

    public Pose3d boxCenterPos;
    public Pose3d tagPos;

    /**
     * Constructor of the command. Sets the shooter power based off of pos data.
     * @param turret (turret)
     * @param limelight (limelight on the turret)
     * @param drivetrain (drivetrain for pos estimation)
     */
    public TurretPowerCommand(Turret turret, LimeLight limelight, CommandSwerveDrivetrain drivetrain) {
        this.turret = turret;
        this.limelight = limelight;
        this.drivetrain = drivetrain;
    }

    @Override
    public void execute() {

        if (limelight.tv == 1) {
            tagPos = new Pose3d(
                limelight.targetPoseRobotSpace[0],
                limelight.targetPoseRobotSpace[1],
                limelight.targetPoseRobotSpace[2],
                new Rotation3d(
                    Math.toRadians(limelight.targetPoseRobotSpace[3]),
                    Math.toRadians(limelight.targetPoseRobotSpace[4]),
                    Math.toRadians(limelight.targetPoseRobotSpace[5])
                )
            );

            boxCenterPos = tagPos.transformBy(tagToBoxCenter);
            targetPos = boxCenterPos.getTranslation();
            // stores the target distance
            targetDistance = Math.hypot(targetPos.getX(), targetPos.getZ());

        } else {
            // rotate relative vector to hub
            double xRel = HUB_X - drivetrain.getState().Pose.getX();
            double zRel = HUB_Z - drivetrain.getState().Pose.getY();

            // stores the target distance
            targetDistance = Math.hypot(xRel, zRel);
        }

        // Quadratic formula for shooter power
        double shooterPow = 0.5 + 0.0199 * targetDistance + 0.00347 * Math.pow(targetDistance, 2);
        shooterPow = MathUtil.clamp(shooterPow,-1,1);
        turret.setShooterMotor(shooterPow + shooterPowerOffset);
    }

    @Override
    public void end(boolean interrupted) {
        turret.setShooterMotor(0);
    }

    @Override
    public boolean isFinished() {
        return false; // never ends on its own
    }

     /**
     * A command to adjust the angle offset of the turret.
     * @param amount (double 0,1 as percentage %) of power to offset by.
     */
    public void adjustPowerOffset(double amount) {
        shooterPowerOffset += amount;
        shooterPowerOffset = MathUtil.clamp(shooterPowerOffset, 0.0, 1.0);
    }
    
    /**
     * A command that returns the shooter power offset
     * @return shooterPowerOffset (double 0,1)
     */
    public double getPowerOffset() {
        return shooterPowerOffset;
    }

    /**
     * Method to see if we are ready to shoot
     * @return true if the shooter is at speed
     */
    public boolean readyToShoot(){
        return turret.atSpeed();
    }
}
