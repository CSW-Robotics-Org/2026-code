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
    public double shooterPowerOffset = 0;
    /**
     * Constructor of the command. Sets the shooter power based off of pos data.
     * @param turret (turret)
     * @param limelight (limelight on the turret)
     */
    public TurretPowerCommand(Turret turret, LimeLight limelight) {
        this.turret = turret;
        this.limelight = limelight;
    }

    @Override
    public void execute() {

        // gets the raw limelight data
        double[] raw = limelight.getTargetPoseRobotSpace();

        // puts it into a pose 3d
        Pose3d hubPos = new Pose3d(
            new Translation3d(raw[0], raw[1], raw[2]),
            new Rotation3d(raw[3],raw[4],raw[5])
        );

        // finds the hypotenuse
        double targetDistance = Math.hypot(hubPos.getX(), hubPos.getZ());
        
        // Quadratic formula for shooter power
        double shooterPow =0.46 + 0.006 * (targetDistance) + 0.009 * Math.pow(targetDistance, 2);
        
        // clamps the shooter power
        shooterPow = MathUtil.clamp(shooterPow,-1,1);

        // if the limelight has no target set the shooter power to 0
        if (!limelight.hasTarget()){
            shooterPow = 0;
        }

        // sets the shooter motor
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
        shooterPowerOffset = MathUtil.clamp(shooterPowerOffset, -1, 1.0);
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
