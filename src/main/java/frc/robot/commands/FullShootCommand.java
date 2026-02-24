package frc.robot.commands;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;

public class FullShootCommand extends Command {

    private final Turret turret;
    private final LimeLight limelight;
    private final CommandSwerveDrivetrain drivetrain;
    private final Hopper hopper;
    public double shooterPowerOffset = 0;
    public double turretAngleOffset = 0;

    /**
     * Constructor of the command. Executes everything needed to shoot.
     * @param turret (turret)
     * @param limelight (limelight on the turret)
     * @param drivetrain (drivetrain for pos estimation)
     * @param hopper (hopper to feed balls)
     */
    public FullShootCommand(Turret turret, LimeLight limelight, CommandSwerveDrivetrain drivetrain,Hopper m_hopper) {
        this.turret = turret;
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.hopper = m_hopper;
    }

    @Override
    public void execute() {
        // Calculate speed and shooter power each tick
        double shooterPower = TurretTracking.ShooterPower(limelight, drivetrain,turret);
        turret.setShooterMotor(shooterPower + shooterPowerOffset);
        // Calculate speed and shooter power each tick
        double turretAngle = TurretTracking.TurretLineup(limelight, drivetrain, turret);
        turret.setTargetAngle(turretAngle+turretAngleOffset);
        
        if (turret.atSpeed()){
            turret.setFeederMotor(0.6);
            hopper.setHopperMotor(0.6);
        }
    }

    @Override
    public void end(boolean interrupted) {
        turret.setShooterMotor(0);
        turret.setTargetAngle(0);
        turret.setFeederMotor(0);
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
     * A command to adjust the power offset of the shooter.
     * @param amount (degrees -85,85) of angle to offset by.
     */
    public void adjustAngleOffset(double amount) {
        turretAngleOffset += amount;
        turretAngleOffset = MathUtil.clamp(turretAngleOffset, 0.0, 1.0);
    }
    
    /**
     * A command that returns the shooter power offset
     * @return turretAngleOffset (degrees -85,85)
     */
    public double getAngleOffset() {
        return turretAngleOffset;
    }
}
