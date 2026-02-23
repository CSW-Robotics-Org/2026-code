package frc.robot.commands;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TurretPowerCommand extends Command {

    private final Turret turret;
    private final LimeLight limelight;
    private final CommandSwerveDrivetrain drivetrain;
    public double shooterPowerOffset = 0;

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
        // Calculate speed and shooter power each tick
        double shooterPower = TurretTracking.ShooterPower(limelight, drivetrain,turret);
        turret.setShooterMotor(shooterPower + shooterPowerOffset);
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
}
