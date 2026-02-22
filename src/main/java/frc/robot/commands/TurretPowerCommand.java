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

    // adjusts the shooter power offset
    public void adjustOffset(double amount) {
        shooterPowerOffset += amount;
        shooterPowerOffset = MathUtil.clamp(shooterPowerOffset, 0.0, 1.0);
    }
    // gets the offset
    public double getOffset() {
        return shooterPowerOffset;
    }

    @Override
    public boolean isFinished() {
        return false; // never ends on its own
    }
}
