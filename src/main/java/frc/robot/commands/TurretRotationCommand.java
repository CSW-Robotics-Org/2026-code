package frc.robot.commands;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TurretRotationCommand extends Command {

    private final Turret turret;
    private final LimeLight limelight;
    private final CommandSwerveDrivetrain drivetrain;
    public double turretAngleOffset = 0;

    /**
     * Constructor of the command. Sets the turret target angle based on pos data.
     * @param turret (turret)
     * @param limelight (limelight on the turret)
     * @param drivetrain (drivetrain for pos estimation)
     */
    public TurretRotationCommand(Turret turret, LimeLight limelight, CommandSwerveDrivetrain drivetrain) {
        this.turret = turret;
        this.limelight = limelight;
        this.drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        // Calculate speed and shooter power each tick
        double turretAngle = TurretTracking.TurretLineup(limelight, drivetrain, turret);
        turret.setTargetAngle(turretAngle+turretAngleOffset);
    }

    @Override
    public void end(boolean interrupted) {
        turret.setTargetAngle(0);
    }

    @Override
    public boolean isFinished() {
        return false; // never ends on its own
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
