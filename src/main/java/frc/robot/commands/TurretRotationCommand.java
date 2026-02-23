package frc.robot.commands;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TurretRotationCommand extends Command {

    private final Turret turret;
    private final LimeLight limelight;
    private final CommandSwerveDrivetrain drivetrain;

    public TurretRotationCommand(Turret turret, LimeLight limelight, CommandSwerveDrivetrain drivetrain) {
        this.turret = turret;
        this.limelight = limelight;
        this.drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        // Calculate speed and shooter power each tick
        double turretAngle = TurretTracking.TurretLineup(limelight, drivetrain, turret);
        turret.setTargetAngle(turretAngle);
    }

    @Override
    public void end(boolean interrupted) {
        turret.setTargetAngle(0);
    }

    @Override
    public boolean isFinished() {
        return false; // never ends on its own
    }
}
