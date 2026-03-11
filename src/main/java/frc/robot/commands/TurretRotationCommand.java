package frc.robot.commands;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TurretRotationCommand extends Command {

    private final Turret turret;
    private final LimeLight limelight;
    public double angleError;

    // creates a pid for the turret
    private static final PIDController turretPID = new PIDController(0.11, 0, 0);

    /**
     * Constructor of the command. Sets the turret target angle based on pos data.
     * @param turret (turret)
     * @param limelight (limelight on the turret)
     */
    public TurretRotationCommand(Turret turret, LimeLight limelight) {
        this.turret = turret;
        this.limelight = limelight;
    }

    @Override
    public void execute() {

       double[] raw = limelight.getRobotPoseTargetSpace();

        Pose3d botInTargetSpace = new Pose3d(
            new Translation3d(raw[0], raw[1], raw[2]),
            new Rotation3d(
                Math.toRadians(raw[3]),  // roll
                Math.toRadians(raw[4]),  // pitch
                Math.toRadians(raw[5])   // yaw
            )
        );


        // hub center relative to the tag
        Translation3d goalInTargetSpace = new Translation3d(0, 0, -0.6);

        // vector from robot to goal
        Translation3d robotToGoal = goalInTargetSpace.minus(botInTargetSpace.getTranslation());

        angleError = Math.toDegrees(Math.atan2(robotToGoal.getX(), robotToGoal.getZ()));

        double turretSpeed = turretPID.calculate(angleError);

        turret.setRotationMotor(turretSpeed);

        if (!limelight.hasTarget()) {
            turret.setRotationMotor(0);
            return;
        }
                
    }

    /**
     * Method to see if the turret is pointed the right way
     * @return true if turret is lined up
     */
    public boolean rotationReady(){
        return Math.abs(angleError)
        < 2;
    }

    @Override
    public void end(boolean interrupted) {
        turret.setRotationMotor(0);
    }

    @Override
    public boolean isFinished() {
        return false; // never ends on its own
    }
}
