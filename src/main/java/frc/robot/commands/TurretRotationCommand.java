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
    public double error;

    // creates a pid for the turret
    private static final PIDController turretPID = new PIDController(0.11, .0075, 0);

    /**
     * Constructor of the command. Sets the turret target angle based on pos data.
     * @param turret (turret)
     * @param limelight (limelight on the turret)
     * @param drivetrain (drivetrain for pos estimation)
     */
    public TurretRotationCommand(Turret turret, LimeLight limelight) {
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

        error = hubPos.getX();

        //System.out.println("Hello world");
        // We want angleError -> 0
        double output = -turretPID.calculate(error, 0);

        // if we cant see an april tag dont move
        if (!limelight.hasTarget()){
            output = 0;
        }
        
        System.out.println("output: " + output);
        System.out.println("Angle Error: " + error);

        // clamp output speed
        output = MathUtil.clamp(output, -0.35, 0.35);
        // Calculate speed and shooter power each tick
        turret.setRotationMotor(output);
    }

    /**
     * Method to see if the turret is pointed the right way
     * @return true if turret is lined up
     */
    public boolean rotationReady(){
        return Math.abs(error)
        < 0.25;
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
