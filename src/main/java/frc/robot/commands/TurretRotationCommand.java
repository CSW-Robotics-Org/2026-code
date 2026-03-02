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
    private final CommandSwerveDrivetrain drivetrain;

    // Hub position (meters)
    private final double HUB_X = 8.27;   // lateral
    private final double HUB_Z = 4.03;   // forward

    // creates a pid for the turret
    private static final PIDController turretPID = new PIDController(0.0055, 0, 0.0);

    public Translation3d targetPos;
    public Pose3d tagPos;
    Transform3d tagToBoxCenter = new Transform3d(
        new Translation3d(-0.5969, 0, 0),
        new Rotation3d()
    );
    public Pose3d boxCenterPos;
    public double angleError;


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

        // if we cant see a tag dont move
        if (limelight.tv == 0){
            turret.setRotationMotor(0);
        }

        // Use Limelight for pos 3s
        tagPos = new Pose3d(
            limelight.robotPosTargetSpace[0],
            limelight.robotPosTargetSpace[1],
            limelight.robotPosTargetSpace[2],
            new Rotation3d(
                Math.toRadians(limelight.robotPosTargetSpace[3]),
                Math.toRadians(limelight.robotPosTargetSpace[4]),
                Math.toRadians(limelight.robotPosTargetSpace[5])
            )
        );

        // set the box center pos
        boxCenterPos = tagPos.transformBy(tagToBoxCenter);
        // find the target pose
        targetPos = boxCenterPos.getTranslation();
        
        // angle error to minimize
        angleError = Math.toDegrees(
        Math.atan2(targetPos.getX(), targetPos.getZ())
        )+1;

        // We want angleError -> 0
        double output = -turretPID.calculate(angleError, 0);

        if (Math.abs(angleError) < 2){
            output = 0;
        }
        if (limelight.tv == 0){
            output = 0;
        }
        // clamp output speed
        output = MathUtil.clamp(output, -0.35, 0.35);
        // Calculate speed and shooter power each tick
        System.out.println("Angle Error: " + angleError);
        System.out.println("output: " + output);
        // turret.setRotationMotor(output);
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
