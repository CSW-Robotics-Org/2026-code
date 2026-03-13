package frc.robot.commands;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

public class FullShootCommand extends Command {

    private final Turret turret;
    private final LimeLight limelight;
    private final Hopper hopper;
    private final Intake intake;
    public double shooterPowerOffset = 0;

    //private final TurretRotationCommand rotationCommand;
    private final TurretPowerCommand powerCommand;
    TurretTracking2 turretTracker;


    /**
     * Constructor of the command. Executes everything needed to shoot.
     * @param turret (turret)
     * @param limelight (limelight on the turret)
     * @param drivetrain (drivetrain for pos estimation)
     * @param hopper (hopper to feed balls)
     */
    public FullShootCommand(Turret turret, LimeLight limelight,Hopper m_hopper, Intake m_intake) {
        this.turret = turret;
        this.limelight = limelight;
        this.hopper = m_hopper;
        this.intake = m_intake;

        //rotationCommand = new TurretRotationCommand(turret, limelight);
        powerCommand = new TurretPowerCommand(turret, limelight);
        turretTracker = new TurretTracking2(turret, limelight);


    }

    @Override
    public void execute() {
        turretTracker.execute();
        powerCommand.execute();
        System.out.println("turret ready? : " + turretTracker.rotationReady() + ", Angle Error: " + turretTracker.tagError);
        System.out.println("shooter ready? : " + powerCommand.readyToShoot() + ", CurrentRPM, targetRPM: " + turret.s_encoder.getVelocity() + ", " + turret.targetRPM);

        // this will run if we all al ready to shoot and starts feeding balls
        if (turretTracker.rotationReady() && powerCommand.readyToShoot()) {

            hopper.setHopperMotor(0.5);
            intake.setIntakeMotor(0.3);
            turret.setFeederMotor(0.15);

        }

        // //if the shooter isnt ready to shoot stop the feeder
        // if (!powerCommand.readyToShoot() || !turretTracker.rotationReady()){
        //     turret.setFeederMotor(0);
        // }



    }

    @Override
    public void end(boolean interrupted) {
        turretTracker.end(interrupted);
        powerCommand.end(interrupted);
        hopper.setHopperMotor(0);
        turret.setFeederMotor(0);
        intake.setIntakeMotor(0);
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
        powerCommand.adjustPowerOffset(amount);
    }
    
    /**
     * A command that returns the shooter power offset
     * @return shooterPowerOffset (double 0,1)
     */
    public double getPowerOffset() {
        return powerCommand.getPowerOffset();
    }

}
