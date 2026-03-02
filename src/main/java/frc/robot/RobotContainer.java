// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.File;
import java.util.logging.LogManager;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveTracking;
import frc.robot.commands.FullShootCommand;
import frc.robot.commands.TurretPowerCommand;
import frc.robot.commands.TurretRotationCommand;
import frc.robot.commands.TurretTracking;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Turret;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // creates our controllers
    private final XboxController m_operator = new XboxController(2);
    private final Joystick l_joystick = new Joystick(0);
    private final Joystick r_joystick = new Joystick(1);


    // auto picker for command  /* Path follower */
    private SendableChooser<Command> autoChooser;

    // ID's will be changed
    public final Hopper m_hopper = new Hopper(35);
    public final Intake m_intake = new Intake(17, 7, 6);
    
    // creates our limelights
    public final Turret m_turret = new Turret(8, 10, 15);
    public final LimeLight limelight = new LimeLight("limelight-front",0,0,0,drivetrain,m_turret);
    
    public TurretPowerCommand ShooterPowerCommand = new TurretPowerCommand(m_turret,limelight,drivetrain);
    public TurretRotationCommand TurretAngleCommand = new TurretRotationCommand(m_turret,limelight,drivetrain);
    public FullShootCommand FullShoot = new FullShootCommand(m_turret,limelight,drivetrain,m_hopper);

    public RobotContainer() {
        configureBindings();

        // creates the autobuilder
        autoChooser = AutoBuilder.buildAutoChooser("Default");
        // puts the autobuilder on smart dashboard
        SmartDashboard.putData("Auto Mode", autoChooser);
        // warms up pathplanner so we dont have issues starting
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {

        Pigeon2 pigeon = new Pigeon2(0);
        var m_config = new MountPoseConfigs();
        m_config.MountPoseYaw = 90;
        pigeon.getConfigurator().apply(m_config);

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // Drivetrain will execute this command periodically
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-r_joystick.getY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-r_joystick.getX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-l_joystick.getX() * MaxAngularRate)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );




        // #### AUTO COMMANDS ####

            // Freeze wheels command
            NamedCommands.registerCommand("FreezeWheels", 
                new InstantCommand(()-> 
                    drivetrain.applyRequest(()-> 
                        drive.withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0)) 
                )
            );

            // Named command for shooting
            NamedCommands.registerCommand("Shoot", ShooterPowerCommand);
            
            // Named command that shoots balls from hopper through shooter
            NamedCommands.registerCommand("FeedAndShoot", 
                new SequentialCommandGroup(
                    // new InstantCommand(()-> m_hopper.setRollerMotor(0.5)),
                    // Commands.race(ShooterPowerCommand,)
                )
            );

       



        // ##### DRIVER CONTROLS #####

            // Theoretically resets the field reletive possitioning
            new JoystickButton(r_joystick,3).onTrue(drivetrain.runOnce(()-> drivetrain.seedFieldCentric()));
            
            // Theoretically applies the break works great in the sim
            new JoystickButton(r_joystick,5).whileTrue(drivetrain.applyRequest(() -> brake));

            // robot rel
            new JoystickButton(r_joystick,4).whileTrue(drivetrain.applyRequest(()-> 
                new SwerveRequest.RobotCentric()
                    .withVelocityX(-r_joystick.getY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-r_joystick.getX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-l_joystick.getX() * MaxAngularRate)
            
            ));



        // ##### OPERATOR CONTROLS #####

            // (Y) Button -> runs the shooter power command
            new JoystickButton(m_operator,4).whileTrue(ShooterPowerCommand);

            // (B) Button -> runs the turret rotation command
            new JoystickButton(m_operator,3).whileTrue(TurretAngleCommand);
            
            // (A) Button -> runs the feeder motor and the hopper motor
            new JoystickButton(m_operator, 2)
                .onTrue( new SequentialCommandGroup(
                    new InstantCommand(()->m_turret.setFeederMotor(0.5)),
                    new InstantCommand(()->m_hopper.setHopperMotor(0.5)
                    )))
                .onFalse(new SequentialCommandGroup(
                    new InstantCommand(()->m_turret.setFeederMotor(0)),
                    new InstantCommand(()->m_hopper.setHopperMotor(0))
                    ));

            // (X) Button -> runs the full shoot command
            new JoystickButton(m_operator,1).whileTrue(FullShoot);

            // new JoystickButton(m_operator,10).onTrue(new InstantCommand(()-> m_intake.setRotationTarget(180)));
            // new JoystickButton(m_operator,9).onTrue(new InstantCommand(()-> m_intake.setRotationTarget(90)));
    
            // (A) Button -> runs the feeder motor and the hopper motor
            new JoystickButton(m_operator, 2)
                .onTrue( new SequentialCommandGroup(
                    new InstantCommand(()->m_intake.setIntakeMotor(0.3)),
                    new InstantCommand(()->m_hopper.setHopperMotor(0.5)
                    )))
                .onFalse(new SequentialCommandGroup(
                    new InstantCommand(()->m_turret.setFeederMotor(0)),
                    new InstantCommand(()->m_hopper.setHopperMotor(0))
                    ));

            new JoystickButton(m_operator,7)
                .onTrue(new InstantCommand(()-> m_intake.setIntakeMotor(-0.3)))
                .onFalse(new InstantCommand(()-> m_intake.setIntakeMotor(0)));

            // (RT) Right Trigger -> While holding it changes the target rotation angle based off of the left stick x
            new JoystickButton(m_operator, 8)
                .whileTrue(
                    new RunCommand(() ->
                        m_turret.setRotationMotor(m_operator.getLeftX()/5)
                    ))
                .onFalse(new InstantCommand(()->m_turret.setRotationMotor(0)));

            // DPad Up -> increase power offset
            new POVButton(m_operator, 0).onTrue(
                new SequentialCommandGroup(
                    new InstantCommand(() -> ShooterPowerCommand.adjustPowerOffset(0.05))
                    // new InstantCommand(() -> FullShoot.adjustPowerOffset(0.05))
                )
            );
            // DPad Down -> decrease power offset
            new POVButton(m_operator, 180).onTrue(
                new SequentialCommandGroup(
                    new InstantCommand(() -> ShooterPowerCommand.adjustPowerOffset(-0.05))
                    // new InstantCommand(() -> FullShoot.adjustPowerOffset(-0.05))
                )
            );




        // puts the power offset and angle offset on sd
        SmartDashboard.putNumber("Shooter Power Offset", ShooterPowerCommand.getPowerOffset());
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }   

}

