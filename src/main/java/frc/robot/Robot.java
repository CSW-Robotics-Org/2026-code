// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.HootAutoReplay;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    private String lastAutoName = "";
    private boolean wasAuto = false;
    private final Pose2d OFF_FIELD = new Pose2d(-100, -100, new Rotation2d());

    private boolean cachedShiftState = false;
    private boolean lastAuto = false;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
    }

    // calculates our shift

    private boolean computeShiftState() {

        String gameData = DriverStation.getGameSpecificMessage();
        Optional<Alliance> allianceOpt = DriverStation.getAlliance();

        if (gameData == null || gameData.isEmpty() || allianceOpt.isEmpty()) {
            return false;
        }

        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        char data = gameData.charAt(0);

        boolean redInactiveFirst;
        if (data == 'R') redInactiveFirst = true;
        else if (data == 'B') redInactiveFirst = false;
        else return false;

        Alliance alliance = allianceOpt.get();

        boolean shift1Active = (alliance == Alliance.Red)
                ? !redInactiveFirst
                : redInactiveFirst;

        double t = Timer.getMatchTime();

        boolean endgame = t <= 30;
        boolean transition = t > 130 && t <= 140;

        int shift;
        if (t > 105) shift = 1;
        else if (t > 80) shift = 2;
        else if (t > 55) shift = 3;
        else shift = 4;

        boolean isOurShift =
                switch (shift) {
                    case 1 -> shift1Active;
                    case 2 -> !shift1Active;
                    case 3 -> shift1Active;
                    case 4 -> !shift1Active;
                    default -> false;
                };

        boolean preShift = false;
        if ((!isOurShift && t <= 108 && t >= 105) || (!isOurShift && t <= 83 && t >= 80) || (!isOurShift && t <= 58 && t >= 55) || (!isOurShift && t <= 33 && t >= 30)){
            preShift = true;
        }

        return isOurShift || endgame || transition || preShift;
    }

    private boolean isRedAlliance() {
        return DriverStation.getAlliance()
                .orElse(Alliance.Blue) == Alliance.Red;
    }

    @Override
    public void disabledInit() {
        m_robotContainer.limelight.initVisionCalibration();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {

        double matchTime = Timer.getMatchTime();

        // Force game data at ~21 seconds into match
        if (matchTime <= 130) {
            DriverStationSim.setGameSpecificMessage("R");
        }

    }
}
