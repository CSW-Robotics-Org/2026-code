package frc.robot.subsystems;

import java.util.Optional;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Dashboard extends SubsystemBase {

    private final RobotContainer m_robotContainer;
    private String lastAutoName = "";
    private boolean wasAuto = false;
    private final Pose2d OFF_FIELD = new Pose2d(-100, -100, new Rotation2d());

    private boolean cachedShiftState = false;
    private boolean lastAuto = false;

    public Dashboard (RobotContainer rc){
        m_robotContainer = rc;

        this.updateSwerve();
    }

    public void updatePowerOffset(){
        // puts the shooter offset on sd
        SmartDashboard.putNumber("Shooter Power Offset", m_robotContainer.ShooterPowerCommand.getPowerOffset());
    }

    public void updateBatteryVoltage(){
        // puts battery voltage
        double voltage = RobotController.getBatteryVoltage();
        SmartDashboard.putNumber("Battery Voltage", voltage);
    }

    public void updateRobotPos(){
        Pose2d robotPose = m_robotContainer.drivetrain.getState().Pose;
        // DO NOT flip robot pose for Field2d
        m_robotContainer.field.setRobotPose(robotPose);
        SmartDashboard.putData(m_robotContainer.field);
    }

    public void updateMatchTime(){
        // puts the match timer on screen
        double timeLeft = Timer.getMatchTime();
        int timeLeftRounded = (int) timeLeft;
        SmartDashboard.putNumber("Match Time", timeLeftRounded);
    }

    public void updateFMSData(){
        // puts the fms connected boolean on screen
        SmartDashboard.putBoolean("FMS Connected", DriverStation.isFMSAttached());
        // puts the ds connected boolean on screen
        SmartDashboard.putBoolean("DS Connected", DriverStation.isDSAttached());

        // puts the enabled state on sd
        String state;

        if (DriverStation.isDisabled()) {
            state = "DISABLED";
        } else if (DriverStation.isAutonomous()) {
            state = "AUTO";
        } else if (DriverStation.isTeleop()) {
            state = "TELEOP";
        } else if (DriverStation.isTest()) {
            state = "TEST";
        } else {
            state = "UNKNOWN";
        }

        SmartDashboard.putString("Robot State", state);
    }

    public void updateAutoPaths(){
        // puts auto paths on screen
        String selectedAuto = m_robotContainer.autoChooser.getSelected().getName();

        if (selectedAuto != null && !selectedAuto.equals(lastAutoName)) {
        lastAutoName = selectedAuto;

        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;;

        // clear old paths
        for (int i = 0; i < 10; i++) {
            m_robotContainer.field.getObject("Path " + i)
                    .setPoses(new java.util.ArrayList<>());
        }

        try {
            var paths = PathPlannerAuto.getPathGroupFromAutoFile(selectedAuto);

            java.util.List<Pose2d> allPoses = new java.util.ArrayList<>();

            for (int i = 0; i < paths.size(); i++) {

                var rawPoses = paths.get(i).getPathPoses();

                java.util.List<Pose2d> posesToDisplay = new java.util.ArrayList<>();

                for (Pose2d pose : rawPoses) {
                    posesToDisplay.add(
                            isRed ? FlippingUtil.flipFieldPose(pose) : pose
                    );
                }

                m_robotContainer.field.getObject("Path " + i)
                        .setPoses(posesToDisplay);

                allPoses.addAll(posesToDisplay);
            }

            if (!allPoses.isEmpty()) {

                Pose2d start = allPoses.get(0);
                Pose2d end = allPoses.get(allPoses.size() - 1);

                m_robotContainer.field.getObject("Auto Start")
                        .setPose(start);

                m_robotContainer.field.getObject("Auto End")
                        .setPose(end);
            }

            } catch (Exception e) {
                System.out.println("Failed to load auto: " + selectedAuto);
            }
        }

        // for clearing at end of auto

        // clears auto off field widget at end of auto
        boolean isAuto = DriverStation.isAutonomous();

        // Detect auto ending
        if (wasAuto && !isAuto) {
            // Clear all paths
            for (int i = 0; i < 10; i++) {
                m_robotContainer.field.getObject("Path " + i)
                    .setPoses(new java.util.ArrayList<>());
            }

            m_robotContainer.field.getObject("Auto Start")
                .setPose(OFF_FIELD);

            m_robotContainer.field.getObject("Auto End")
                .setPose(OFF_FIELD);
        }

        wasAuto = isAuto;

    }

    public void updateShift(){
        boolean isAutoNow = DriverStation.isAutonomousEnabled();

        // force update on auto start/exit OR game data arrival
        boolean shouldUpdate =
                cachedShiftState == false || isAutoNow != lastAuto ||
                DriverStation.getGameSpecificMessage().length() > 0;

        if (shouldUpdate) {
            cachedShiftState = computeShiftState();
        }

        lastAuto = isAutoNow;

        SmartDashboard.putBoolean("Shift", cachedShiftState || isAutoNow);
    }

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

    public void updateSwerve(){
        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {

                builder.setSmartDashboardType("SwerveDrive");

                // Front Left
                builder.addDoubleProperty(
                    "Front Left Angle",
                    () -> m_robotContainer.drivetrain.getModules()[0]
                            .getPosition(false).angle.getRadians(),
                    null
                );

                builder.addDoubleProperty(
                    "Front Left Velocity",
                    () -> m_robotContainer.drivetrain.getModules()[0]
                            .getCurrentState().speedMetersPerSecond,
                    null
                );

                // Front Right
                builder.addDoubleProperty(
                    "Front Right Angle",
                    () -> m_robotContainer.drivetrain.getModules()[1]
                            .getPosition(false).angle.getRadians(),
                    null
                );

                builder.addDoubleProperty(
                    "Front Right Velocity",
                    () -> m_robotContainer.drivetrain.getModules()[1]
                            .getCurrentState().speedMetersPerSecond,
                    null
                );

                // Back Left
                builder.addDoubleProperty(
                    "Back Left Angle",
                    () -> m_robotContainer.drivetrain.getModules()[2]
                            .getPosition(false).angle.getRadians(),
                    null
                );

                builder.addDoubleProperty(
                    "Back Left Velocity",
                    () -> m_robotContainer.drivetrain.getModules()[2]
                            .getCurrentState().speedMetersPerSecond,
                    null
                );

                // Back Right
                builder.addDoubleProperty(
                    "Back Right Angle",
                    () -> m_robotContainer.drivetrain.getModules()[3]
                            .getPosition(false).angle.getRadians(),
                    null
                );

                builder.addDoubleProperty(
                    "Back Right Velocity",
                    () -> m_robotContainer.drivetrain.getModules()[3]
                            .getCurrentState().speedMetersPerSecond,
                    null
                );

                // Robot heading
                builder.addDoubleProperty(
                    "Robot Angle",
                    () -> m_robotContainer.drivetrain.getState().Pose.getRotation().getRadians(),
                    null
                );
            }
        });
    }


    public void periodic(){
        this.updateAutoPaths();
        this.updateBatteryVoltage();
        this.updateFMSData();
        this.updateMatchTime();
        this.updatePowerOffset();
        this.updateRobotPos();
        this.updateShift();
    }


    
}