/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.containers;

import static frc.robot.autonomous.ExtendedTrajectoryUtilities.tryGetDeployedTrajectory;
import static frc.robot.autonomous.GenericAutonUtilities.createDefaultControllerBuilder;
import static frc.robot.utility.ExtendedMath.withHardDeadzone;
import static frc.robot.utility.ExtendedMath.withContinuousDeadzone;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.pshoot.Autonomous_PreciseShootingCommand;
import frc.robot.autonomous.Autonomous_ForceIndexBallsCommand;
import frc.robot.autonomous.GenericAutonUtilities;
import frc.robot.autonomous.Autonomous_IndexBallsCommand;
import frc.robot.autonomous.Autonomous_SingleSensorIndexBallsCommand;
import frc.robot.autonomous.VisionDistanceCalculator;
import frc.robot.autonomous.pshoot.VisionPreciseShootingOI;
import frc.robot.components.hardware.CameraVisionComponent;
import frc.robot.components.hardware.LimelightVisionComponent;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.factory.HardwareIntakeFactory;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.factory.HardwareArmFactory;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.factory.HardwareClimberFactory;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.factory.HardwareIndexerFactory;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.factory.HardwareShooterFactory;
import frc.robot.subsystems.swerve.SwerveOI;
import frc.robot.subsystems.swerve.TrackLoadingCommand;
import frc.robot.subsystems.swerve.kinematic.command.KinematicSwerve_RampedDriveCommand;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.OdometricSwerveDashboardUtility;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_AdvancedFollowTrajectoryCommand;
import frc.robot.subsystems.swerve.odometric.factory.EntropySwerveFactory;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.factory.HardwareTurretFactory;
import frc.robot.subsystems.vision.CameraVisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utility.OutputRamper;

/**
 * Add your docs here.
 */
public class DriverPracticeRobotContainer implements RobotContainer {

    private static final boolean coastingEnabled = false;

    private SendableChooser<Command> autonomousChooser;

    private Joystick driveStick = new Joystick(0), controlStick = new Joystick(1);
    private JoystickButton 
    intakeInButton = new JoystickButton(controlStick, 5),
    intakeOutButton = new JoystickButton(controlStick, 3),
    indexerInButton  = new JoystickButton(controlStick,6),
    indexerOutButton = new JoystickButton(controlStick, 4),
    mainIntakeButton = new JoystickButton(controlStick, 1),
    shootButton = new JoystickButton(controlStick,2),
    climberUpButton = new JoystickButton(controlStick, 9),
    climberDownButton = new JoystickButton(controlStick, 11),
    resetGyroButton = new JoystickButton(driveStick, 5),
    backupIndexerButton = new JoystickButton(controlStick, 8),
    alignToLoadButton = new JoystickButton(driveStick, 3);
    
    private Intake intake = new HardwareIntakeFactory().makeIntake();
    private Arm arm = new HardwareArmFactory().makeArm();
    private OdometricSwerve swerve = new EntropySwerveFactory().makeSwerve();
    private Indexer indexer = new HardwareIndexerFactory().makeIndexer();
    private Shooter shooter = new HardwareShooterFactory().makeShooter();
    private Turret turret = new HardwareTurretFactory().makeTurret();
    private VisionSubsystem limelight = new VisionSubsystem(new LimelightVisionComponent());

    private Climber climber = new HardwareClimberFactory().makeClimber();

    private CameraVisionSubsystem cameraVision = new CameraVisionSubsystem(new CameraVisionComponent());
    private TrackLoadingCommand trackLoadingCommand = new TrackLoadingCommand(swerve, cameraVision);
    
    private boolean useFancyIntakeCommand = true;
    private VisionDistanceCalculator visionDistanceCalculator;
    private VisionPreciseShootingOI visionPreciseShootingOI;

    private double xSensitivity = 4, ySensitivity = 4, zSensitivity = 4, xDeadzone = 0.2, yDeadzone = 0.2,
            zDeadzone = 0.3;
    private boolean useContinuousDriving = false, useDriverTab = true;

    private double turretRadianOffset = 0.0;
    private Translation2d 
    visionTargetTranslation = new Translation2d(15.98, -5.85), 
    innerTargetTranslation = new Translation2d(15.98 + 0.74,-5.85);

    public DriverPracticeRobotContainer() {

        configureBasicOverrides();

        // alternateConfigureMainintakeButton();
        configureMainIntakeButton();

        configureShooter();

        resetGyroButton.whenPressed(() -> swerve.resetPose());
        swerve.resetPose(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)));

        configureSwerve();

        configureTurret();

        configureClimber();

        configureSmartDashboardControls();

        configureAutonomous();

        backupIndexerButton.whileHeld(new Autonomous_IndexBallsCommand(indexer, intake, 1, 0));
    }

    private double getTurretRadianOffset() {
        return turretRadianOffset;
    }

    private void setTurretRadianOffset(double turretRadianOffset) {
        this.turretRadianOffset = turretRadianOffset;
    }

    private void configureAutonomous() {
        autonomousChooser = new SendableChooser<>();

        autonomousChooser.addOption(
            "Shoot and Cross The Line", 
            createShootAndCrossTheLineCommand());

        autonomousChooser.addOption(
            "Away From Center, Move Forward and Shoot",
            createAwayFromCenterMoveForwardAndShootCommand());

        autonomousChooser.addOption(
            "Away From Center, Move Backward and Shoot",
            createAwayFromCenterMoveBackwardAndShootCommand());

        autonomousChooser.addOption(
            "Citrus Compatible Primary", 
            createCitrusCompatibleCommand());
        autonomousChooser.addOption(
            "Citrus Compatible Secondary",
            createCitrusCompatibleSecondary());
        autonomousChooser.addOption(
            "Trench Citrus Compatible Primary", 
            createTrenchCitrusCompatiblePartACommand());

        autonomousChooser.addOption(
            "Trench Citrus Compatible Second", 
            createTrenchCitrusCompatibleBCommand());
        
        /* Example Path Added to AutonomousChooser */
        autonomousChooser.addOption(
            "Example Autonomous", 
            createExampleAutonomousCommand());

        SmartDashboard.putData("Selected Auto", autonomousChooser);
    }

    private SequentialCommandGroup createTrenchCitrusCompatibleBCommand() {
        return createTrenchCitrusPart1Command()
            .andThen(
                new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                    swerve,
                    createDefaultControllerBuilder()
                    .withTrajectory(tryGetDeployedTrajectory("TrenchCitrusCompatiblePart2B"))
                    .withEndRotation(new Rotation2d(7 * Math.PI / 6))
                    .buildController()))
            .andThen(() -> arm.setAngle(Math.PI / 2), arm)
            .andThen(new Autonomous_IndexBallsCommand(indexer, intake, 1, 0.9).withTimeout(5))
            .andThen(() -> arm.setAngle(0), arm)
            .andThen(
                new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                    swerve,
                    createDefaultControllerBuilder()
                    .withTrajectory(tryGetDeployedTrajectory("TrenchCitrusCompatiblePart3B"))
                    .withEndRotation(new Rotation2d(Math.PI))
                    .buildController()))
                .andThen(new Autonomous_PreciseShootingCommand(shooter, indexer, visionPreciseShootingOI)
                        .withTimeout(4));
    }

    private SequentialCommandGroup createTrenchCitrusCompatiblePartACommand() {
        return createTrenchCitrusPart1Command()
        .andThen(new OdometricSwerve_AdvancedFollowTrajectoryCommand(
            swerve,
            createDefaultControllerBuilder()
            .withEndRotation(new Rotation2d(Math.PI))
            .withTrajectory(tryGetDeployedTrajectory("TrenchCitrusCompatiblePart2A"))
            .withMaxVelocity(4.0)
            .with_kW(6)
            .buildController()))
        // .andThen(this::aimAtInnerPort, limelight, turret, swerve)

        .andThen(() -> swerve.moveFieldCentric(0, 0, 0), swerve)
        .andThen(new Autonomous_PreciseShootingCommand(shooter, indexer, visionPreciseShootingOI).withTimeout(6));
    }

    private CommandGroupBase createTrenchCitrusPart1Command() {
        return new InstantCommand(() -> swerve.resetPose(new Pose2d(13, -7.5, new Rotation2d(Math.PI))), swerve)
            // .andThen(this::aimAtInnerPort, turret, limelight, swerve)
            .andThen(new Autonomous_PreciseShootingCommand(shooter, indexer, visionPreciseShootingOI).withTimeout(3))
            .andThen(
                
                (new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                    swerve,
                    createDefaultControllerBuilder()
                    .withEndRotation(new Rotation2d(Math.PI))
                    .with_kW(6)
                    .withRotationsEnabled(true)
                    .withTrajectory(tryGetDeployedTrajectory("TrenchCitrusCompatiblePart1"))
                    .withMaxVelocity(2)
                    .buildController()))
                .deadlineWith(
                    new Autonomous_ForceIndexBallsCommand(indexer, intake, arm, 1, 0.9, Math.PI/2.1)
                    ));
    }

    private SequentialCommandGroup createCitrusCompatibleSecondary(){
        return 
        new InstantCommand(() -> swerve.resetPose(new Pose2d(12.565, -4.875, new Rotation2d(Math.PI))), swerve)
        .andThen(() -> turretRadianOffset = 0.0)
        .andThen(new Autonomous_PreciseShootingCommand(shooter, indexer, visionPreciseShootingOI).withTimeout(3))
        .andThen(new OdometricSwerve_AdvancedFollowTrajectoryCommand(
            swerve, 
            createDefaultControllerBuilder()
            .withEndRotation(new Rotation2d(-1.72, -0.749))
            .with_kW(6)
            .withTrajectory(tryGetDeployedTrajectory("CitrusCompatiblePart1"))
            .withMaxVelocity(4.0)
            .buildController()))
        .andThen(
            new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                swerve, 
                createDefaultControllerBuilder()
                .withEndRotation(new Rotation2d(-1.72,-0.749))
                .with_kW(6)
                .withMaxVelocity(1)
                .withTrajectory(tryGetDeployedTrajectory("CitrusCompatiblePart2")).buildController())
            .deadlineWith(new Autonomous_ForceIndexBallsCommand(indexer, intake, arm, 1, 0.9, Math.PI / 2.1)))
        .andThen(new OdometricSwerve_AdvancedFollowTrajectoryCommand(
            swerve, 
            createDefaultControllerBuilder()
            .withEndRotation(new Rotation2d(Math.PI))
            .withMaxVelocity(4.0)
            .with_kW(6)
            .withTrajectory(tryGetDeployedTrajectory("CitrusCompatiblePart3"))
            .buildController()))
        .andThen(() -> turretRadianOffset = 0.0)
        .andThen(new Autonomous_PreciseShootingCommand(shooter, indexer, visionPreciseShootingOI));
    }
    private SequentialCommandGroup createCitrusCompatibleCommand() {
        return new InstantCommand(() -> turretRadianOffset = Units.degreesToRadians(3.5))
            .andThen(new InstantCommand(() -> swerve.resetPose(new Pose2d(12.565, -4.875, new Rotation2d(Math.PI))), swerve))
            .andThen(new Autonomous_PreciseShootingCommand(shooter, indexer, -3390, -2520, 1.47, 500).withTimeout(3))
            .andThen(new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                swerve,
                createDefaultControllerBuilder()
                .withEndRotation(new Rotation2d(Math.PI + Math.PI * 1.2 / 7))
                .withTrajectory(tryGetDeployedTrajectory("CitrusCompatiblePart1"))
                .withMaxVelocity(4.0)
                .buildController()))
            .andThen(() -> swerve.moveFieldCentric(0, 0, 0), swerve).andThen(() -> arm.setAngle(Math.PI / 2.1), arm)
            .andThen(
                new Autonomous_IndexBallsCommand(indexer, intake, 1.0, 0.9)
                .raceWith(
                    new RunCommand(() -> swerve.moveFieldCentric(0.1, -0.25 * 2, 0), swerve)
                    .withTimeout(2)
                    .andThen(
                        new RunCommand(() -> swerve.moveFieldCentric(0.1 * 2.2, -0.25 * 2.2, 0), swerve)
                        .withTimeout(0.7)
                        .andThen(
                            new RunCommand(() -> swerve.moveFieldCentric(0, 0, 1))
                            .withTimeout(1))
                        .andThen(
                            new RunCommand(() -> swerve.moveFieldCentric(-0.1, 0, 0))
                            .withTimeout(1))
                        .andThen(new WaitCommand(1))
                        .andThen(
                            new RunCommand(() -> swerve.moveFieldCentric(0, 0, -2))
                            .withTimeout(1)))))
            .andThen(() -> turretRadianOffset = Units.degreesToRadians(3.0))
            .andThen(() -> arm.setAngle(0), arm)
            .andThen(() -> intake.setSpeed(0), intake)
            .andThen(() -> indexer.setSpeed(0), indexer)
            .andThen(
                new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                    swerve,
                    createDefaultControllerBuilder()
                    .withEndRotation(new Rotation2d(Math.PI))
                    .withInitialAllowableTranslationError(0.5)
                    .withFinalAllowableTranslationError(0.02)
                    .withTrajectory(tryGetDeployedTrajectory("CitrusCompatiblePart3"))
                    .withMaxVelocity(4.0)
                    .buildController())
                .withTimeout(0))
            .andThen(() -> swerve.moveFieldCentric(0, 0, 0), swerve)
            .andThen(
                new Autonomous_PreciseShootingCommand(shooter, indexer, visionPreciseShootingOI)
                .withTimeout(4));
    }

    private SequentialCommandGroup createAwayFromCenterMoveBackwardAndShootCommand() {
        return new InstantCommand(() -> swerve.resetPose(new Pose2d(13, -2.5, new Rotation2d(Math.PI))), swerve)
            .andThen(
                new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                    swerve,
                    createDefaultControllerBuilder()
                    .withTrajectory(tryGetDeployedTrajectory("AwayFromCenterBackward"))
                    .withEndRotation(new Rotation2d(Math.PI)).buildController()))
            .andThen(() -> swerve.moveFieldCentric(0, 0, 0))
            .andThen(
                new Autonomous_PreciseShootingCommand(shooter, indexer, visionPreciseShootingOI)
                .withTimeout(2));
    }

    private SequentialCommandGroup createAwayFromCenterMoveForwardAndShootCommand() {
        return new InstantCommand(() -> swerve.resetPose(new Pose2d(13, -2.5, new Rotation2d(Math.PI))), swerve)
            .andThen(
                new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                    swerve,
                    createDefaultControllerBuilder()
                    .withTrajectory(tryGetDeployedTrajectory("AwayFromCenterForward"))
                    .withEndRotation(new Rotation2d(Math.PI))
                    .buildController()))
            .andThen(() -> swerve.moveFieldCentric(0, 0, 0))
            .andThen(
                new Autonomous_PreciseShootingCommand(shooter, indexer, visionPreciseShootingOI)
                .withTimeout(2));
    }

    private SequentialCommandGroup createShootAndCrossTheLineCommand() {
        return new InstantCommand(() -> swerve.resetPose(new Pose2d(13, -5.75, new Rotation2d(Math.PI))), swerve)
            .andThen(
                new Autonomous_PreciseShootingCommand(shooter, indexer, -3390, -2520, 1.47, 500)
                .withTimeout(4))
            .andThen(
                new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                    swerve,
                    createDefaultControllerBuilder()
                    .withEndRotation(new Rotation2d(Math.PI))
                    .withTrajectory(tryGetDeployedTrajectory("CrossTheLine"))
                    .buildController()));
    }

    /* Example Autonomous Command */
    private CommandBase createExampleAutonomousCommand(){
        return new InstantCommand(() -> swerve.resetPose(new Translation2d(0.762, 12.714)),swerve)
        .andThen(new OdometricSwerve_AdvancedFollowTrajectoryCommand(
            swerve, 
            createDefaultControllerBuilder()
            .withEndRotation(new Rotation2d(Math.PI))
            .withTrajectory(tryGetDeployedTrajectory("ExampleTrajectory"))
            .withMaxVelocity(1.0)
            .buildController()));

    }
    @Override
    public void teleopInit() {
        turretRadianOffset = 0;
    }
    private void configureShooter() {
        visionDistanceCalculator = GenericAutonUtilities.makeEntropyVisionDistanceCalculator(limelight);
        visionPreciseShootingOI = new VisionPreciseShootingOI(visionDistanceCalculator);
        //var shootCommand = new Autonomous_PreciseShootingCommand(shooter, indexer, -500,-500,1,500);
        var shootCommand = new Autonomous_PreciseShootingCommand(shooter, indexer, visionPreciseShootingOI);
        shootButton.whileHeld(shootCommand);
    }

    private void configureSmartDashboardControls() {
        SmartDashboard.putData("Control Preferences", new Sendable() {

            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addBooleanProperty("Use Sensors When Indexing", () -> useFancyIntakeCommand, value -> useFancyIntakeCommand = value);
            }
        });

        SmartDashboard.putData("Vision Distance Calculator", visionDistanceCalculator);

        SmartDashboard.putData("Indexer", indexer);

        SmartDashboard.putData(new Sendable(){
        
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Turret Radian Offset", () -> getTurretRadianOffset(), value -> setTurretRadianOffset(value));
            }
        });

    }

    private void configureClimber() {
        climberUpButton.whenPressed(() -> climber.setSpeed(1), climber).whenReleased(() -> climber.setSpeed(0),
                climber);

        climberDownButton.whenPressed(() -> climber.setSpeed(-1), climber).whenReleased(() -> climber.setSpeed(0),
                climber);
    }

    private void configureTurret() {
        turret.setDefaultCommand(
            new PIDCommand(
                new PIDController(-3, 0, 0), 
                limelight::getHorizontalOffset,
                this::getTurretRadianOffset, 
                turret::setTurretOutput, 
                turret,limelight));
    }

    private void configureSwerve() {
        var driverTab = Shuffleboard.getTab("Driver Controls");
        var xSettings = driverTab.getLayout("X Settings",BuiltInLayouts.kList);
        var ySettings = driverTab.getLayout("Y Settings",BuiltInLayouts.kList);
        var zSettings = driverTab.getLayout("Z Settings",BuiltInLayouts.kList);

        var xSensitivityEntry = xSettings.addPersistent("Sensitivity", 4).getEntry();
        var xDeadzoneEntry = xSettings.addPersistent("Deadzone", 0.2).getEntry();

        var ySensitivityEntry = ySettings.addPersistent("Sensitivity", 4).getEntry();
        var yDeadzoneEntry = ySettings.addPersistent("Deadzone", 0.2).getEntry();

        var zSensitivityEntry = zSettings.addPersistent("Sensitivity", 4).getEntry();
        var zDeadzoneEntry = zSettings.addPersistent("Deadzone", 0.3).getEntry();

        var continuousDrivingEntry = driverTab.addPersistent("Use Continuous Driving", false).getEntry();


        swerve.setDefaultCommand(new RunCommand(() -> {
            double forwardSpeed, leftwardSpeed, counterClockwardSpeed;
            if(continuousDrivingEntry.getBoolean(false)){
                forwardSpeed = withContinuousDeadzone(-driveStick.getY(), yDeadzoneEntry.getDouble(0.2)) * ySensitivityEntry.getDouble(4);
                leftwardSpeed = withContinuousDeadzone(-driveStick.getX(), xDeadzoneEntry.getDouble(0.2)) * xSensitivityEntry.getDouble(4);
                counterClockwardSpeed = withContinuousDeadzone(-driveStick.getZ(), zDeadzoneEntry.getDouble(0.3)) * zSensitivityEntry.getDouble(4);
            }else{
                forwardSpeed = withHardDeadzone(-driveStick.getY(), yDeadzoneEntry.getDouble(0.2)) * ySensitivityEntry.getDouble(4);
                leftwardSpeed = withHardDeadzone(-driveStick.getX(), xDeadzoneEntry.getDouble(0.2)) * xSensitivityEntry.getDouble(4);
                counterClockwardSpeed = withHardDeadzone(-driveStick.getZ(), zDeadzoneEntry.getDouble(0.3)) * zSensitivityEntry.getDouble(4);
            }
            swerve.moveFieldCentric(forwardSpeed, leftwardSpeed, counterClockwardSpeed);
        },swerve));

        
        
        if(useDriverTab){
            Shuffleboard.selectTab("Driver Controls");
        }

        alignToLoadButton.whenHeld(trackLoadingCommand);
    }
    private CommandBase createHardDeadzoneSwerveCommand(){
        return new RunCommand(() -> {
            var forwardSpeed = withHardDeadzone(-driveStick.getY(), yDeadzone) * ySensitivity;
            var leftwardSpeed = withHardDeadzone(-driveStick.getX(), xDeadzone) * xSensitivity;
            var counterClockwardSpeed = withHardDeadzone(-driveStick.getZ(), zDeadzone) * zSensitivity;
            
            swerve.moveFieldCentric(forwardSpeed, leftwardSpeed, counterClockwardSpeed);
            
        }, swerve);
    }
    private CommandBase createContinuousDeadzoneSwerveCommand(){
        return new RunCommand(() -> {
            var forwardSpeed = withContinuousDeadzone(-driveStick.getY(), 1/(1-yDeadzone), yDeadzone) * ySensitivity;
            var leftwardSpeed = withContinuousDeadzone(-driveStick.getX(), 1/(1-xDeadzone), xDeadzone) * xSensitivity;
            var counterClockwardSpeed = withContinuousDeadzone(-driveStick.getZ(), 1/(1-zDeadzone), zDeadzone) * zSensitivity;
            
            swerve.moveFieldCentric(forwardSpeed, leftwardSpeed, counterClockwardSpeed);
        }, swerve);
    }
    private CommandBase createRampedContinuousDeadzoneSwerveCommand(){
        double ramperPeriod = 0.2;
        double maxAbsoluteRampingPerSecond = 2;
        return new KinematicSwerve_RampedDriveCommand(
            swerve, 
            new OutputRamper(0, 0.2, maxAbsoluteRampingPerSecond * ramperPeriod, 0), 
            new OutputRamper(0, 0.2, maxAbsoluteRampingPerSecond * ramperPeriod, 0), 
            new OutputRamper(0, 0.2, maxAbsoluteRampingPerSecond * ramperPeriod, 0), 
            new SwerveOI(){
            
                @Override
                public double getZ() {
                    return withContinuousDeadzone(-driveStick.getZ(), 1/(1-zDeadzone), zDeadzone) * zSensitivity;
                }
            
                @Override
                public double getY() {
                    return withContinuousDeadzone(-driveStick.getX(), 1/(1-xDeadzone), xDeadzone) * xSensitivity;
                }
            
                @Override
                public double getX() {
                    return withContinuousDeadzone(-driveStick.getY(), 1/(1-yDeadzone), yDeadzone) * ySensitivity;
                }
            });
    }

    private void configureMainIntakeButton() {
        mainIntakeButton
        .whileHeld(
            new ConditionalCommand(
                new Autonomous_IndexBallsCommand(indexer, intake, 1, 0.9)
                .alongWith(
                    new FunctionalCommand(
                        () -> arm.setAngle(Math.PI / 2), 
                        () -> {}, 
                        interrupted -> arm.setAngle(0), 
                        () -> false, 
                        arm)), 
                    new FunctionalCommand(
                        () -> {
                            indexer.setSpeed(1);
                            intake.setSpeed(-1);
                            arm.setAngle(Math.PI / 2);
                        }, 
                        () -> {}, 
                        interrupted -> {
                            indexer.setSpeed(0);
                            intake.setSpeed(0);
                            arm.setAngle(0);
                        }, 
                        () -> false, 
                        indexer, arm, intake), 
                () -> useFancyIntakeCommand));
    }
    private void alternateConfigureMainintakeButton(){
        mainIntakeButton
        .whileHeld(new Autonomous_SingleSensorIndexBallsCommand(arm, intake, indexer, 0.0, 0.0, 1.0));
        //.whileHeld(new Autonomous_FastIndexBallsCommand(0.001, intake, arm, indexer, 0.0, 0.0, 1.0));
    }

    private void configureBasicOverrides() {
        intakeInButton
        .whenPressed(() -> {
            intake.setSpeed(1);
            arm.setAngle(Math.PI);
        }, intake, arm)
        .whenReleased(() -> {
            intake.setSpeed(0);
            arm.setAngle(0);
        }, intake, arm);

        intakeOutButton
        .whenPressed(() -> {
            intake.setSpeed(-1);
            arm.setAngle(Math.PI);
        }, intake, arm)
        .whenReleased(() -> {
            intake.setSpeed(0);
            arm.setAngle(0);
        }, intake, arm);

        indexerInButton
        .whenPressed(() -> indexer.setSpeed(1), indexer)
        .whenReleased(() -> indexer.setSpeed(0),indexer);

        indexerOutButton
        .whenPressed(() -> indexer.setSpeed(-1), indexer)
        .whenReleased(() -> indexer.setSpeed(0), indexer);
    }

    @Override
    public Command getAutonomousCommand() {
        return autonomousChooser.getSelected();
    }
    private void aimAtInnerPort(){
        turretRadianOffset = visionDistanceCalculator.getDesiredTurretOffset(swerve.getCurrentPose().getTranslation(), visionTargetTranslation, innerTargetTranslation);
    }
}
