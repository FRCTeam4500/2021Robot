package frc.robot.containers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.ControlConstants;
import frc.robot.ControlOI;
import frc.robot.autonomous.GenericAutonUtilities;
import frc.robot.autonomous.VisionDistanceCalculator;
import frc.robot.autonomous.autonCommands.AwayFromCenterMoveBackwardAndShootCommand;
import frc.robot.autonomous.autonCommands.AwayFromCenterMoveForwardAndShootCommand;
import frc.robot.autonomous.autonCommands.CitrusCompatibleCommand;
import frc.robot.autonomous.autonCommands.ShootAndCrossTheLineCommand;
import frc.robot.autonomous.pshoot.Autonomous_PreciseShootingCommand;
import frc.robot.autonomous.pshoot.VisionPreciseShootingOI;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmOI;
import frc.robot.subsystems.arm.commands.TeleopArmCommand;
import frc.robot.subsystems.arm.factory.HardwareArmFactory;
import frc.robot.subsystems.climber.ClimberOI;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerOI;
import frc.robot.subsystems.indexer.commands.IndexerRunCommand;
import frc.robot.subsystems.indexer.commands.TeleopIndexerCommand;
import frc.robot.subsystems.indexer.factory.HardwareIndexerFactory;
import frc.robot.subsystems.intake.command.IntakeArmRunCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterOI;
import frc.robot.subsystems.shooter.command.PreciseShootingCommand;
import frc.robot.subsystems.shooter.command.ShootStraightCommand;
import frc.robot.subsystems.shooter.factory.HardwareShooterFactory;
import frc.robot.subsystems.swerve.SwerveOI;
import frc.robot.subsystems.swerve.commands.HardDeadzoneSwerveCommand;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.OdometricSwerveDashboardUtility;
import frc.robot.subsystems.swerve.odometric.factory.EntropySwerveFactory;
import frc.robot.subsystems.intake.factory.HardwareIntakeFactory;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretOI;
import frc.robot.subsystems.turret.factory.HardwareTurretFactory;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.components.hardware.LimelightVisionComponent;
import frc.robot.subsystems.turret.command.TurretAutoCommand;
import frc.robot.utility.ValueGetterCommand;

/**
 * GRC robot container
 * When designing robot containers, we are trying to keep as much code as possible out of the container class itself.
 * we don't want the containers to become cluttered messes.
 * See the flowchart for reference on how to design commands and subsystems.
 * Try to avoid inlining anonymous command here, create an actual class for the command if its more than like 3 lines.
 * You can directly control subsystems from here, but try to make their normal behavior controlled by their default command, its called default for a reason
 * Also if controlling a subsystem requires more than 1 line of code in a lambda, just make a command, its worth it to save on verbosity
 *
 * */

public class GRCRobotContainer implements RobotContainer, SwerveOI, ClimberOI, ArmOI, IndexerOI, ShooterOI, TurretOI, ControlOI {


    private ShuffleboardTab driverTab;
    private Joystick
            driveStick = new Joystick(0),
            controlStick = new Joystick(1);


    private JoystickButton
            resetGyroButton,
            climberUpButton,
            climberDownButton,
            intakeTrigger,
            shooterTrigger,
            intakeInButton,
            intakeOutButton,
            indexerInButton,
            indexerOutButton;
    

    private OdometricSwerve swerve;
    private Arm arm;
    private Indexer indexer;
    private Intake intake;
    private Shooter shooter;
    private Turret turret;
    private VisionSubsystem vision;
    private VisionPreciseShootingOI visionPreciseShooting;
    private VisionDistanceCalculator visionDistanceCalculator;
    private SubsystemBase valueGetter;


    private boolean indexerActive;
    private boolean shooterActive;
    private double armAngle;
    private double climberSpeed;
    private double turretAngle;
    private double targetTurretOffset;

    private double xDeadzone = ControlConstants.xDeadzone;
    private double yDeadzone = ControlConstants.yDeadzone;
    private double zDeadzone = ControlConstants.zDeadzone;
    private double xSensitivity = ControlConstants.xSensitivity;
    private double ySensitivity = ControlConstants.ySensitivity;
    private double zSensitivity = ControlConstants.zSensitivity;

    private boolean useFancyIntakeCommand = true;


    public GRCRobotContainer() {
        driverTab = Shuffleboard.getTab("Driver Controls");

        resetGyroButton = new JoystickButton(driveStick, 5); //initialize buttons
        intakeTrigger = new JoystickButton(driveStick, 1);
        shooterTrigger = new JoystickButton(controlStick, 1);
        intakeInButton = new JoystickButton(driveStick, 12);
        intakeOutButton = new JoystickButton(driveStick, 11);
        indexerInButton = new JoystickButton(controlStick, 10);
        indexerOutButton = new JoystickButton(controlStick, 9);
        climberUpButton = new JoystickButton(controlStick, 12);
        climberDownButton = new JoystickButton(controlStick,11);

        swerve = EntropySwerveFactory.makeSwerve(); //initialize subsystems
        arm = HardwareArmFactory.makeArm();
        indexer = HardwareIndexerFactory.makeIndexer();
        intake = HardwareIntakeFactory.makeIntake();
        shooter = HardwareShooterFactory.makeShooter();
        turret = HardwareTurretFactory.makeTurret();
        vision = new VisionSubsystem(new LimelightVisionComponent());
        visionDistanceCalculator = GenericAutonUtilities.makeEntropyVisionDistanceCalculator(vision);
        valueGetter = new SubsystemBase(){

        };
        valueGetter.setDefaultCommand(new ValueGetterCommand("Arm Angle", () -> arm.getAngle(), valueGetter));

        configureSwerve(); //configure subsystem commands and controls
        configureClimber();
        configureArmIntakeIndexer();
        configureShooter();
        configureTurret();
        configureAutonomous();

    }

    //Configuration
    public void configureSwerve(){
        resetGyroButton.whenPressed(() -> swerve.resetPose());
        swerve.resetPose(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)));
        swerve.setDefaultCommand(new HardDeadzoneSwerveCommand(swerve, this, this));
    }

    public void configureArmIntakeIndexer(){
        arm.setDefaultCommand(new TeleopArmCommand(arm, this));
        indexer.setDefaultCommand(new TeleopIndexerCommand(indexer, this, intake,  1, 1));

        intakeTrigger.whenPressed(() -> {armAngle = Math.PI/3; indexerActive = true;  }); //default indexer
        intakeTrigger.whenReleased(() -> {armAngle = 0; indexerActive = false; });


        intakeInButton.whenHeld(new IntakeArmRunCommand(intake, arm, 1));
        intakeOutButton.whenHeld(new IntakeArmRunCommand(intake, arm, 1));
        indexerInButton.whenHeld(new IndexerRunCommand(indexer,1));
        indexerOutButton.whenPressed(new IndexerRunCommand(indexer, -1));
    }

    public void configureClimber(){
        climberUpButton.whenPressed(()->{climberSpeed=1;}).whenReleased(()->{climberSpeed=0;});
        climberDownButton.whenPressed(()->{climberSpeed=-1;}).whenReleased(()->{climberSpeed=0;});
    }

    public void configureShooter(){
        visionPreciseShooting = new VisionPreciseShootingOI(visionDistanceCalculator);
        shooter.setDefaultCommand(new PreciseShootingCommand(shooter, indexer, visionPreciseShooting, this));
        shooterTrigger.whenPressed(() -> shooterActive=true);
        shooterTrigger.whenReleased(() -> shooterActive=false);
    }

    public void configureTurret() {
        targetTurretOffset = 0;
        turret.setDefaultCommand(new TurretAutoCommand(turret, vision, this));
    }

    private void configureSmartDashboardControls() {
        SmartDashboard.putData("Control Preferences", new Sendable() {
            public void initSendable(SendableBuilder builder) {
                builder.addBooleanProperty("Use Sensors When Indexing", () -> useFancyIntakeCommand, value -> useFancyIntakeCommand = value);
                builder.addDoubleProperty("X Axis Sensitivity", () -> xSensitivity, value -> xSensitivity = value);
                builder.addDoubleProperty("Y Axis Sensitivity", () -> ySensitivity, value -> ySensitivity = value);
                builder.addDoubleProperty("Z Axis Sensitivity", () -> zSensitivity, value -> zSensitivity = value);
                builder.addDoubleProperty("X Axis Deadzone", () -> xDeadzone, value -> xDeadzone = value);
                builder.addDoubleProperty("Y Axis Deadzone", () -> yDeadzone, value -> yDeadzone = value);
                builder.addDoubleProperty("Z Axis Deadzone", () -> zDeadzone, value -> zDeadzone = value);
            }
        });
        SmartDashboard.putData("Swerve Transform", new OdometricSwerveDashboardUtility(swerve));
        SmartDashboard.putData("Vision Distance Calculator", visionDistanceCalculator);
        SmartDashboard.putData("Indexer", indexer);
        SmartDashboard.putData(new Sendable(){
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Turret Radian Offset", () -> getTargetTurretOffset(), value -> { targetTurretOffset = value; });
            }
        });

    }

    public void configureAutonomous(){
        SendableChooser autonomousChooser = new SendableChooser();

        autonomousChooser.addOption("Shoot and Cross the line", new ShootAndCrossTheLineCommand(swerve, shooter, indexer));
        autonomousChooser.addOption("Away from center move forward and shoot", new AwayFromCenterMoveForwardAndShootCommand(swerve, shooter, indexer));
        autonomousChooser.addOption("Away from center move backward and shoot", new AwayFromCenterMoveBackwardAndShootCommand(swerve, shooter, indexer, visionPreciseShooting));
        autonomousChooser.addOption("Citrus Compatible Primary", new CitrusCompatibleCommand(swerve, arm, indexer, intake, turret, this, shooter, visionPreciseShooting));
    


    }

    //Methods from OIs
    public double getX(){
        return driveStick.getX();
    }
    public double getY(){
        return driveStick.getY();
    }
    public double getZ(){
        return driveStick.getZ();
    }
    public double getClimberSpeed(){
        return climberSpeed;
    }
    public double getArmAngle(){
        return armAngle;
    }
    public boolean getIndexerActive() { return indexerActive;}
    public boolean useSensors() {return useFancyIntakeCommand; }
    public boolean getShooterActive() { return shooterActive;}
    public double getTurretAngle() { return turretAngle;}
    public double getTargetTurretOffset() { return targetTurretOffset;}
    public void setTargetTurretOffset(double set){targetTurretOffset = set;}

    public double getxDeadzone(){ return xDeadzone;};
    public double getyDeadzone(){ return yDeadzone;};
    public double getzDeadzone(){ return zDeadzone;};
    public double getxSensitivity() { return xSensitivity;};
    public double getySensitivity() { return ySensitivity;};
    public double getzSensitivity() { return zSensitivity;};
}
