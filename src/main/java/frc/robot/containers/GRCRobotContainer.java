package frc.robot.containers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.GenericAutonUtilities;
import frc.robot.autonomous.VisionDistanceCalculator;
import frc.robot.autonomous.pshoot.Autonomous_PreciseShootingCommand;
import frc.robot.autonomous.pshoot.VisionPreciseShootingOI;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmOI;
import frc.robot.subsystems.arm.commands.TeleopArmCommand;
import frc.robot.subsystems.arm.factory.HardwareArmFactory;
import frc.robot.subsystems.climber.ClimberOI;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerOI;
import frc.robot.subsystems.indexer.commands.TeleopIndexerCommand;
import frc.robot.subsystems.indexer.factory.HardwareIndexerFactory;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterOI;
import frc.robot.subsystems.shooter.command.PreciseShootingCommand;
import frc.robot.subsystems.shooter.command.ShootStraightCommand;
import frc.robot.subsystems.shooter.factory.HardwareShooterFactory;
import frc.robot.subsystems.swerve.SwerveOI;
import frc.robot.subsystems.swerve.commands.HardDeadzoneSwerveCommand;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.factory.EntropySwerveFactory;
import frc.robot.subsystems.intake.factory.HardwareIntakeFactory;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretOI;
import frc.robot.subsystems.turret.factory.HardwareTurretFactory;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.components.hardware.LimelightVisionComponent;
import frc.robot.subsystems.turret.command.TurretAutoCommand;

public class GRCRobotContainer implements RobotContainer, SwerveOI, ClimberOI, ArmOI, IndexerOI, ShooterOI, TurretOI {


    private ShuffleboardTab driverTab;
    private Joystick
            driveStick = new Joystick(0),
            controlStick = new Joystick(1);


    private JoystickButton
            resetGyroButton,
            climberUpButton,
            climberDownButton,
            intakeTrigger,
            shooterTrigger;
    

    private OdometricSwerve swerve;
    private Arm arm;
    private Indexer indexer;
    private Intake intake;
    private Shooter shooter;
    private Turret turret;
    private VisionSubsystem vision;
    private VisionPreciseShootingOI visionPreciseShooting;

    private boolean indexerActive;
    private boolean shooterActive;
    private double armAngle;
    private double climberSpeed;
    private double turretAngle;
    private double targetTurretOffset;


    public GRCRobotContainer() {
        driverTab = Shuffleboard.getTab("Driver Controls");

        resetGyroButton = new JoystickButton(driveStick, 5); //initialize buttons
        climberUpButton = new JoystickButton(controlStick,9);
        climberDownButton = new JoystickButton(controlStick, 11);
        intakeTrigger = new JoystickButton(driveStick, 1);
        shooterTrigger = new JoystickButton(controlStick, 1);

        swerve = EntropySwerveFactory.makeSwerve(); //initialize subsystems
        arm = HardwareArmFactory.makeArm();
        indexer = HardwareIndexerFactory.makeIndexer();
        intake = HardwareIntakeFactory.makeIntake();
        shooter = HardwareShooterFactory.makeShooter();
        turret = HardwareTurretFactory.makeTurret();
        vision = new VisionSubsystem(new LimelightVisionComponent());

        configureSwerve(); //configure subsystem commands and controls
        configureClimber();
        configureArmIntakeIndexer();
        configureShooter();
        configureTurret();
    }

    //Configuration
    public void configureSwerve(){
        resetGyroButton.whenPressed(() -> swerve.resetPose());
        swerve.resetPose(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)));
        swerve.setDefaultCommand(new HardDeadzoneSwerveCommand(swerve, this));
    }

    public void configureArmIntakeIndexer(){
        arm.setDefaultCommand(new TeleopArmCommand(arm, this));
        indexer.setDefaultCommand(new TeleopIndexerCommand(indexer, this, intake,  1, 1));
        intakeTrigger.whenPressed(()->{armAngle = Math.PI/2; indexerActive = true;  });
        intakeTrigger.whenReleased(()->{armAngle = 0; indexerActive = false; });
    }

    public void configureClimber(){
        climberUpButton.whenPressed(()->{climberSpeed=1;}).whenReleased(()->{climberSpeed=0;});
        climberDownButton.whenPressed(()->{climberSpeed=-1;}).whenReleased(()->{climberSpeed=0;});
    }

    public void configureShooter(){
        visionPreciseShooting = new VisionPreciseShootingOI(GenericAutonUtilities.makeEntropyVisionDistanceCalculator(vision));
        shooter.setDefaultCommand(new PreciseShootingCommand(shooter, indexer, visionPreciseShooting, this));
        shooterTrigger.whenPressed(()->{shooterActive=true;});
        shooterTrigger.whenReleased(()->{shooterActive=false;});
    }

    public void configureTurret() {
        targetTurretOffset = 0;
        turret.setDefaultCommand(new TurretAutoCommand(turret, vision, this));
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
    public boolean getShooterActive() { return shooterActive;}
    public double getTurretAngle() { return turretAngle;}
    public double getTargetTurretOffset() { return targetTurretOffset;}
}
