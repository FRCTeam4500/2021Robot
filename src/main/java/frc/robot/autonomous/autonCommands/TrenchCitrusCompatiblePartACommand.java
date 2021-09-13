package frc.robot.autonomous.autonCommands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.pshoot.Autonomous_PreciseShootingCommand;
import frc.robot.autonomous.pshoot.VisionPreciseShootingOI;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_AdvancedFollowTrajectoryCommand;

import static frc.robot.autonomous.ExtendedTrajectoryUtilities.tryGetDeployedTrajectory;
import static frc.robot.autonomous.GenericAutonUtilities.createDefaultControllerBuilder;

public class TrenchCitrusCompatiblePartACommand extends SequentialCommandGroup {
    public TrenchCitrusCompatiblePartACommand(OdometricSwerve swerve, Shooter shooter, Indexer indexer, Intake intake, Arm arm, VisionPreciseShootingOI visionPreciseShootingOI){
        super(new TrenchCitrusPart1Command(swerve, shooter, indexer, intake, arm, visionPreciseShootingOI)
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
                .andThen(new Autonomous_PreciseShootingCommand(shooter, indexer, visionPreciseShootingOI).withTimeout(6)));
    }
}
