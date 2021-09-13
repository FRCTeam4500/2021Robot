package frc.robot.autonomous.autonCommands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.autonomous.Autonomous_IndexBallsCommand;
import frc.robot.autonomous.pshoot.Autonomous_PreciseShootingCommand;
import frc.robot.autonomous.pshoot.VisionPreciseShootingOI;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_AdvancedFollowTrajectoryCommand;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretOI;

import static frc.robot.autonomous.ExtendedTrajectoryUtilities.tryGetDeployedTrajectory;
import static frc.robot.autonomous.GenericAutonUtilities.createDefaultControllerBuilder;

public class CitrusCompatibleCommand extends SequentialCommandGroup {
    public CitrusCompatibleCommand(OdometricSwerve swerve, Arm arm, Indexer indexer, Intake intake, Turret turret, TurretOI toi, Shooter shooter, VisionPreciseShootingOI soi){
        super(new InstantCommand(() -> toi.setTargetTurretOffset(Units.degreesToRadians(3.5)))
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
                .andThen(() -> toi.setTargetTurretOffset(Units.degreesToRadians(3.0)))
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
                        new Autonomous_PreciseShootingCommand(shooter, indexer, soi)
                                .withTimeout(4)));
    }
}
