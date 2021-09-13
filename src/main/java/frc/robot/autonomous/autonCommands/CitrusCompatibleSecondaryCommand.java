package frc.robot.autonomous.autonCommands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.Autonomous_ForceIndexBallsCommand;
import frc.robot.autonomous.pshoot.Autonomous_PreciseShootingCommand;
import frc.robot.autonomous.pshoot.VisionPreciseShootingOI;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_AdvancedFollowTrajectoryCommand;
import frc.robot.subsystems.turret.TurretOI;

import static frc.robot.autonomous.ExtendedTrajectoryUtilities.tryGetDeployedTrajectory;
import static frc.robot.autonomous.GenericAutonUtilities.createDefaultControllerBuilder;

public class CitrusCompatibleSecondaryCommand extends SequentialCommandGroup {
    public CitrusCompatibleSecondaryCommand(OdometricSwerve swerve, Shooter shooter, Indexer indexer, Intake intake, Arm arm, VisionPreciseShootingOI visionPreciseShootingOI, TurretOI toi){
        super(new InstantCommand(() -> swerve.resetPose(new Pose2d(12.565, -4.875, new Rotation2d(Math.PI))), swerve)
                .andThen(() -> toi.setTargetTurretOffset(0.0))
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
                .andThen(() -> toi.setTargetTurretOffset(0.0))
                .andThen(new Autonomous_PreciseShootingCommand(shooter, indexer, visionPreciseShootingOI)));
    }
}
