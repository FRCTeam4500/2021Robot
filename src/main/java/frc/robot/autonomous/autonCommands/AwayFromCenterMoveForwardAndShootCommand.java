package frc.robot.autonomous.autonCommands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.pshoot.Autonomous_PreciseShootingCommand;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_AdvancedFollowTrajectoryCommand;

import static frc.robot.autonomous.ExtendedTrajectoryUtilities.tryGetDeployedTrajectory;
import static frc.robot.autonomous.GenericAutonUtilities.createDefaultControllerBuilder;

public class AwayFromCenterMoveForwardAndShootCommand extends SequentialCommandGroup {
    public AwayFromCenterMoveForwardAndShootCommand(OdometricSwerve swerve, Shooter shooter, Indexer indexer){
        addRequirements(swerve, shooter, indexer);
        addCommands(new InstantCommand(() -> swerve.resetPose(new Pose2d(13, -5.75, new Rotation2d(Math.PI))), swerve));
        addCommands(new Autonomous_PreciseShootingCommand(shooter, indexer, -3390, -2520, 1.47, 500)
                .withTimeout(4));
        addCommands(new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                swerve,
                createDefaultControllerBuilder()
                        .withEndRotation(new Rotation2d(Math.PI))
                        .withTrajectory(tryGetDeployedTrajectory("CrossTheLine"))
                        .buildController()));
    }
}
