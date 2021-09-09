package frc.robot.autonomous.autonCommands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.pshoot.Autonomous_PreciseShootingCommand;
import frc.robot.autonomous.pshoot.VisionPreciseShootingOI;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_AdvancedFollowTrajectoryCommand;

import static frc.robot.autonomous.ExtendedTrajectoryUtilities.tryGetDeployedTrajectory;
import static frc.robot.autonomous.GenericAutonUtilities.createDefaultControllerBuilder;

public class AwayFromCenterMoveBackwardAndShootCommand extends SequentialCommandGroup {
    public AwayFromCenterMoveBackwardAndShootCommand(OdometricSwerve swerve, Shooter shooter, Indexer indexer, VisionPreciseShootingOI soi){
        addRequirements(swerve, shooter, indexer);
        addCommands(new InstantCommand(() -> swerve.resetPose(new Pose2d(13, -2.5, new Rotation2d(Math.PI))), swerve));
        addCommands(new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                swerve,
                createDefaultControllerBuilder()
                        .withTrajectory(tryGetDeployedTrajectory("AwayFromCenterBackward"))
                        .withEndRotation(new Rotation2d(Math.PI)).buildController()));
        addCommands(new InstantCommand(() -> swerve.moveFieldCentric(0, 0, 0), swerve));
        addCommands(new Autonomous_PreciseShootingCommand(shooter, indexer, soi)
                .withTimeout(2));
    }
}
