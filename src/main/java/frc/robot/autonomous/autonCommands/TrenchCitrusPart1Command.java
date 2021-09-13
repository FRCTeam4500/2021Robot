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

import static frc.robot.autonomous.ExtendedTrajectoryUtilities.tryGetDeployedTrajectory;
import static frc.robot.autonomous.GenericAutonUtilities.createDefaultControllerBuilder;

public class TrenchCitrusPart1Command extends SequentialCommandGroup {
    TrenchCitrusPart1Command(OdometricSwerve swerve, Shooter shooter, Indexer indexer, Intake intake, Arm arm, VisionPreciseShootingOI visionPreciseShootingOI){
        super(new InstantCommand(() -> swerve.resetPose(new Pose2d(13, -7.5, new Rotation2d(Math.PI))), swerve)
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
                                )));
    }
}
