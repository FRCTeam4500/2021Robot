package frc.robot.autonomous.autonCommands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.autonomous.Autonomous_IndexBallsCommand;
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

public class TrenchCitrusCompatibleBCommand extends SequentialCommandGroup {
    public TrenchCitrusCompatibleBCommand(OdometricSwerve swerve, Shooter shooter, Indexer indexer, Intake intake, Arm arm, VisionPreciseShootingOI visionPreciseShootingOI){
        new TrenchCitrusPart1Command(swerve, shooter, indexer, intake, arm, visionPreciseShootingOI)
                .andThen(
                        new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                                swerve,
                                createDefaultControllerBuilder()
                                        .withTrajectory(tryGetDeployedTrajectory("TrenchCitrusCompatiblePart2B"))
                                        .withEndRotation(new Rotation2d(7 * Math.PI / 6))
                                        .buildController()))
                .andThen(() -> arm.setAngle(Math.PI/2), arm)
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
}

