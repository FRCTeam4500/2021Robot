/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.odometric.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;

public class OdometricSwerve_AdvancedFollowTrajectoryCommand extends CommandBase {
  private OdometricSwerve swerve;
  private AdvancedSwerveController controller;
  /**
   * Creates a new OdometricSwerve_AdvancedFollowTrajectoryCommand.
   */
  public OdometricSwerve_AdvancedFollowTrajectoryCommand(OdometricSwerve swerve, AdvancedSwerveController controller) {
    this.swerve = swerve;
    this.controller = controller;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.reset(swerve.getCurrentPose().getTranslation());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var output = controller.calculateFieldCentricChassisSpeeds(swerve.getCurrentPose());
    SmartDashboard.putNumber("Output X", output.vxMetersPerSecond);
    SmartDashboard.putNumber("Output Y", output.vyMetersPerSecond);
    swerve.moveFieldCentric(output.vxMetersPerSecond,output.vyMetersPerSecond,output.omegaRadiansPerSecond);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atFinalPose(swerve.getCurrentPose());
  }
}
