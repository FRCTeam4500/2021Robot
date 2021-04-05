// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.odometric.command.v2;

import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.utility.ExtendedMath;

public class OdometricSwerve_FollowDottedTrajectoryCommand extends OdometricSwerve_FollowTrajectoryCommand {

  protected double internalTime = 0.0, timeStep = 0.02, threshold = 0.1;

  public OdometricSwerve_FollowDottedTrajectoryCommand(OdometricSwerve swerve, Trajectory trajectory,
      HolonomicDriveController controller) {
    super(swerve, trajectory, controller);
  }
  @Override
  public void initialize() {
    internalTime = 0.0;
  }
  @Override
  public void execute() {

    while(internalTime < trajectory.getTotalTimeSeconds() && signedDistance() < threshold){
      internalTime += timeStep;
    }
    applyState(trajectory.sample(internalTime));
  }
  double signedDistance(){
    var sample = trajectory.sample(internalTime);
    var currPose = swerve.getCurrentPose();
    var displacement = sample.poseMeters.getTranslation().minus(currPose.getTranslation());
    var direction = new Translation2d(sample.poseMeters.getRotation().getCos(), sample.poseMeters.getRotation().getSin());
    return Math.signum(ExtendedMath.dot(displacement, direction))*displacement.getNorm();
  }
  @Override
  public boolean isFinished() {
    return internalTime >= trajectory.getTotalTimeSeconds() && currentTranslation.getDistance(swerve.getCurrentPose().getTranslation()) <= threshold;
  }
  public OdometricSwerve_FollowDottedTrajectoryCommand(OdometricSwerve swerve, Trajectory trajectory,
      HolonomicDriveController controller, double threshold, double timeStep) {
    super(swerve, trajectory, controller);
    this.timeStep = timeStep;
    this.threshold = threshold;
  }
  public OdometricSwerve_FollowDottedTrajectoryCommand(OdometricSwerve swerve, Trajectory trajectory,
      HolonomicDriveController controller, double threshold) {
    super(swerve, trajectory, controller);
    this.threshold = threshold;
  }
  public double getTimeStep() {
    return timeStep;
  }
  public void setTimeStep(double timeStep) {
    this.timeStep = timeStep;
  }
  public double getThreshold() {
    return threshold;
  }
  public void setThreshold(double threshold) {
    this.threshold = threshold;
  }
  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      builder.addDoubleProperty("Timestep", this::getTimeStep, this::setTimeStep);
      builder.addDoubleProperty("Threshold", this::getThreshold, this::setThreshold);
  }
}