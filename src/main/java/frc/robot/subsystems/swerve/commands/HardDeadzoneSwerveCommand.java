package frc.robot.subsystems.swerve.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveOI;
import frc.robot.subsystems.swerve.kinematic.KinematicSwerve;
import frc.robot.subsystems.swerve.normal.NormalSwerve;

import static frc.robot.ControlConstants.*;
import static frc.robot.utility.ExtendedMath.withHardDeadzone;


public class HardDeadzoneSwerveCommand extends CommandBase {
    private final KinematicSwerve swerve;
    private final SwerveOI oi;

    public HardDeadzoneSwerveCommand(KinematicSwerve swerve, SwerveOI oi) { //takes any subclass of Swerve
        this.swerve = swerve;
        this.oi = oi;
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        var forwardSpeed = withHardDeadzone(-oi.getY(), yDeadzone) * ySensitivity;
        var leftwardSpeed = withHardDeadzone(-oi.getX(), xDeadzone) * xSensitivity;
        var counterClockwardSpeed = withHardDeadzone(-oi.getZ(), zDeadzone) * zSensitivity;
        swerve.moveFieldCentric(forwardSpeed, leftwardSpeed, counterClockwardSpeed);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
