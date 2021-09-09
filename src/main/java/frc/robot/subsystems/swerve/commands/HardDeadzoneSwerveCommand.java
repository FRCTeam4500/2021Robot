package frc.robot.subsystems.swerve.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ControlOI;
import frc.robot.subsystems.swerve.SwerveOI;
import frc.robot.subsystems.swerve.kinematic.KinematicSwerve;
import frc.robot.subsystems.swerve.normal.NormalSwerve;

import static frc.robot.utility.ExtendedMath.withHardDeadzone;


public class HardDeadzoneSwerveCommand extends CommandBase {
    private final KinematicSwerve swerve;
    private final SwerveOI oi;
    private ControlOI coi;

    public HardDeadzoneSwerveCommand(KinematicSwerve swerve, SwerveOI oi, ControlOI coi) { //takes any subclass of Swerve
        this.swerve = swerve;
        this.oi = oi;
        this.coi = coi;
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        var forwardSpeed = withHardDeadzone(-oi.getY(), coi.getyDeadzone()) * coi.getySensitivity();
        var leftwardSpeed = withHardDeadzone(-oi.getX(), coi.getxDeadzone()) * coi.getxSensitivity();
        var counterClockwardSpeed = withHardDeadzone(-oi.getZ(), coi.getzDeadzone()) * coi.getzSensitivity();
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
