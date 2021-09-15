package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberOI;


public class ClimberCommand extends CommandBase {
    private final Climber climber;
    private final ClimberOI oi;
    private double oiSpeed;

    public ClimberCommand( Climber climber, ClimberOI oi) {
        this.climber = climber;
        this.oi = oi;
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.climber);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        climber.setSpeed(oi.getClimberSpeed());
    }


    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        climber.setSpeed(0);
    }
}
