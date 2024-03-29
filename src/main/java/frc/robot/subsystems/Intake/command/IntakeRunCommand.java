package frc.robot.subsystems.intake.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeOI;
import frc.robot.subsystems.intake.Intake;


public class IntakeRunCommand extends CommandBase {
    private Intake intake;
    private IntakeOI oi;
    public IntakeRunCommand(Intake intake, IntakeOI oi) {
        this.intake = intake;
        this.oi = oi;
        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intake.setSpeed(oi.getIntakeSpeed());
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
