package frc.robot.subsystems.intake.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import org.opencv.core.Mat;

public class IntakeArmRunCommand extends CommandBase {
    private Arm arm;
    private Intake intake;
    private double speed;
    public IntakeArmRunCommand(Intake intake, Arm arm, double speed){
        this.arm = arm;
        this.intake = intake;
        this.speed = speed;
    }
    public void execute(){

        arm.setAngle(Math.PI/2);
        intake.setSpeed(speed);
    }
    public void end(boolean interrupted){
        arm.setAngle(0);
        intake.setSpeed(0);
    }
}
