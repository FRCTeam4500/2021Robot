package frc.robot.subsystems.intake.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import org.opencv.core.Mat;

public class WeirdIntakeArmRunCommand extends CommandBase {
    private Arm arm;
    private Intake intake;
    private Indexer indexer;
    private double speed;
    public WeirdIntakeArmRunCommand(Intake intake, Arm arm, Indexer indexer, double speed){
        this.arm = arm;
        this.intake = intake;
        this.indexer = indexer;
        this.speed = speed;
    }
    public void execute(){

        arm.setAngle(Math.PI/2.8);
        intake.setSpeed(speed);
        if(speed == 0 && !indexer.sensor0RegistersBall()){
            intake.setSpeed(0.6);
        }
    }

    public void end(boolean interrupted){
        arm.setAngle(0);
        intake.setSpeed(0);
    }
}
