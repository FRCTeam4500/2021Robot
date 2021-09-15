package frc.robot.subsystems.indexer.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.IndexerOI;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;

public class TeleopIndexerCommand extends CommandBase {
    private Indexer indexer;
    private IndexerOI oi;
    private Intake intake;
    private double indexerSpeed;
    private double intakeSpeed;
    public TeleopIndexerCommand(Indexer indexer, IndexerOI oi, Intake intake, double indexerSpeed, double intakeSpeed){
        this.indexer = indexer;
        this.intake = intake;
        this.oi = oi;
        this.indexerSpeed = indexerSpeed;
        this.intakeSpeed = -intakeSpeed;
        addRequirements(indexer, intake);
    }
    public void execute(){

        if(oi.getIndexerActive() && oi.useSensors()){ // use sensors
            if(indexer.sensor0RegistersBall()){
                indexer.setSpeed(1);
                intake.setSpeed(0);
            }else{
                indexer.setSpeed(0);
                intake.setSpeed(intakeSpeed);
            }
        }
        else if (oi.getIndexerActive() && !oi.useSensors()){ //dont use sensors
            intake.setSpeed(intakeSpeed);
            indexer.setSpeed(1);

        }
        else{
            indexer.setSpeed(0);
            intake.setSpeed(0);
        }
    }
    public void end(boolean interrupted){
        intake.setSpeed(0);
        indexer.setSpeed(0);
    }
}
