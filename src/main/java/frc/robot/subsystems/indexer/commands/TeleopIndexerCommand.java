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
    private boolean lastActive;
    public TeleopIndexerCommand(Indexer indexer, IndexerOI oi, Intake intake, double indexerSpeed, double intakeSpeed){
        this.indexer = indexer;
        this.oi = oi;
        addRequirements(indexer);
    }
    public void execute(){

        if(oi.getIndexerActive() && oi.useSensors()){ // use sensors
            if (indexer.sensor0RegistersBall() && !indexer.sensor5RegistersBall()){
                indexer.setSpeed(indexerSpeed);
                intake.setSpeed(0);
            }
            else if(indexer.sensor1RegistersBall() ||
                    indexer.sensor2RegistersBall() ||
                    indexer.sensor3RegistersBall() ||
                    indexer.sensor4RegistersBall() ||
                    indexer.sensor5RegistersBall()){
                indexer.setSpeed(0);
                intake.setSpeed(intakeSpeed);
            }
        }
        else if (oi.getIndexerActive() && !oi.useSensors()){ //dont use sensors
            indexer.setSpeed(1);
            intake.setSpeed(1);
        }
        else { //stop indexer
            if (indexer.getSpeed() != 0.0 && oi.getIndexerActive() == false && lastActive == true) { //only runs on the cycle when the indexer turns off
                indexer.setSpeed(0); //so that it doesnt lock the indexer at 0
            }
        }
        lastActive = oi.getIndexerActive();
    }
    public void end(boolean interrupted){
        intake.setSpeed(0);
        indexer.setSpeed(0);
    }
}
