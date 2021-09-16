package frc.robot.subsystems.indexer.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.indexer.Indexer;

public class IndexerRunSensorCommand extends CommandBase {
    private Indexer indexer;
    private double speed;
    public IndexerRunSensorCommand(Indexer indexer, double speed){
        this.indexer = indexer;
        this.speed = speed;

    }
    public void execute(){

        if (indexer.sensor0RegistersBall() && speed != 0){
            indexer.setSpeed(speed);
        }
        else if (indexer.sensor0RegistersBall() && speed == 0){
            indexer.setSpeed(1);
        }
        else{
            indexer.setSpeed(0);
        }
    }

    public boolean isFinished(){
        return false;
        //!indexer.sensor0RegistersBall();
    }
    public void end(boolean interrupted){
        indexer.setSpeed(0);
    }
}
