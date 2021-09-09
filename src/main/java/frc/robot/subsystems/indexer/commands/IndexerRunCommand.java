package frc.robot.subsystems.indexer.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.Indexer;

public class IndexerRunCommand extends CommandBase {
    private Indexer indexer;
    private double speed;
    public IndexerRunCommand(Indexer indexer, double speed){
        this.indexer = indexer;
        this.speed = speed;
    }
    public void execute(){
        indexer.setSpeed(speed);
    }
    public void end(boolean interrupted){
        indexer.setSpeed(0);
    }
}
