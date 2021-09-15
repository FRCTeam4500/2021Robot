package frc.robot.subsystems.shooter.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.autonomous.pshoot.PreciseShootingOI;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterOI;

public class PreciseShootingCommand extends CommandBase {
    private Shooter shooter;
    private Indexer indexer;
    private PreciseShootingOI oi;
    private ShooterOI shoi;

    public PreciseShootingCommand(Shooter shooter, Indexer indexer, PreciseShootingOI oi, ShooterOI shoi) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.oi = oi;
        this.shoi = shoi;
        addRequirements(shooter);
    }

    public void execute() {
        if (shoi.getShooterActive()) {
            double topSpeed = oi.getTopSpeed();
            double bottomSpeed = oi.getBottomSpeed();
            double coeff = oi.getCoefficient();
            var threshold = oi.getThreshold();
            shooter.run(topSpeed * coeff, bottomSpeed * coeff);
            if (shooter.atSpeeds(threshold)) {
                indexer.setSpeed(1);
            } else {
                indexer.setSpeed(0);
            }
        }
        else{
            shooter.run(0,0);
            indexer.setSpeed(0);
        }
    }
    public void end(boolean interrupted){
        shooter.run(0,0);
        indexer.setSpeed(0);
    }
}

