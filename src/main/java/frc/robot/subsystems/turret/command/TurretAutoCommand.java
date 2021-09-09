package frc.robot.subsystems.turret.command;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.components.hardware.LimelightVisionComponent;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretOI;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class TurretAutoCommand extends PIDCommand {
    public TurretAutoCommand(Turret turret, VisionSubsystem limelight, TurretOI oi){
        super(new PIDController(-3, 0, 0), limelight::getHorizontalOffset, oi::getTargetTurretOffset, turret::setTurretOutput, limelight, turret);
    }
}
