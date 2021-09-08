package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.OutputSetterComponent;

public class Intake extends SubsystemBase {
    OutputSetterComponent motor;
    public Intake(OutputSetterComponent motor){
        this.motor = motor;

    }

    public void setSpeed(double speed){
        motor.setOutput(speed);
    }
}
