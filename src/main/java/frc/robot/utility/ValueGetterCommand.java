package frc.robot.utility;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.DoubleSupplier;

public class ValueGetterCommand extends CommandBase {
    private String key;
    private DoubleSupplier value;

    public ValueGetterCommand(String key, DoubleSupplier value, Subsystem... requirements){
        this.key = key;
        this.value = value;
        addRequirements(requirements);
    }

    public void execute(){
        SmartDashboard.putNumber(key, value.getAsDouble());
    }
}
