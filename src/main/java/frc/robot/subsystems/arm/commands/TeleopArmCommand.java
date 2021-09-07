package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmOI;

public class TeleopArmCommand extends CommandBase {
    private ArmOI oi;
    private Arm arm;
    public TeleopArmCommand(Arm arm, ArmOI oi){
        this.arm = arm;
        this.oi = oi;
        addRequirements(arm);
    }
    public void execute(){
        arm.setAngle(oi.getArmAngle());
    }
}
