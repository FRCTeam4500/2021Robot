package frc.robot.subsystems.arm;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.AngleGetterSetterComponent;
import frc.robot.components.AngleSetterComponent;

public class Arm extends SubsystemBase {
    AngleGetterSetterComponent motor;
    public double armAngle;
    public final double ARM_ROTS_PER_MOTOR_ROTS = 0.2; //(Math.PI/2)/(3600.0/4096.0);
    public Arm(AngleGetterSetterComponent motor) {
        this.motor = motor;
    }

    public void setAngle(double angle){
        motor.setAngle(angle / ARM_ROTS_PER_MOTOR_ROTS);
    }
    public double getAngle() {
        return motor.getAngle() * ARM_ROTS_PER_MOTOR_ROTS;

    }
}

