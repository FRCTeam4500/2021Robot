package frc.robot.components;

public interface OutputGetterSetterComponent extends OutputGetterComponent, OutputSetterComponent{
    public void setOutput(double output);
    public double getOutput();
}
