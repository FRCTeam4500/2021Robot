package frc.robot.subsystems.turret;

public interface TurretOI {
    public double getTurretAngle();
    public double getTargetTurretOffset();
    public void setTargetTurretOffset(double angleRadians);
}
