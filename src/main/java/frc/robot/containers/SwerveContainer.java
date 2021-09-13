package frc.robot.containers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.ControlConstants;
import frc.robot.ControlOI;
import frc.robot.subsystems.swerve.SwerveOI;
import frc.robot.subsystems.swerve.commands.HardDeadzoneSwerveCommand;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.OdometricSwerveDashboardUtility;
import frc.robot.subsystems.swerve.odometric.factory.EntropySwerveFactory;

public class SwerveContainer implements RobotContainer, SwerveOI, ControlOI {
    private OdometricSwerve swerve;

    private Joystick driveStick;
    private JoystickButton resetGyroButton;


    private double xDeadzone = ControlConstants.xDeadzone;
    private double yDeadzone = ControlConstants.yDeadzone;
    private double zDeadzone = ControlConstants.zDeadzone;
    private double xSensitivity = ControlConstants.xSensitivity;
    private double ySensitivity = ControlConstants.ySensitivity;
    private double zSensitivity = ControlConstants.zSensitivity;


    public SwerveContainer(){
        driveStick = new Joystick(0);


        swerve = EntropySwerveFactory.makeSwerve();
        swerve.setDefaultCommand(new HardDeadzoneSwerveCommand(swerve, this, this));

        resetGyroButton = new JoystickButton(driveStick, 5);
        resetGyroButton.whenPressed(() -> {swerve.resetPose();});

        configureSmartDashboard();
    }

    public void configureSmartDashboard(){
        SmartDashboard.putData("Control Preferences", new Sendable() {
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("X Axis Sensitivity", () -> xSensitivity, value -> xSensitivity = value);
                builder.addDoubleProperty("Y Axis Sensitivity", () -> ySensitivity, value -> ySensitivity = value);
                builder.addDoubleProperty("Z Axis Sensitivity", () -> zSensitivity, value -> zSensitivity = value);
                builder.addDoubleProperty("X Axis Deadzone", () -> xDeadzone, value -> xDeadzone = value);
                builder.addDoubleProperty("Y Axis Deadzone", () -> yDeadzone, value -> yDeadzone = value);
                builder.addDoubleProperty("Z Axis Deadzone", () -> zDeadzone, value -> zDeadzone = value);
            }
        });
        SmartDashboard.putData("Swerve Transform", new OdometricSwerveDashboardUtility(swerve));
    }

    public double getX(){
        return driveStick.getX();
    }
    public double getY(){
        return driveStick.getY();
    }
    public double getZ(){
        return driveStick.getZ();
    }


    public double getxDeadzone(){ return xDeadzone;};
    public double getyDeadzone(){ return yDeadzone;};
    public double getzDeadzone(){ return zDeadzone;};
    public double getxSensitivity() { return xSensitivity;};
    public double getySensitivity() { return ySensitivity;};
    public double getzSensitivity() { return zSensitivity;};
}
