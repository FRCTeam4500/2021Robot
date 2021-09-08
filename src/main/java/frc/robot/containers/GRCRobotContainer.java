package frc.robot.containers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.ControlConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmOI;
import frc.robot.subsystems.arm.commands.TeleopArmCommand;
import frc.robot.subsystems.arm.factory.HardwareArmFactory;
import frc.robot.subsystems.climber.ClimberOI;
import frc.robot.subsystems.shooter.ShooterOI;
import frc.robot.subsystems.swerve.SwerveOI;
import frc.robot.subsystems.swerve.commands.HardDeadzoneSwerveCommand;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.OdometricWheelModule;
import frc.robot.subsystems.swerve.odometric.factory.EntropySwerveFactory;

import static frc.robot.ControlConstants.xSensitivity;
import static frc.robot.ControlConstants.ySensitivity;
import static frc.robot.ControlConstants.zSensitivity;
import static frc.robot.ControlConstants.xDeadzone;
import static frc.robot.ControlConstants.yDeadzone;
import static frc.robot.ControlConstants.zDeadzone;
import static frc.robot.utility.ExtendedMath.withHardDeadzone;

public class GRCRobotContainer implements RobotContainer, SwerveOI, ClimberOI, ArmOI {


    private ShuffleboardTab driverTab;
    private Joystick
            driveStick = new Joystick(0),
            controlStick = new Joystick(1);


    private JoystickButton
            resetGyroButton = new JoystickButton(driveStick, 5),
            climberUpButton = new JoystickButton(controlStick,9),
            climberDownButton = new JoystickButton(controlStick, 11),
            intakeTrigger = new JoystickButton(driveStick, 1);




    public OdometricSwerve swerve = EntropySwerveFactory.makeSwerve();
    public Arm arm = new HardwareArmFactory().makeArm();


    public double armAngle;
    public double climberSpeed;

    public GRCRobotContainer() {
        driverTab = Shuffleboard.getTab("Driver Controls");
        resetGyroButton.whenPressed(() -> swerve.resetPose());
        swerve.resetPose(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)));

        configureSwerve();

    }

    //Configuration
    public void configureSwerve(){
        swerve.setDefaultCommand(new HardDeadzoneSwerveCommand(swerve, this));
    }

    public void configureArm(){
        arm.setDefaultCommand(new TeleopArmCommand(arm, this));
        intakeTrigger.whenPressed(()->{armAngle = Math.PI/2; });
        intakeTrigger.whenReleased(()->{ armAngle = 0; });
    }

    public void configureClimber(){
        climberUpButton.whenPressed(()->{climberSpeed=1;}).whenReleased(()->{climberSpeed=0;});
        climberDownButton.whenPressed(()->{climberSpeed=-1;}).whenReleased(()->{climberSpeed=0;});
    }

    public double getArmAngle(){
        return armAngle;
    }



    //Methods from OIs
    public double getX(){
        return driveStick.getX();
    }
    public double getY(){
        return driveStick.getY();
    }
    public double getZ(){
        return driveStick.getZ();
    }
    public double getClimberSpeed(){
        return climberSpeed;
    }
}
