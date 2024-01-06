package frc.robot.layout;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.core.util.controllers.ButtonMap.Axis;
import frc.robot.core.util.controllers.ButtonMap.Button;
import frc.robot.core.swerve.TalonSwerve.SwerveConstants;
import frc.robot.core.util.controllers.GameController;
import frc.robot.subsystems.Drivetrain;

public class TwoJoyStickDriverMap extends DriverMap {
  private Drivetrain drivetrain;

  public TwoJoyStickDriverMap(GameController controller) {
    super(controller);
    drivetrain = Drivetrain.getInstance();
  }

  @Override
  public ChassisSpeeds getChassisSpeeds() {
    double x = Math.pow(controller.getAxis(Axis.AXIS_LEFT_X), 1) * SwerveConstants.MAX_VELOCITY;
    double y = Math.pow(controller.getAxis(Axis.AXIS_LEFT_Y), 1) * SwerveConstants.MAX_VELOCITY;
    double rot = controller.getAxis(Axis.AXIS_RIGHT_X) * SwerveConstants.MAX_ANGULAR_VELOCITY * 0.7;

    return ChassisSpeeds.fromFieldRelativeSpeeds(-y, -x, -rot, new Rotation2d(0, 0));
  }

  @Override
  public JoystickButton getTestButton() {
    return controller.getButton(Button.BUTTON_B);
  }

  public JoystickButton getResetOdometryButton() {
    return controller.getButton(Button.BUTTON_A);
  }

  // Driving logic is already implemented in the base class (DriverMap) using Drivetrain.java's
  // superclass!
  @Override
  public void registerCommands() {
    super.registerCommands();
    getTestButton().onTrue(drivetrain.followTrajectoryCommand("Test Path", true));
  }
}
