package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class TestPath extends SequentialCommandGroup {
  public TestPath() {
    String path = "Test Path";

    var swerve = Drivetrain.getInstance();
    addCommands(swerve.followTrajectoryCommand(path, true));
  }
}
