package frc.robot.commands.Climbers.Both;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Climbers.Single.ClimberHome;
import frc.robot.subsystems.Climber;

public class BothClimbersHome extends ParallelCommandGroup {
  public BothClimbersHome(Climber lefClimber, Climber rightClimber) {
    addCommands(
      new ClimberHome(lefClimber),
      new ClimberHome(rightClimber)
    );
  }
}
