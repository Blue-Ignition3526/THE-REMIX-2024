package frc.robot.commands.Climbers.Both;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Climbers.Single.ClimberUp;
import frc.robot.subsystems.Climber;

public class BothClimbersUp extends ParallelCommandGroup {
  public BothClimbersUp(Climber leftClimber, Climber rightClimber) {
    addCommands(
      new ClimberUp(leftClimber),
      new ClimberUp(rightClimber)
    );
  }
}
