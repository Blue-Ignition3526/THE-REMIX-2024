package frc.robot.commands.Climbers.Both;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Climbers.Single.ClimberFullyDown;
import frc.robot.subsystems.Climber;

public class BothClimbersFullyDown extends ParallelCommandGroup {
  public BothClimbersFullyDown(Climber leftClimber, Climber rightClimber) {
    addCommands(
      new ClimberFullyDown(leftClimber),
      new ClimberFullyDown(rightClimber)
    );
  }
}
