package frc.robot.commands.Climbers.Both;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Climbers.Single.ClimberDown;
import frc.robot.subsystems.Climber;

public class BothClimbersDown extends ParallelCommandGroup {
  public BothClimbersDown(Climber leftClimber, Climber rightClimber) {
    addCommands(
      new ClimberDown(leftClimber),
      new ClimberDown(rightClimber)
    );
  }
}
