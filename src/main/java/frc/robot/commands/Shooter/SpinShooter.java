package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SpinShooter extends Command {
  private final Shooter shooter;
  private final Timer timer = new Timer();

  public SpinShooter(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    this.timer.reset();
    this.timer.start();
  }

  @Override
  public void execute() {
    this.shooter.shootSpeaker();
  }

  @Override
  public void end(boolean interrupted) {
    this.shooter.stop();
    this.timer.stop();
  }

  @Override
  public boolean isFinished() {
    return (this.timer.get() > 6.0);
  }
}
