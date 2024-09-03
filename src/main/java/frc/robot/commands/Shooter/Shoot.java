package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
  private final Shooter shooter;
  private final IntakeRollers rollers;
  private final Timer timer = new Timer();

  public Shoot(Shooter shooter, IntakeRollers rollers) {
    this.shooter = shooter;
    this.rollers = rollers;
    addRequirements(shooter, rollers);
  }

  @Override
  public void initialize() {
    this.timer.reset();
    this.timer.start();
  }
  
  @Override
  public void execute() {
    this.shooter.shootSpeaker();
    this.rollers.giveToShooter();
  }

  @Override
  public void end(boolean interrupted) {
    this.shooter.stop();
    this.rollers.stop();
    this.timer.stop();
  }

  @Override
  public boolean isFinished() {
    return (this.timer.get() > 1.0);
  }
}
