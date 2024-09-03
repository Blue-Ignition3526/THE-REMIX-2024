package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeLifter;
import frc.robot.subsystems.IntakeRollers;

public class ShootAmp extends Command {
  private final IntakeRollers rollers;
  private final IntakeLifter lifter;
  private final Timer timer = new Timer();

  public ShootAmp(IntakeRollers rollers, IntakeLifter lifter) {
    this.rollers = rollers;
    this.lifter = lifter;
    addRequirements(rollers, lifter);
  }

  @Override
  public void initialize() {
    this.timer.reset();
    this.timer.start();
  }

  @Override
  public void execute() {
    this.lifter.setLifterAngle(Constants.Intake.Lifter.PhysicalModel.kAmplifierFinalAngle);
    if (this.timer.get() > 0.3) this.rollers.setRollersSpeed(-0.43);
  }

  @Override
  public void end(boolean interrupted) {
    this.rollers.stop();
    this.timer.stop();
    this.lifter.setLifterAngle(Constants.Intake.Lifter.PhysicalModel.kShooterAngle);
  }

  @Override
  public boolean isFinished() {
    return (this.timer.get()>1.0);
  }
}
