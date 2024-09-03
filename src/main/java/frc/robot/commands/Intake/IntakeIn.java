package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeRollers;
import lib.BlueShift.utils.LimelightLED;

public class IntakeIn extends Command {
  private IntakeRollers rollers;

  public IntakeIn(IntakeRollers rollers) {
    this.rollers = rollers;
    addRequirements(rollers);
  }
  
  @Override
  public void initialize() {

  }
  
  @Override
  public void execute() {
    rollers.setRollersIn();
  }
  
  @Override
  public void end(boolean interrupted) {
    if (rollers.hasPiece()) LimelightLED.blinkLeds(Constants.Vision.Limelight3.kName);
    rollers.stop();
  }
  
  @Override
  public boolean isFinished() {
    return rollers.hasPiece();
  }
}
