package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeLifter;
import frc.robot.subsystems.IntakeRollers;
import lib.BlueShift.utils.LimelightLED;

public class PickUpPiece extends Command {
  private IntakeRollers rollers;
  private IntakeLifter lifter;

  public PickUpPiece(IntakeRollers rollers, IntakeLifter lifter) {
    this.rollers = rollers;
    this.lifter = lifter;
    addRequirements(rollers, lifter);
  }

  @Override
  public void initialize() {
    this.lifter.setLifterAngle(Constants.Intake.Lifter.PhysicalModel.kGroundAngle);
  }
  
  @Override
  public void execute() {
    this.rollers.setRollersIn();
  }

  @Override
  public void end(boolean interrupted) {
    if (this.rollers.hasPiece()) LimelightLED.blinkLeds(Constants.Vision.Limelight3.kName);
    this.lifter.setLifterAngle(Constants.Intake.Lifter.PhysicalModel.kShooterAngle);
    this.rollers.stop();
  }

  @Override
  public boolean isFinished() {
    return this.rollers.hasPiece();
  }
}
