package frc.robot.commands.SwerveDrive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import lib.BlueShift.utils.JoystickUtils;

public class DriveSwerve extends Command {
  SwerveDrive swerveDrive;

  SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.SwerveDrive.PhysicalModel.kMaxAcceleration.magnitude());
  SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.SwerveDrive.PhysicalModel.kMaxAcceleration.magnitude());
  SlewRateLimiter rotLimiter = new SlewRateLimiter(Constants.SwerveDrive.PhysicalModel.kMaxAngularAcceleration.magnitude());

  Supplier<Double> xSpeed;
  Supplier<Double> ySpeed;
  Supplier<Double> rotSpeed;
  Supplier<Boolean> fieldRelative;
  Supplier<Boolean> trackingSpeaker;
  
  public DriveSwerve(
    SwerveDrive swerveDrive,
    Supplier<Double> xSpeed,
    Supplier<Double> ySpeed,
    Supplier<Double> rotSpeed,
    Supplier<Boolean> fieldRelative,
    Supplier<Boolean> trackingSpeaker
  ) {
    this.swerveDrive = swerveDrive;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rotSpeed = rotSpeed;
    this.fieldRelative = fieldRelative;
    this.trackingSpeaker = trackingSpeaker;
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    double x = xSpeed.get();
    double y = ySpeed.get();
    double rot = rotSpeed.get();

    x = JoystickUtils.applyDeadbband(x, Constants.SwerveDrive.kJoystickDeadband);
    y = JoystickUtils.applyDeadbband(y, Constants.SwerveDrive.kJoystickDeadband);
    rot = JoystickUtils.applyDeadbband(rot, Constants.SwerveDrive.kJoystickDeadband);
    
    x = xLimiter.calculate(x);
    y = yLimiter.calculate(y);
    rot = rotLimiter.calculate(rot);

    x *= Constants.SwerveDrive.PhysicalModel.kMaxSpeed.in(MetersPerSecond);
    y *= Constants.SwerveDrive.PhysicalModel.kMaxSpeed.in(MetersPerSecond);
    rot *= Constants.SwerveDrive.PhysicalModel.kMaxAngularSpeed.in(RadiansPerSecond);
    
    if (this.fieldRelative.get()) swerveDrive.driveFieldRelative(x, y, rot);
    else swerveDrive.driveRobotRelative(x, y, rot);
  }

  @Override
  public void end(boolean interrupted) {
    swerveDrive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
