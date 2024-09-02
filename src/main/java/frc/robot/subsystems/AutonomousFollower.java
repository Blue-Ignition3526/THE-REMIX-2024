package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AutonomousFollower extends SubsystemBase {
  public AutonomousFollower(
    Supplier<Pose2d> poseSupplier,
    Consumer<Pose2d> resetPose,
    Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
    Consumer<ChassisSpeeds> robotRelativeOutput
  ) {
    AutoBuilder.configureHolonomic(
        poseSupplier,
        resetPose,
        robotRelativeSpeedsSupplier,
        robotRelativeOutput,
        new HolonomicPathFollowerConfig(
            Constants.SwerveDrive.Autonomous.kTranslatePIDConstants,
            Constants.SwerveDrive.Autonomous.kRotatePIDConstants,
            Constants.SwerveDrive.Autonomous.kMaxSpeedMetersPerSecond.in(MetersPerSecond),
            Constants.SwerveDrive.PhysicalModel.kWheelBase.in(Meters) / 2,
            new ReplanningConfig(true, true)
        ),
        () -> {
          if (DriverStation.getAlliance().isPresent()) return DriverStation.getAlliance().get() == Alliance.Red;
          return false;
        },
        this
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
