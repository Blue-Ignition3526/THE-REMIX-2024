package lib.BlueShift.odometry.swerve;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.BlueShift.odometry.EstimatedPositionProvider;
import lib.BlueShift.odometry.vision.OdometryCamera;
import lib.BlueShift.odometry.vision.VisionOdometryPoseEstimate;
import lib.BlueShift.odometry.vision.camera.LimelightOdometryCamera;

public class FilteredSwerveDrivePoseEstimator extends SubsystemBase implements EstimatedPositionProvider {
  private final Supplier<SwerveModulePosition[]> modulePositionsSupplier;
  private final Supplier<Rotation2d> headingSupplier;
  private boolean visionEnabled;

  private final OdometryCamera[] cameras;
  private final SwerveDrivePoseEstimator estimator;

  public FilteredSwerveDrivePoseEstimator(
    SwerveDriveKinematics kinematics,
    Supplier<SwerveModulePosition[]> modulePositionsSupplier,
    Supplier<Rotation2d> headingSupplier,
    OdometryCamera[] cameras,
    Pose2d initialPose,
    Matrix<N3, N1> stateStdDev,
    Matrix<N3, N1> visionStdDev,
    boolean visionEnabled
  ) {
    this.modulePositionsSupplier = modulePositionsSupplier;
    this.headingSupplier = headingSupplier;
    this.visionEnabled = visionEnabled;

    this.cameras = cameras;
    this.estimator = new SwerveDrivePoseEstimator(
      kinematics,
      headingSupplier.get(),
      modulePositionsSupplier.get(),
      initialPose,
      stateStdDev,
      visionStdDev
    );
  }

  public void setVisionEnabled(boolean enabled) {
    visionEnabled = enabled;
  }

  public void enableVision() {
    visionEnabled = true;
  }

  public void disableVision() {
    visionEnabled = false;
  }

  public Pose2d getEstimatedPosition() {
    return estimator.getEstimatedPosition();
  }

  public Rotation2d getEstimatedHeading() {
    return estimator.getEstimatedPosition().getRotation();
  }

  public void resetPosition(Rotation2d heading, SwerveModulePosition[] modulePositions, Pose2d pose) {
    estimator.resetPosition(heading, modulePositions, pose);
  }

  @Override
  public void periodic() {
    estimator.updateWithTime(RobotController.getFPGATime() / 1e+6, headingSupplier.get(), modulePositionsSupplier.get());

    if (!visionEnabled) return;
    for (var camera : cameras) {
      if (!camera.isEnabled()) continue;
      // Set the heading of the camera if it is a LimelightOdometryCamera
      if (camera instanceof LimelightOdometryCamera) ((LimelightOdometryCamera)camera).setHeading(this.getEstimatedHeading().getDegrees());
      Optional<VisionOdometryPoseEstimate> estimate = camera.getEstimate();
      if (estimate.isEmpty()) continue;
      estimator.addVisionMeasurement(estimate.get().pose, estimate.get().timestamp, estimate.get().stdDev);
    }
  }
}
