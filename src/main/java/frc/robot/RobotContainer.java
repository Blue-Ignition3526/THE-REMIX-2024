package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Climbers.Both.BothClimbersDown;
import frc.robot.commands.Climbers.Both.BothClimbersHome;
import frc.robot.commands.Climbers.Both.BothClimbersUp;
import frc.robot.commands.SwerveDrive.DriveSwerve;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Gyro.Gyro;
import frc.robot.subsystems.Gyro.GyroIOPigeon;
import lib.BlueShift.odometry.swerve.FilteredSwerveDrivePoseEstimator;
import lib.BlueShift.odometry.vision.OdometryCamera;
import lib.BlueShift.odometry.vision.camera.LimelightOdometryCamera;
import lib.BlueShift.odometry.vision.camera.PhotonOdometryCamera;
import lib.BlueShift.odometry.vision.camera.VisionOdometryFilters;

public class RobotContainer {
  // * Gyro
  private final Gyro m_gyro = new Gyro(new GyroIOPigeon(Constants.SwerveDrive.kGyroDevice));

  // *  Swerve Modules
  private final SwerveModule m_frontLeft = new SwerveModule(Constants.SwerveDrive.SwerveModules.kFrontLeftOptions);
  private final SwerveModule m_frontRight = new SwerveModule(Constants.SwerveDrive.SwerveModules.kFrontRightOptions);
  private final SwerveModule m_backLeft = new SwerveModule(Constants.SwerveDrive.SwerveModules.kBackLeftOptions);
  private final SwerveModule m_backRight = new SwerveModule(Constants.SwerveDrive.SwerveModules.kBackRightOptions);

  // * Swerve Drive
  private final SwerveDrive m_swerveDrive = new SwerveDrive(m_frontLeft, m_frontRight, m_backLeft, m_backRight, m_gyro);

  // * Odometry
  private final PhotonCamera leftCam = new PhotonCamera("Arducam_Left");
  private final PhotonCamera rightCam = new PhotonCamera("Arducam_Right");

  private final OdometryCamera[] cameras = new OdometryCamera[] {
    new PhotonOdometryCamera(leftCam, null, true, VisionOdometryFilters::noFilter),
    new PhotonOdometryCamera(rightCam, null, true, VisionOdometryFilters::noFilter),

    new LimelightOdometryCamera("limelight-threeg", true, VisionOdometryFilters::noFilter),
    new LimelightOdometryCamera("limelight-three", false, VisionOdometryFilters::noFilter),
    new LimelightOdometryCamera("limelight-twoplus", true, VisionOdometryFilters::noFilter)
  };

  private final FilteredSwerveDrivePoseEstimator poseEstimator = new FilteredSwerveDrivePoseEstimator(
    Constants.SwerveDrive.PhysicalModel.kDriveKinematics,
    m_swerveDrive::getModulePositions,
    m_swerveDrive::getHeading,
    cameras,
    new Pose2d(),
    VecBuilder.fill(0, 0, 0),
    VecBuilder.fill(0, 0, 0),
    true
  );

  // * Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // * Climbers
  private final Climber m_leftClimber = new Climber(Constants.Climber.kLeftClimberMotorID, "LeftClimber");
  private final Climber m_rightClimber = new Climber(Constants.Climber.kRightClimberMotorID, "RightClimber");

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    this.m_swerveDrive.setDefaultCommand(new DriveSwerve(
        m_swerveDrive,
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> -controller.getRightX(),
        () -> true,
        () -> false
      )
    );

    // Climbers
    controller.povUp().whileTrue(new BothClimbersUp(this.m_leftClimber, this.m_rightClimber));
    controller.povDown().whileTrue(new BothClimbersDown(this.m_leftClimber, this.m_rightClimber));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public Command getTeleopCommand() {
    return new BothClimbersHome(m_leftClimber, m_rightClimber);
  }
}
