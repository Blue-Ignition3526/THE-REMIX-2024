package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.HashMap;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Field;
import frc.robot.commands.Climbers.Both.BothClimbersDown;
import frc.robot.commands.Climbers.Both.BothClimbersHome;
import frc.robot.commands.Climbers.Both.BothClimbersUp;
import frc.robot.commands.Intake.IntakeOut;
import frc.robot.commands.Intake.LifterAmp;
import frc.robot.commands.Intake.LifterFloor;
import frc.robot.commands.Intake.LifterShooter;
import frc.robot.commands.Intake.PickUpPiece;
import frc.robot.commands.Intake.ShootAmp;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.SpinShooter;
import frc.robot.commands.SwerveDrive.DriveSwerve;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IntakeLifter;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.Shooter;
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
  // * Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // * Gyro
  private final Gyro m_gyro;

  // *  Swerve Modules
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

  // * Swerve Drive
  private final SwerveDrive m_swerveDrive;

  // * Odometry
  private final PhotonCamera leftCam = new PhotonCamera("Arducam_Left");
  private final PhotonCamera rightCam = new PhotonCamera("Arducam_Right");

  private final OdometryCamera[] cameras = new OdometryCamera[] {
    new LimelightOdometryCamera("limelight-threeg", true, VisionOdometryFilters::limelightFilter),
    new LimelightOdometryCamera("limelight-three", false, VisionOdometryFilters::noFilter),
    new LimelightOdometryCamera("limelight-twoplus", true, VisionOdometryFilters::limelightFilter),

    new PhotonOdometryCamera(leftCam, Constants.Vision.Arducam_Left.kRobotToCamera, true, VisionOdometryFilters::noFilter),
    new PhotonOdometryCamera(rightCam, Constants.Vision.Arducam_Right.kRobotToCamera, true, VisionOdometryFilters::noFilter)
  };

  private final FilteredSwerveDrivePoseEstimator poseEstimator;

  // * Climbers
  private final Climber m_leftClimber;
  private final Climber m_rightClimber;

  // * Intake
  private final IntakeRollers m_intakeRollers;
  private final IntakeLifter m_intakeLifter;

  // * Shooter
  private final Shooter m_shooter;

  // * Autochooser
  private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {
    this.m_gyro = new Gyro(new GyroIOPigeon(Constants.SwerveDrive.kGyroDevice));

    this.m_frontLeft = new SwerveModule(Constants.SwerveDrive.SwerveModules.kFrontLeftOptions);
    this.m_frontRight = new SwerveModule(Constants.SwerveDrive.SwerveModules.kFrontRightOptions);
    this.m_backLeft = new SwerveModule(Constants.SwerveDrive.SwerveModules.kBackLeftOptions);
    this.m_backRight = new SwerveModule(Constants.SwerveDrive.SwerveModules.kBackRightOptions);

    this.m_swerveDrive = new SwerveDrive(m_frontLeft, m_frontRight, m_backLeft, m_backRight, m_gyro);

    this.poseEstimator = new FilteredSwerveDrivePoseEstimator(
      Constants.SwerveDrive.PhysicalModel.kDriveKinematics,
      m_swerveDrive::getModulePositions,
      m_swerveDrive::getHeading,
      cameras,
      new Pose2d(),
      VecBuilder.fill(0, 0, 0),
      VecBuilder.fill(0, 0, 0),
      true
    );

    this.m_leftClimber = new Climber(Constants.Climber.kLeftClimberMotorID, "LeftClimber");
    this.m_rightClimber = new Climber(Constants.Climber.kRightClimberMotorID, "RightClimber");

    this.m_intakeLifter =  new IntakeLifter();
    this.m_intakeRollers = new IntakeRollers();

    this.m_shooter = new Shooter();

    // Configure Auto Builder
    AutoBuilder.configureHolonomic(
        poseEstimator::getEstimatedPosition,
        poseEstimator::resetPosition,
        m_swerveDrive::getRobotRelativeChassisSpeeds,
        m_swerveDrive::driveRobotRelative, 
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
        m_swerveDrive
    );

    NamedCommands.registerCommands(new HashMap<String, Command>(){{
      put("PickupPiece", new PickUpPiece(m_intakeRollers, m_intakeLifter));
      put("Shoot", new Shoot(m_shooter, m_intakeRollers));
    }});

    this.m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoChooser", m_autoChooser);
    SmartDashboard.putData("poseEstimator/setVisionPose", new InstantCommand(poseEstimator::setvisionPose));
    // SmartDashboard.putData("poseEstimator/resetPose", new InstantCommand(poseEstimator.));
    SmartDashboard.putData("SwerveDrive/ResetTurningEncoders", new InstantCommand(m_swerveDrive::resetTurningEncoders));

    configureBindings();
  }

  private void configureBindings() {
    // Drivetrain
    this.m_swerveDrive.setDefaultCommand(new DriveSwerve(
        m_swerveDrive,
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> -controller.getRightX(),
        () -> true,
        () -> false
      )
    );

    controller.rightStick().onTrue(new InstantCommand(() -> this.m_swerveDrive.zeroHeading()));

    // Climbers
    controller.povUp().whileTrue(new BothClimbersUp(this.m_leftClimber, this.m_rightClimber));
    controller.povDown().whileTrue(new BothClimbersDown(this.m_leftClimber, this.m_rightClimber));

    // Intake
    controller.leftTrigger().whileTrue(new IntakeOut(this.m_intakeRollers));
  
    controller.povLeft().whileTrue(new LifterAmp(m_intakeLifter));
    controller.povLeft().onFalse(new ShootAmp(this.m_intakeRollers, this.m_intakeLifter));

    controller.rightBumper().onTrue(new LifterFloor(this.m_intakeLifter));
    controller.leftBumper().onTrue(new LifterShooter(this.m_intakeLifter));

    controller.a().toggleOnTrue(new PickUpPiece(this.m_intakeRollers, this.m_intakeLifter));

    // Shooter
    controller.rightTrigger().whileTrue(new SpinShooter(this.m_shooter));
    controller.rightTrigger().onFalse(new Shoot(this.m_shooter, this.m_intakeRollers));
  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      new InstantCommand(poseEstimator::setvisionPose),
      m_autoChooser.getSelected()
    );
  }

  public Command getTeleopCommand() {
    return Commands.print("No teleop command configured");
  }
}
