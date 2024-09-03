package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.BlueShift.control.LazyCANSparkMax;

public class Shooter extends SubsystemBase {
    LazyCANSparkMax leftMotor;
    LazyCANSparkMax rightMotor;

    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;

    boolean state;

    public Shooter() {
        this.leftMotor = new LazyCANSparkMax(Constants.Shooter.kLeftShooterMotorID, MotorType.kBrushless);
        this.leftEncoder = leftMotor.getEncoder();

        this.rightMotor = new LazyCANSparkMax(Constants.Shooter.kRightShooterMotorID, MotorType.kBrushless);
        this.rightMotor.setInverted(true);
        this.rightEncoder = rightMotor.getEncoder();

    }

    public void setLeftMotor(double speed) {
        leftMotor.set(speed);
    }

    public void setRightMotor(double speed) {
        rightMotor.set(speed);
    }

    public void set(double leftSpeed, double rightSpeed) {
        this.state = true;
        setLeftMotor(leftSpeed);
        setRightMotor(rightSpeed);
    }

    public void set(double speed) {
        set(speed, speed);
    }

    public void shootSpeaker() {
        this.set(Constants.Shooter.kShooterSpeakerLeftSpeed, Constants.Shooter.kShooterSpeakerRightSpeed);
    }

    public double getLeftMotorRpm() {
        return leftEncoder.getVelocity();
    }

    public double getRightMotorRpm() {
        return rightEncoder.getVelocity();
    }

    public void stop() {
        this.state = false;
        set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Shooter/IsShooting", state);
        SmartDashboard.putNumber("Shooter/LeftMotorRPM", getLeftMotorRpm());
        SmartDashboard.putNumber("Shooter/RightMotorRPM", getRightMotorRpm());
    }
}
