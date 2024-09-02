package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.BlueShift.control.LazyCANSparkMax;

public class Climber extends SubsystemBase {
    LazyCANSparkMax climberMotor;
    String name;

    public Climber(int motorID, String name) {
        this.name = name;
        this.climberMotor = new LazyCANSparkMax(motorID, MotorType.kBrushless);
    }

    public void set(double speed) {
        climberMotor.set(speed);
    }

    public void setClimberUp() {
        climberMotor.set(Constants.Climber.kClimberUpSpeed);
    }

    public void setClimberDown() {
        climberMotor.set(Constants.Climber.kClimberDownSpeed);
    }

    public void stop() {
        climberMotor.set(0);
    }

    public double getCurrent() {
        return climberMotor.getOutputCurrent();
    }
}
