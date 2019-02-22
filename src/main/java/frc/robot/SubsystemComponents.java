package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public interface SubsystemComponents {
    interface Drivetrain {
        SpeedController leftSP = new SpeedControllerGroup(new WPI_TalonSRX(0), new WPI_TalonSRX(1));
        SpeedController rightSP = new SpeedControllerGroup(new WPI_TalonSRX(4), new WPI_TalonSRX(5));
        Encoder leftEncoder = new Encoder(0, 1);
        Encoder rightEncoder = new Encoder(2, 3);
    }
    interface DIO {
        ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    }
}