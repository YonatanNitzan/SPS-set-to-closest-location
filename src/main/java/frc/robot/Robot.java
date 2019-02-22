/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.Supplier;

import com.spikes2212.dashboard.ConstantHandler;
import com.spikes2212.dashboard.DashBoardController;
import com.spikes2212.genericsubsystems.drivetrains.TankDrivetrain;
import com.spikes2212.genericsubsystems.drivetrains.commands.DriveArcade;
import com.spikes2212.utils.RunnableCommand;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SubsystemConstants.SPS.Location;
import odometry.OdometryHandler;
import odometry.OdometryUnit;
import odometry.SetPosition;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static OI oi;

  public static DashBoardController dbc = new DashBoardController();

  public static TankDrivetrain drivetrain;

  public static OdometryUnit unit = new OdometryUnit(SubsystemComponents.Drivetrain.leftEncoder::getDistance,
      SubsystemComponents.Drivetrain.rightEncoder::getDistance, SubsystemConstants.SPS.ROBOT_WIDTH, SubsystemComponents.DIO.gyro::getAngle);
  public static OdometryHandler handler = new OdometryHandler(unit, SubsystemConstants.SPS.INIT_X,
      SubsystemConstants.SPS.INIT_Y, SubsystemConstants.SPS.INIT_YAW);

  public static double distancePerTick = 15.3 * Math.PI / 36000;

  public static Supplier<Double> customX = ConstantHandler.addConstantDouble("Custom x", 1);
  public static Supplier<Double> customY = ConstantHandler.addConstantDouble("Custom y", 1);
  public static Supplier<Double> customYaw = ConstantHandler.addConstantDouble("Custom yaw", 1);
  
  @Override
  public void robotInit() {
    drivetrain = new TankDrivetrain(SubsystemComponents.Drivetrain.leftSP::set,
        SubsystemComponents.Drivetrain.rightSP::set);
    SubsystemComponents.Drivetrain.leftEncoder.setDistancePerPulse(distancePerTick);
    SubsystemComponents.Drivetrain.rightEncoder.setDistancePerPulse(distancePerTick);

    SubsystemConstants.SPS.resetSensors.run();

    dbc.addNumber("Robot x", handler::getX);
    dbc.addNumber("Robot y", handler::getY);
    dbc.addNumber("Robot angle", handler::getYaw);

    oi = new OI();

    drivetrain.setDefaultCommand(new DriveArcade(drivetrain, oi.right::getX, () -> -oi.left.getY()));

    SmartDashboard.putData("Set middle", new SetPosition(4, 8, 0, SubsystemConstants.SPS.resetSensors, handler));
    SmartDashboard.putData("Set custom location", new SetPosition(customX.get(), customY.get(), customYaw.get(), SubsystemConstants.SPS.resetSensors, handler));
    SmartDashboard.putData("Reset Sensors", new RunnableCommand(SubsystemConstants.SPS.resetSensors));
    SmartDashboard.putData("wait for 5 seconds", new WaitCommand(5000000));
    SmartDashboard.putData("Set nearest location", new RunnableCommand(new Runnable(){
      
      private static final int yawError = 1;
      private static final int distError = 1;

      @Override
      public void run() {
        Location nearest = SubsystemConstants.SPS.locations[0];
        
        for (Location l : SubsystemConstants.SPS.locations) {
          if (Math.abs(l.yaw - handler.getYaw()) <= yawError)
            if(calcDistFromRobot(l) <= distError && calcDistFromRobot(l) <= calcDistFromRobot(nearest))
              nearest = l;
        }

        SubsystemConstants.SPS.resetSensors.run();
        handler.setPosition(nearest.x, nearest.y, nearest.yaw);
      }

      private double calcDistFromRobot(Location l) {
        return Math.sqrt(Math.pow(l.x - handler.getX(), 2) + Math.pow(l.y - handler.getY(), 2));
      }
    }, true));
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    handler.calculate();
    dbc.update();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
