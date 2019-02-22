package frc.robot;

public interface SubsystemConstants {
    interface SPS {

        final double INIT_X = 0;
        final double INIT_Y = 0;
        final double INIT_YAW = 0;
        final double ROBOT_WIDTH = 1;

        public class Location {
            public double x, y, yaw;

            public Location(double x, double y, double yaw) {
                this.x = x;
                this.y = y;
                this.yaw = yaw;
            }
        }

        Location locations[] = { new Location(0, 0, 0), // Name of location
                new Location(1, 1, 100) // Name of location
        };

        Runnable resetSensors = new Runnable() {
            @Override
            public void run() {
                SubsystemComponents.Drivetrain.leftEncoder.reset();
                SubsystemComponents.Drivetrain.rightEncoder.reset();
                SubsystemComponents.DIO.gyro.reset();
            }
        };

    }
}