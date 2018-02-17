package org.usfirst.frc.team4342.robot;

/**
 * Constants such as Dimensions and PID
 */
public class Constants {
    /** Robot Length in inches */
    public static final double ROBOT_LENGTH = 27.5;
    /** Robot Width in inches */
    public static final double ROBOT_WIDTH = 32.5;

    /**
     * Elevator Constants
     */
    public static class Elevator {
        // TODO: Get PID values
        public static final double P = 0.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
    }

    /**
     * Drive Constants
     */
    public static class Drive {
        // Heading
        // TODO: Get PID values
        public static final double P = 0.0;
        public static final double I = 0.0;
        public static final double D = 0.0;

        // Swerve Pivot
        public static class PivotPID {
			public static final double FR_P = 1.2;
			public static final double FR_I = 0.01;
			public static final double FR_D = 0.005;

			public static final double FL_P = 1.2;
			public static final double FL_I = 0.01;
			public static final double FL_D = 0.005;

			public static final double RR_P = 1.2;
			public static final double RR_I = 0.01;
			public static final double RR_D = 0.005;

			public static final double RL_P = 1.2;
			public static final double RL_I = 0.01;
			public static final double RL_D = 0.005;
		}
    }
}
