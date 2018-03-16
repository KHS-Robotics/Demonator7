package org.usfirst.frc.team4342.robot.logging;

import org.usfirst.frc.team4342.robot.OI;
import org.usfirst.frc.team4342.robot.subsystems.SwerveDrive.SwerveModule;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class to put data to the SmartDashboard on a separate thread
 */
public class DemonDashboard 
{
	private DemonDashboard() {}
	
	private static boolean running;
	
	/**
	 * Starts the <code>DemonDashboard</code> on a new thread.
	 */
	public static void start()
	{
		if(running)
			return;
		
		Logger.info("Starting DemonDashboard...");
		new DemonDashboardThread().start();
		
		running = true;
	}
	
	/**
	 * Stops the <code>DemonDashboard</code>
	 */
	public static void stop()
	{
		running = false;
	}
	
	/**
	 * The magic behind this class...
	 */
	private static class DemonDashboardThread extends Thread implements Runnable
	{
		private static final OI oi = OI.getInstance();
		
		/**
		 * Puts data to the SmartDashboard every 50ms. The data is retrieved from OI
		 * @see org.usfirst.frc.team4342.robot.OI
		 */
		@Override
		public void run()
		{
			SmartDashboard.putBoolean("DemonDashboard", true);
			SmartDashboard.putBoolean("DemonDashboard-Crash", false);
			
			while(running)
			{
				try
				{
                    if(oi.Drive != null) {
						SmartDashboard.putNumber("Drive-Heading", oi.Drive.getHeading());
                        SmartDashboard.putNumberArray("Drive-Encs-FR-FL-RR-RL", oi.Drive.getAllDistances());
                        
                        putSwerveModuleData(oi.Drive.fr, "FR");
                        putSwerveModuleData(oi.Drive.fl, "FL");
                        putSwerveModuleData(oi.Drive.rr, "RR");
                        putSwerveModuleData(oi.Drive.rl, "RL");
                    }
                    
                    if(oi.Elevator != null) {
                        SmartDashboard.putNumber("Elev-Dist", oi.Elevator.getPosition());
                        SmartDashboard.putBoolean("Elev-LS", oi.Elevator.isAtBottom());
                    }
                    
                    if(oi.Intake != null) {
//                    	SmartDashboard.putNumber("Intake-Ultra", oi.Intake.getUltraRange());
//                    	SmartDashboard.putBoolean("Intake-HasCube", oi.Intake.hasCube());
                    }

					Thread.sleep(50);
				}
				catch(Exception ex)
				{
					Logger.error("DemonDashboard crashed!", ex);
					SmartDashboard.putBoolean("DemonDashboard-Crash", true);
					DemonDashboard.stop();
				}
			}
			
			SmartDashboard.putBoolean("DemonDashboard", false);
        }
        
        /**
         * Puts Swerve Module data to the SmartDashboard
         * @param m the swerve module
         * @param name the name of the module (e.g., "FR" for front right)
         */
        private void putSwerveModuleData(SwerveModule m, String name) {
            SmartDashboard.putNumber(name + "-Dist", m.getDistance());
            SmartDashboard.putNumber(name + "-Angle", m.getAngle());
            SmartDashboard.putNumber(name + "-Votage", m.getVoltage());
            SmartDashboard.putNumber(name + "-Setpoint", m.getSetpoint());
            SmartDashboard.putBoolean(name + "-Flipping", m.isFlipDrive());
        }
	}
}