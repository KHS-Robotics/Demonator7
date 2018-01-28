package org.usfirst.frc.team4342.robot.logging;

import org.usfirst.frc.team4342.robot.OI;

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
			
			while(running)
			{
				try
				{
					// Get values from oi object here
					SmartDashboard.putNumber("NavX-Angle", oi.NavX.getAngle());
					SmartDashboard.putNumber("NavX-Yaw", oi.NavX.getYaw());
					SmartDashboard.putNumber("NavX-Roll", oi.NavX.getRoll());
					SmartDashboard.putNumber("NavX-Pitch", oi.NavX.getPitch());
					
					SmartDashboard.putNumber("Enc-ED", oi.Elevator.getDistance());
					SmartDashboard.putBoolean("Elev-AtBottom", oi.Elevator.isAtBottom());
					
					SmartDashboard.putNumber("Drive-LD", oi.Drive.getLeftDistance());
					SmartDashboard.putNumber("Drive-RD", oi.Drive.getRightDistance());
					SmartDashboard.putNumber("Drive-Heading", oi.Drive.getHeading());
					
					Thread.sleep(50);
				}
				catch(Exception ex)
				{
					Logger.error("DemonDashboard crashed!", ex);
					DemonDashboard.stop();
				}
			}
			
			SmartDashboard.putBoolean("DemonDashboard", false);
		}
	}
}
