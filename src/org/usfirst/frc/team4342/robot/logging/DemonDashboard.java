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
					
					SmartDashboard.putNumber("Ultra-LH", oi.LeftHeight.getRangeInches());
					SmartDashboard.putNumber("Ultra-RH", oi.RightHeight.getRangeInches());
					SmartDashboard.putNumber("Ultra-LD", oi.LeftDistance.getRangeInches());
					SmartDashboard.putNumber("Ultra-HD", oi.RightDistance.getRangeInches());
					
					SmartDashboard.putNumber("Enc-LD", oi.LeftDrive.getDistance());
					SmartDashboard.putNumber("Enc-RD", oi.RightDrive.getDistance());
					SmartDashboard.putNumber("Enc-ED", oi.EleEnc.getDistance());
					
					SmartDashboard.putBoolean("Elev-BLS", oi.EleLS.get());
					
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
