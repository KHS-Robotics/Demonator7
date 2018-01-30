package org.usfirst.frc.team4342.robot.logging;

import org.usfirst.frc.team4342.robot.OI;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
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
					// NavX
					SmartDashboard.putNumber("NavX-Angle", oi.NavX.getAngle());
					SmartDashboard.putNumber("NavX-Yaw", oi.NavX.getYaw());
					SmartDashboard.putNumber("NavX-Roll", oi.NavX.getRoll());
					SmartDashboard.putNumber("NavX-Pitch", oi.NavX.getPitch());
					
					// Elevator
					SmartDashboard.putNumber("Elev-ED", oi.Elevator.getDistance());
					SmartDashboard.putBoolean("Elev-AtBottom", oi.Elevator.isAtBottom());
					SmartDashboard.putBoolean("Elev-AtSetpoint", oi.Elevator.isAtSetpoint());
					
					// Drive
					SmartDashboard.putNumber("Drive-Heading", oi.Drive.getHeading());
					SmartDashboard.putNumberArray("Drive-AllDists", oi.Drive.getAllDistances());
					SmartDashboard.putBoolean("Drive-OnTarget", oi.Drive.onTarget());
					
					// Power Distribution Panel
					SmartDashboard.putData("PDP", oi.PDP);

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
	}
}
