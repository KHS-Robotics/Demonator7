package org.usfirst.frc.team4342.robot.logging;

import org.usfirst.frc.team4342.robot.OI;

import edu.wpi.first.wpilibj.command.Scheduler;
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
					SmartDashboard.putNumber("Drive-GetHeading", oi.Drive.getHeading());

					SmartDashboard.putData(Scheduler.getInstance());

					SmartDashboard.putData("Drive", oi.Drive);
					SmartDashboard.putData("Elevator", oi.Elevator);
					SmartDashboard.putData("Intake", oi.Intake);
					SmartDashboard.putData("Climber", oi.Climber);
					
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
