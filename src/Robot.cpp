#include <memory>
#include <vector>
#include <thread>

#include <Commands/Command.h>
#include <Commands/Scheduler.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <CameraServer.h>
#include <Joystick.h>

#include "Commands/ExampleCommand.h"
#include "CommandBase.h"

#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>


// for access from static functions. ENSURE THREAD SAFETY!
class Robot;
Robot* theRobot;

class Robot: public frc::IterativeRobot {
public:
	
	frc::Joystick* stick;
	
	
	
	void RobotInit() override {
		theRobot = this;
		
		chooser.AddDefault("Default Auto", new ExampleCommand());
		// chooser.AddObject("My Auto", new MyAutoCommand());
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
		
		CameraServer::GetInstance()->StartAutomaticCapture(0);
		
		stick = new frc::Joystick(0);
		
		std::thread cameraThreadObj(cameraThread);
		cameraThreadObj.detach();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	void DisabledInit() override {

	}

	void DisabledPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString code to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the if-else structure below with additional strings & commands.
	 */
	void AutonomousInit() override {
		/* std::string autoSelected = frc::SmartDashboard::GetString("Auto Selector", "Default");
		if (autoSelected == "My Auto") {
			autonomousCommand.reset(new MyAutoCommand());
		}
		else {
			autonomousCommand.reset(new ExampleCommand());
		} */

		autonomousCommand.reset(chooser.GetSelected());

		if (autonomousCommand.get() != nullptr) {
			autonomousCommand->Start();
		}
	}

	void AutonomousPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}

	void TeleopInit() override {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != nullptr) {
			autonomousCommand->Cancel();
		}
	}
	
	
	static void cameraThread() {
		CameraServer::GetInstance()->StartAutomaticCapture(1);
		CameraServer::GetInstance()->StartAutomaticCapture(0);
		
		cs::CvSink frontSink = CameraServer::GetInstance()->GetVideo("cam0");
		cs::CvSink backSink = CameraServer::GetInstance()->GetVideo("cam1");
		
		cs::CvSource output = CameraServer::GetInstance()->PutVideo("Camera", 640, 480);
		
		cv::Mat mat;
		while(true) {
		// access once for thread safety
			bool usingFront = theRobot->usingFrontCamera;
			if (usingFront) pushFrame(frontSink, output, mat);
			else pushFrame(backSink, output, mat);
		}
	}
	static void pushFrame(cs::CvSink sink, cs::CvSource output, cv::Mat reusableMat) {
		if (sink.GrabFrame(reusableMat) == 0) {
			// Send the output the error.
			output.NotifyError(sink.GetError());
			// skip the rest of the current iteration
			return;
		}
		
		output.PutFrame(reusableMat);
	}
	
	

	// system for button handlers.
	// call from TeleopPeriodic(). If true, call function associated with button.
	
	std::vector<bool> buttons = std::vector<bool>(12);
	bool wasButtonJustPressed(int button) {
		bool isPressed = stick->GetRawButton(button);
		
		if (isPressed) {
			if (!buttons[button - 1]) {
				buttons[button - 1] = true;
				return true;
			}
		}
		else buttons[button - 1] = false;
		return false;
	}
	
	void TeleopPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	
		if (wasButtonJustPressed(3)) toggleCamera();
	}

	void TestPeriodic() override {
		frc::LiveWindow::GetInstance()->Run();
	}
	
	
	
	bool usingFrontCamera = true;
	void toggleCamera() {
		usingFrontCamera = !usingFrontCamera;
		
		if (usingFrontCamera) CameraServer::GetInstance()->StartAutomaticCapture(0);
		else CameraServer::GetInstance()->StartAutomaticCapture(1);
	}

private:
	std::unique_ptr<frc::Command> autonomousCommand;
	frc::SendableChooser<frc::Command*> chooser;
};



START_ROBOT_CLASS(Robot)
