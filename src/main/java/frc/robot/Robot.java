// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.dacubeking.AutoBuilder.robot.reflection.ClassInformationSender;
import com.dacubeking.AutoBuilder.robot.robotinterface.AutonomousContainer;
import com.dacubeking.AutoBuilder.robot.robotinterface.CommandTranslator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Autons.CatzAutonomous;
import frc.Autons.CatzRobotTracker;
import frc.Autons.DriveStraightPathGenerator;
import frc.Mechanisms.CatzDrivetrain;
import frc.Utils.CatzMathUtils;

import frc.DataLogger.*;

@SuppressWarnings("unused")
public class Robot extends TimedRobot {
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private final SendableChooser<String> sideChooser = new SendableChooser<>();

  private XboxController xboxDrv;
  private CatzDrivetrain driveTrain;
  
  private double steerAngle = 0.0;
  private double drivePower = 0.0;
  private double turnPower = 0.0;
   
  private static DataCollection dataCollection;
  private static Timer currentTime;
  private ArrayList<CatzLog> dataArrayList;
  
  private final int XBOX_DRV_PORT = 0;

  private final CatzAutonomous autonDrive = CatzAutonomous.getAutonomousInstance();
  private final CatzRobotTracker robotTracker = CatzRobotTracker.getRobotTrackerInstance();

  @Override
  public void robotInit() {
      startThread();
      driveTrain = CatzDrivetrain.getDrivetrainInstance();
      xboxDrv = new XboxController(XBOX_DRV_PORT);

      dataCollection = new DataCollection();
      dataArrayList = new ArrayList<CatzLog>();
      dataCollection.dataCollectionInit(dataArrayList);

      currentTime = new Timer();

      robotTracker.resetPosition(new Pose2d());
      
      AutonomousContainer.getInstance().initialize(
                true,
                new CommandTranslator(
                        autonDrive::setAutoPath,
                        autonDrive::stopMovement,
                        autonDrive::setAutoRotation,
                        autonDrive::isFinished,
                        autonDrive::getAutoElapsedTime,
                        robotTracker::resetPosition,
                        true
                ),
                false,
                this
        );

      AutonomousContainer.getInstance().getAutonomousNames().forEach(name -> autoChooser.addOption(name, name));

      sideChooser.setDefaultOption("Blue", "blue");
      sideChooser.addOption("Red", "red");

      SmartDashboard.putData("Path Choices", autoChooser);
      SmartDashboard.putData("Red or Blue", sideChooser);
  }

  @Override
  public void robotPeriodic()
  {
    driveTrain.updateShuffleboard();

    CatzAutonomous.getAutonomousInstance().updateShuffleboard();

    autonDrive.smartDashboard();
    autonDrive.smartDashboard_DEBUG();
    
    //SmartDashboard.putNumber("NavX", navX.getAngle());
    //drivetrain.testAngle();
  }


  @Override
  public void teleopInit()
  {
    autonDrive.kill();
    currentTime.reset();
    currentTime.start();

    dataCollection.setLogDataID(dataCollection.LOG_ID_SWERVE_MODULE);
    dataCollection.startDataCollection();

    driveTrain.setBrakeMode();
  }

  @Override
  public void teleopPeriodic()
  {
    steerAngle = CatzMathUtils.calcJoystickAngle(xboxDrv.getLeftX(), xboxDrv.getLeftY());
    drivePower = CatzMathUtils.calcJoystickPower(xboxDrv.getLeftX(), xboxDrv.getLeftY());
    turnPower = xboxDrv.getRightX();

    if(drivePower >= 0.1)
    {
      if(Math.abs(turnPower) >= 0.1)
      {
        driveTrain.translateTurn(steerAngle, drivePower, turnPower);
      }
      else
      {
        driveTrain.drive(steerAngle, drivePower);
      }
      driveTrain.dataCollection();
    }
    else if(Math.abs(turnPower) >= 0.1)
    {
      driveTrain.rotateInPlace(turnPower);
      driveTrain.dataCollection();
    }
    else
    {
      driveTrain.setSteerPower(0.0);
      driveTrain.setDrivePower(0.0);
    }

  }


  @Override
  public void autonomousInit() {
    startThread();

    currentTime.reset();
    currentTime.start();

    dataCollection.setLogDataID(dataCollection.LOG_ID_SWERVE_MODULE);
    dataCollection.startDataCollection();
    String autoName = autoChooser.getSelected();

    AutonomousContainer.getInstance().runAutonomous(autoName, sideChooser.getSelected(), true);
  }

  @Override
  public void autonomousPeriodic(){
    driveTrain.dataCollection();

  }

  @Override
  public void disabledInit()
  {
    driveTrain.setCoastMode();
    currentTime.stop();
    driveTrain.setCoastMode();
    if(dataCollection.logDataValues == true)
    {
      dataCollection.stopDataCollection();

      try 
      {
        dataCollection.exportData(dataArrayList);
      } 
      catch (Exception e) 
      {
        e.printStackTrace();
      }
    }
  }

  @Override
  public void testInit(){
    driveTrain.initializeOffsets();
  }

  @Override
  public void simulationInit(){
    ClassInformationSender.updateReflectionInformation("frc");
  }

  private void startThread(){
    CatzRobotTracker.getRobotTrackerInstance().start();
    CatzAutonomous.getAutonomousInstance().start();
  }

  public static DataCollection getDataCollection(){
    return dataCollection;
  }
  
  public static double getCurrentTime(){
    return currentTime.get();
  }
}
