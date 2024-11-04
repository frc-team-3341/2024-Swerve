package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.swerve.SwerveTeleopCMD;
import frc.robot.subsystems.swerve.SwerveDriveTrain;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleIOSparkMax;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;


public class RobotContainer {

  // ---------------------- START OF CONFIG SECTION --------------------------

  // Defines starting pose of robot
  // TODO - Please remove this in future if developing for AprilTags
  Pose2d startpose = new Pose2d(new Translation2d(0, 0), new Rotation2d());
  // add start pose if needed
  // ---------------------- END OF CONFIG SECTION --------------------------

  // Xbox + an additional one for PC use
  private final Joystick drivingXbox = new Joystick(0);
  // Chooser for testing swerveTeleopCMD commands
  private final SendableChooser<Command> teleopCommandChooser = new SendableChooser<>();

  SwerveModuleIO[] swerveMods = new SwerveModuleIO[4];
  private SwerveDriveTrain swerveDriveTrain = new SwerveDriveTrain(startpose,
          Constants.SwerveModuleIOConfig.module0,
          Constants.SwerveModuleIOConfig.module1,
          Constants.SwerveModuleIOConfig.module2,
          Constants.SwerveModuleIOConfig.module3);

  private final SwerveTeleopCMD swerveTeleopCMD = new SwerveTeleopCMD(this.swerveDriveTrain, this.drivingXbox , Constants.currentRobot.allianceEnabled);

  // Empty testing commands (not used if not needed)
  //private TestFourModules allFour;
  // Empty Auto object

  // Empty CrabDrive object
  //private CrabDrive crabDrive;

  // Field centric toggle - true for field centric, false for robot centric
  private boolean fieldCentricToggle = true;


  public RobotContainer() {
    // Construct swerveDriveTrain subsystem with appropriate modules - DO NOT REMOVE THIS
    //this.constructSwerve();
    // Create swerveDriveTrain commands - DO NOT REMOVE THIS

    //this.createSwerveCommands();
    this.swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);

    // Construct all other things
    this.configureBindings();
  }

  private void constructSwerve() {
    Pose2d startpose = new Pose2d(new Translation2d(0, 0), new Rotation2d());
    if (Constants.currentRobot.dataLogEnabled) {
      // Data logging works on both real + simulated robot with all DriverStation
      // outputs!
      DataLogManager.start();
      DriverStation.startDataLog(DataLogManager.getLog(), false);
      SmartDashboard.putString("Data Log Folder: ", DataLogManager.getLogDir());
    }
      // Construct swerveDriveTrain modules with real motors
    for (int i = 0; i < swerveMods.length; i++) {
      swerveMods[i] = new SwerveModuleIOSparkMax(i, Constants.currentRobot.moduleCANIDs[i][0],
              Constants.currentRobot.moduleCANIDs[i][1], Constants.currentRobot.moduleCANIDs[i][2],
              Constants.currentRobot.moduleAngleOffsets[i], Constants.SwerveConstants.moduleInverts[i]);
    }
    this.swerveDriveTrain = new SwerveDriveTrain(startpose, this.swerveMods[0], this.swerveMods[1], this.swerveMods[2], this.swerveMods[3]);
  }

  private void createSwerveCommands() {

    //toggle FieldCentric
    if(this.drivingXbox.getRawButtonPressed(XboxController.Button.kX.value)){
      fieldCentricToggle = !fieldCentricToggle;
    }
    SmartDashboard.putBoolean("isFieldCentric", fieldCentricToggle);

      // Empty SwerveTeleop object
//    SwerveTeleopCMD swerveTeleopCMD = new SwerveTeleopCMD(this.swerveDriveTrain, -this.drivingXbox.getRawAxis(translationAxis),
//              -this.drivingXbox.getRawAxis(strafeAxis),
//              -this.drivingXbox.getRawAxis(rotationAxis),
//              this.drivingXbox.getRawAxis(XboxController.Axis.kRightTrigger.value),
//              fieldCentricToggle,
//              Constants.currentRobot.allianceEnabled);

    //SwerveTeleopCMD swerveTeleopCMD = new SwerveTeleopCMD(this.swerveDriveTrain, this.drivingXbox , Constants.currentRobot.allianceEnabled);




    //crabDrive = new CrabDrive(this.swerveDriveTrain,-this.drivingXbox.getX(),-this.drivingXbox.getY());
    //allFour = new TestFourModules(swerveDriveTrain, drivingXbox);
    
    teleopCommandChooser.addOption("Regular Teleop", swerveTeleopCMD);
    //teleopCommandChooser.addOption("Crab Teleop", crabDrive);
    //teleopCommandChooser.addOption("Module Test Command", allFour);
    teleopCommandChooser.setDefaultOption("Regular Teleop", swerveTeleopCMD);
   
    // autoPaths = new InitializeAutoPaths(swerveDriveTrain, shooter);
  
    SmartDashboard.putData(teleopCommandChooser);
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return null;
    
  }


  public void initCommandInTeleop() {
    swerveDriveTrain.setDefaultCommand(teleopCommandChooser.getSelected());
  }

  /**
   * Gets Robot.isReal() from RobotContainer (slow when calling every loop)
   * 
   * @return If simulated or not
   */
}