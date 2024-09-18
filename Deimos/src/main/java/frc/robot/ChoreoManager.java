package frc.robot;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.AftershockChoreo.AftershockChoreo;
import frc.lib.AftershockChoreo.AftershockChoreoTrajectory;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class ChoreoManager {
    private static ChoreoManager mInstance;

    private DriveSubsystem mDriveSubsystem;
    private Field2d mField;
    private AftershockChoreoTrajectory traj;

    private ChoreoManager() {
        mDriveSubsystem = DriveSubsystem.getInstance();
        mField = new Field2d();
        traj = AftershockChoreo.getTrajectory("danke");

        mField.getObject("traj").setPoses(
                traj.getInitialPose(), traj.getFinalPose());
        mField.getObject("trajPoses").setPoses(
                traj.getPoses());

        SmartDashboard.putData(mField);
    }

    /**
     * Must be run <b>continuously</b> in robot periodic
     */
    public void updatePose() {
        mField.setRobotPose(mDriveSubsystem.getPose());
    }

    /**
     * Return this command in RobotContainer getAutonomousCommand
     * 
     * @return Choreo swerve trajectories command
     */
    public Command getChoreoAutonomousCommand() {
        traj = AftershockChoreo.getTrajectory("danke");

        mField.getObject("traj").setPoses(
                traj.getInitialPose(), traj.getFinalPose());
        mField.getObject("trajPoses").setPoses(
                traj.getPoses());

        var thetaController = new PIDController(DriveConstants.kPTheta, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        mDriveSubsystem.resetOdometry(traj.getInitialPose());

        Command swerveCommand = AftershockChoreo.choreoSwerveCommand(
                traj, // Choreo trajectory from above
                mDriveSubsystem::getPose, // A function that returns the current field-relative pose of
                // the robot: your
                // wheel or vision odometry
                new PIDController(DriveConstants.kPX, 0.0, 0.0), // PIDController for field-relative
                // X
                // translation (input: X error in
                // meters,
                // output: m/s).
                new PIDController(DriveConstants.kPY, 0.0, 0.0), // PIDController for field-relative
                // Y
                // translation (input: Y error in
                // meters,
                // output: m/s).
                thetaController, // PID constants to correct for rotation
                // error
                (ChassisSpeeds speeds) -> mDriveSubsystem.drive(speeds),
                () -> true, // Whether or not to mirror the path based on alliance (this assumes the path is
                      // created for the blue alliance)
                mDriveSubsystem // The subsystem(s) to require, typically your drive subsystem only
        );

        return Commands.sequence(
                Commands.runOnce(() -> mDriveSubsystem.resetOdometry(traj.getInitialPose())),
                swerveCommand,
                mDriveSubsystem.run(() -> mDriveSubsystem.drive(new ChassisSpeeds())));
    }

    public synchronized static ChoreoManager getInstance() {
        if (mInstance == null) {
            mInstance = new ChoreoManager();
        }
        return mInstance;
    }
}