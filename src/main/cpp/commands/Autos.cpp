// #include "commands/Autos.h"
// #include <pathplanner/lib/auto/SwerveAutoBuilder.h>
// #include <pathplanner/lib/PathPlanner.h>

// using namespace pathplanner;
// using namespace autos;

// std::vector<PathPlannerTrajectory> pathGroup = PathPlanner::loadPathGroup("FullAuto", {PathConstraints(4_mps, 3_mps_sq)});

// std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;

// SwerveAutoBuilder autoBuilder(
//     []() { return m_drive.GetPose(); }, // Function to supply current robot pose
//     [](auto initPose) { m_drive.ResetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
//     PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
//     PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
//     [](auto speeds) { m_drive.Drive(speeds); }, // Output function that accepts field relative ChassisSpeeds
//     eventMap, // Our event map
//     { &m_drive }, // Drive requirements, usually just a single drive subsystem
//     true // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
// );

// frc2::CommandPtr fullAuto = autoBuilder.fullAuto(pathGroup);