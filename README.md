<div align="center">
  <a href="https://github.com/subzero-robotics/2024-CommandRobot">
    <img src="https://raw.githubusercontent.com/SubZero-Robotics/2024-CommandRobot/main/robot.png" alt="Robot" width="300">
  </a>

<h3 align="center">2024 Command Robot</h3>

  <p align="center">
    Code for our 2024 season robot. Built with the WPILib command-based framework, based on the REV MAXSwerve C++ example.
    <br />
    <br />
    <a href="https://github.com/subzero-robotics/2024-CommandRobot/issues">Current Issues</a>
    ·
    <a href="https://www.thebluealliance.com/team/5690/2024">The Blue Alliance</a>
  </p>
</div>

## Table of contents

- [Table of contents](#table-of-contents)
- [About](#about)
- [Development Cycle](#development-cycle)
  - [Main](#main)
  - [Competition](#competition)
- [Subsystems](#subsystems)
- [CAN IDs](#can-ids)
- [Digital Input Ports](#digital-input-ports)
- [Network Map](#network-map)
- [Button Bindings](#button-bindings)
  - [Xbox Controller](#xbox-controller)
  - [Keypad](#keypad)
- [State](#state)
- [Commands](#commands)
- [Command Compositions](#command-compositions)
- [Factory Commands](#factory-commands)
- [Autos](#autos)
  - [Auto Factory](#auto-factory)
  - [Auto Chooser](#auto-chooser)
  - [Autos](#autos-1)
- [State Commands](#state-commands)
- [LED Commands](#led-commands)
- [Getting started](#getting-started)
  - [Prerequisites](#prerequisites)
- [Making changes](#making-changes)
  - [Cloning the repo](#cloning-the-repo)
  - [Issues](#issues)
  - [Branches](#branches)
  - [Adding commits](#adding-commits)
  - [Pushing commits](#pushing-commits)
  - [Pulling branches](#pulling-branches)
  - [Pull requests](#pull-requests)
- [Contact](#contact)

## About

This repository is a monorepo containing robot code for team 5690's 2024 robot capable of playing FRC Crescendo. It features a MAXSwerve drivebase, and is based on the MAXSwerve C++ template.

## Development Cycle

### Main

Main should **always** contain known, tested, and working code that has been throroughly verified by running it on the robot. Use other development branches to create new features and test them before pulling into `main` via a Pull Request (PR). `main` cannot have code pushed to it directly, meaning a PR is **mandatory**. Furthermore, to prevent merge conflicts all branches should be based on `main`.

Use the `main` branch for non-competition usage and practice.

### Competition

Before a competition, create a new branch based on `main` and lock it via a GitHub protection rule to prevent pushes. Only use this branch during a competition to avoid unnecessary or breaking deployments to the RIO.

> [!WARNING]
> ***Changes should only be made to this branch if there is a blocking error that must be fixed immediately!***

## Subsystems

Following the WPILib command based structure we have broken our robot up into a number of subsystems. They are listed below:

| Subsystem                                                 | Purpose                                                   |
| :-------------------------------------------------------- | :-------------------------------------------------------- |
| [Drive](src/main/include/subsystems/DriveSubsystem.h)     | Drives robot                                              |
| [Intake](src/main/include/subsystems/IntakeSubsystem.h)   | Activates the intake                                      |
| [Scoring](src/main/include/subsystems/ScoringSubsystem.h) | Scores Notes                                              |
| [Arm](src/main/include/subsystems/ArmSubsystem.h)         | Raises and lowers the amp arm, uses BaseSingleAxis        |
| [LED](src/main/include/subsystems/LedSubsystem.h)         | Wrapper for the ConnectorX moduledriver, reads from State |
| [Climb](src/main/include/subsystems/ClimbSubsystem.h)     | Climbs on the Stage                                       |
| [State](src/main/include/subsystems/StateSubsystem.h)     | Finite state machine for the robot during teleop          |

## CAN IDs

|   Purpose/Name    | CAN ID | Motor/Driver Type | PDH Port |
| :---------------: | :----: | :---------------: | :------: |
| Front Right Drive |   2    |     SparkMax      |    8     |
| Rear Right Drive  |   4    |     SparkMax      |    3     |
|  Rear Left Drive  |   6    |     SparkMax      |    16    |
| Front Left Drive  |   8    |     SparkMax      |    11    |
| Front Right Turn  |   1    |     SparkMax      |    7     |
|  Rear Right Turn  |   3    |     SparkMax      |    4     |
|  Rear Left Turn   |   5    |     SparkMax      |    15    |
|  Front Left Turn  |   7    |     SparkMax      |    12    |
|      Intake       |   20   |     SparkMax      |    17    |
|      Vector       |   22   |     SparkMax      |    5     |
|     Amp Lower     |   24   |     SparkFlex     |    19    |
|     Amp Upper     |   21   |     SparkFlex     |    2     |
|   Speaker Lower   |   25   |     SparkFlex     |    18    |
|   Speaker Upper   |   19   |     SparkFlex     |    0     |
|   Left Climber    |   10   |     SparkMax      |    9     |
|   Right Climber   |   11   |     SparkMax      |    10    |
|   First Pigeon    |   9    |        N/A        |    22    |
|        Arm        |   62   |     SparkMax      |    1     |

\* = Inverted

## Digital Input Ports

| Port  |            Device            |
| :---: | :--------------------------: |
|   1   |   Center Upper Beam Break    |
|   2   | Lower Podium Side Beam Break |
|   3   |   Lower Center Beam Break    |
|   4   |  Lower Amp Side Beam Break   |
|   5   |  Upper Amp Side Beam Break   |
|   6   | Upper Podium Side Beam Break |

## Network Map

| Device  |             Address              |
| :-----: | :------------------------------: |
| Gateway |            10.56.90.1            |
| Gateway | 10.56.90.129 (subject to change) |
|   RIO   |            10.56.90.2            |
| Laptop  |             Dynamic              |
|   LL3   |           10.56.90.11            |
|  LL2+   |           10.56.90.12            |
|   LL3   |           10.56.90.13            |

## Button Bindings

### Xbox Controller

|               Button               |                    Action                     |
| :--------------------------------: | :-------------------------------------------: |
|                 A                  |                   Score Amp                   |
|                 B                  |                    Intake                     |
|                 X                  |                 Score Podium                  |
|                 Y                  |                Score Subwoofer                |
|            Left Bumper             |      Send both climbers up at 100% speed      |
|            Right Bumper            |                   Downtake                    |
|            Left Trigger            |            Moves left climber down            |
|           Right Trigger            |           Moves right climber down            |
| Left Stick / Top Right Back Paddle |                     Feed                      |
| Right Stick / Top Left Back Paddle |                 Toggle Aimbot                 |
|  POV Up / Lower Left Back Paddle   | Extends climbers to absolute distance of 20in |
| POV Down / Lower Right Back Paddle |              Toggle Auto Scoring              |

### Keypad

| Raw Button Number |   Key    | Action                                                                                                       |
| :---------------: | :------: | :----------------------------------------------------------------------------------------------------------- |
|         0         |    /     | Go to approx location via on the fly PP. Dynamically swaps between Source and scoring based on note presence |
|         3         | NUM LOCK | Go to Source center via on the fly PP.                                                                       |
|         4         |    7     | Go to Amp and auto score via on the fly PP.                                                                  |
|         5         |    8     | Go to Podium and auto score via on the fly PP.                                                               |
|         6         |    9     | Go to Subwoofer and auto score via on the fly PP.                                                            |
|         7         |    4     | Automatically cycles between Source and Amp using top intake. Scores Amp 2x before scoring Subwoofer 1x      |
|         8         |    5     | Zeroes gyro.                                                                                                 |
|         9         |    6     | Extends climbers to absolute distance of 20in.                                                               |
|        10         |    1     | Drives to Stage Left and extends climbers (Amp side when on blue)                                            |
|        16         |    2     | Drives to Stage Center and extends climbers                                                                  |
|        17         |    3     | Drives to Stage Right and extends climbers (Source side when on blue)                                        |
|        18         |    0     | Automatically intakes note in view of camera before automatically scoring it at the Subwoofer.               |
|        19         |   Del    | Panic button; cancels any running state and returns to manual.                                               |

## State

We have 18 states that each execute a function to schedule a command based on the desired state. This allows a secondary operator to perform sweeping, fully-autonomous functions such as scoring, intaking, and more. Each state also has a check to ensure one action cannot lead to another if there's a conflict. Furthermore, the robot's LEDs will indicate the current state with changing colors and patterns. States can be chained through use of `RunStateDeferred` to switch commands internally.

| Name                         |                          Purpose                          |
| :--------------------------- | :-------------------------------------------------------: |
| Manual                       |                        Do nothing                         |
| ScoringSpeaker               |   Run to the podium location and score into the speaker   |
| ScoringAmp                   |                 Run to the amp and score                  |
| ScoringSubwoofer             |  Run to the speaker location and score into the speaker   |
| Loaded                       |             Note is loaded into the magazine              |
| Intaking                     |    Drive forward while intaking until note is present     |
| ClimbStage Left/Right/Center |       Go to the stage location, climb, then balance       |
| Source Left/Center/Right     |        Go to one of the three slots at the Source         |
| Funni                        |                        Do a funni                         |
| AutoSequenceSpeaker          |  Auto cycling sequence for the Speaker (Podium Scoring)   |
| AutoSequenceAmp              |             Auto cycling sequence for the Amp             |
| AutoSequenceSubwoofer        | Auto cycling sequence for the Speaker (Subwoofer Scoring) |
| EnemyWing                    | Drives to the enemy wing, approximate pathing for Source  |
| AllianceWing                 |    Drives to own wing, approximate pathing for Scoring    |

Notes:

- Every automatic action can be interrupted by any input on the first operator's controller, as well as a panic button on the keypad
- Every automatic action has a timeout
- When moving to a location, on-the-fly pathplanner runs first for the approximate location from anywhere before a premade path is executed for a more accurate alignment with the target pose
- Every automatic action completes by moving back into the Manual state

## Commands

State actions are composed of underlying commands that are also called by the first operator's gamepad. Some of these might be intaking, run scoring subsystem, or drive.

|                                 Command                                  |                                                 Purpose                                                 |
| :----------------------------------------------------------------------: | :-----------------------------------------------------------------------------------------------------: |
| [DriveVelocityCommand](src/main/include/commands/DriveVelocityCommand.h) |                          Drives the robot at a set velocity in a set direction                          |
|   [ExtendClimbCommand](src/main/include/commands/ExtendClimbCommand.h)   |                               Extends both ClimbSubsystems at a set speed                               |
|  [FlywheelRampCommand](src/main/include/commands/FlywheelRampCommand.h)  | Ramps up one of the sides of the ScoringSubsystem to a predefined speed based on the `ScoringDirection` |
|          [FeedCommand](src/main/include/commands/FeedCommand.h)          |     Feeds the note to one side of the robot using the vector motor based on the `ScoringDirection`      |
|         [ShootCommand](src/main/include/commands/ShootCommand.h)         |                         Stops the shooting motors after the note has been shot                          |
|                [Funni](src/main/include/commands/Funni.h)                |                                              Does a funni                                               |
|          [TurnToAngle](src/main/include/commands/TurnToAngle.h)          |                           Turns the robot to an angle using the gyro and PID                            |

## Command Compositions

|         Command          |                                                                                        Purpose                                                                                        |   Timeout   |
| :----------------------: | :-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------: | :---------: |
|          Intake          | Runs `IntakeInInitialCommand`, waits a predefined delay, runs `IntakeInSecondaryCommand`, waits a predefined delay, stops the motors. If there is a note present it exits immediately |  5 seconds  |
|   FeedUntilNotPresent    |                                                     Runs `FeedCommand` until the top beam break is not broken; stops the motors.                                                      |     N/A     |
|   DowntakeUntilPresent   |                                                      Downtakes the note until the bottom beam break is broken; stops the motors.                                                      |     N/A     |
|          Funni           |                                                                                     Does a Funni                                                                                      | 20 seconds  |
|   StopIntakeAndScoring   |                                                     Stops the intake and scoring motors; used to ensure nothing is left running.                                                      |     N/A     |
| RunUntilNotePresentUpper |                                                   Runs the intake until any of the upper beam breaks are broken; stops the motors.                                                    |     N/A     |
| RunUntilNotePresentLower |                                            Runs the intake until either the lower amp or podium beam breaks are broken; stops the motors.                                             |     N/A     |
|     PreScoreShuffle      |                                                    Runs until note present upper, runs until note present lower, stops the motors.                                                    |  1 second   |
|      ArmScoringGoal      |                 Takes in a ScoringDirection and decides which angle to set the arm to, returns a CommandPtr to set the arm's goal angle based on the ScoringDirection                 |     N/A     |
|        ScoreRamp         |                     Runs `ArmScoringGoal`, ramps the flywheel up to the speed based on the ScoringDirection. If there is not a note present it exits immediately.                     |  3 seconds  |
|        ScoreShoot        |                     Runs `FeedFeed`, waits a predefined delay, runs `ShootCommand`. If there is not a note present it exits immediately. Returns the arm to home                      |  3 seconds  |
|          Score           |                                                                           Runs `ScoreRamp` and `ScoreShoot`                                                                           | 1.5 Seconds |

## Factory Commands

|          Method          |                                                                                Purpose                                                                                |
| :----------------------: | :-------------------------------------------------------------------------------------------------------------------------------------------------------------------: |
| GetPathFromFinalLocation |               Takes in a `FinalLocation` and returns a CommandPtr to drive to that location using `GetApproxCommand` and then `GetFinalApproachCommand`               |
| GetPathFromFinalLocation | Takes in a `FinalLocation` and returns a CommandPtr to run a prep command before driving to that location using `GetApproxCommand` and then `GetFinalApproachCommand` |
|     GetApproxCommand     |      Takes in a `FinalLocation` and returns a PP on the fly command to the `ApproxPose` of the `FinalLocation`; flips the pose depending on the `FinalLocation`       |
| GetFinalApproachCommand  |                               Takes in a `FinalLocation` and returns a CommandPtr to drive from the `ApproxPose` to the `FinalLocation`                               |

## Autos

### Auto Factory

A factory for getting autos in a failsafe manner based on a enum key.

|         Method          |                                Purpose                                |
| :---------------------: | :-------------------------------------------------------------------: |
|     GetEmptyCommand     |  Returns a WaitCommand of 15 seconds that can be run as a failsafe.   |
| PathPlannerPathFromName | Gets the command for the `PathPlannerAuto` for a given string `name`. |
|         GetAuto         |  Gets the `PathPlannerAuto` Command for an auto given its `AutoType`  |

### Auto Chooser

A generic sendable chooser that allows you to filter down autos in a master chooser based on tags. By default the master chooser will hold all autos. By refining selections with the auxilary choosers, you can filter down the master list.

### Autos

|    AutoType     |          Display Name          |        File Name         |                     Tags                     |                                                                 Description                                                                  | Validated |
| :-------------: | :----------------------------: | :----------------------: | :------------------------------------------: | :------------------------------------------------------------------------------------------------------------------------------------------: | :-------: |
|    EmptyAuto    |           Empty Auto           |           N/A            |   `0 notes`, `minimal`, `close`, `center`    |                                                                 Does Nothing                                                                 |    Yes    |
|  FourNoteAuto   |          4 Note Auto           |       4 Note Auto        | `4 notes`, `close`, `high`, `amp`, `center`  |                                                        Scores 4 notes in the Speaker                                                         |    Yes    |
|  PlaceAndLeave  |        Place and leave         |     Place and leave      |     `1 note`, `minimal`, `mid`, `source`     |                                               Scores 1 note in the Speaker, crosses auto line                                                |    Yes    |
|  ThreeNoteAuto  |          3 Note Auto           |       3 Note Auto        |    `3 notes`, `close`, `decent`, `center`    |                                                        Scores 3 notes in the Speaker                                                         |    Yes    |
| TwoNoteAmpSide  |        2 Note Amp Side         |     2 Note Amp Side      |      `2 notes`, `decent`, `far`, `amp`       |                                   Scores 2 notes in the Speaker, starting on the Amp Side of the Subwoofer                                   |    Yes    |
|    LeaveWing    |           Leave Wing           |        Leave Wing        |   `0 notes`, `minimal`, `close`, `source`    |                                                             Leaves the auto zone                                                             |    Yes    |
|  TwoNoteCenter  | 2 Note Center Note Under Stage |   2 Note Center Note 3   |      `2 notes`, `far`, `high`, `center`      |     Scores 2 notes in the Speaker, starting on the Source Side of the Subwoofer and collecting the center most note from the center line     |    Yes    |
|  TwoNoteSource  |       2 Note Source Side       |    2 Note Source Side    |     `2 notes`, `decent`, `far`, `source`     | Scores 2 notes in the Speaker, starting on the Source Side of the Subwoofer and collecting the closest note to the Source on the center line |    Yes    |
| ThreeNoteCenter |    3 Note Center Note 3 + 4    | 3 Note Center Note 3 + 4 | `3 notes`, `high`, `far`, `source`, `center` |           Scores 3 notes in the Speaker, starting on the Source Side of the Subwoofer and collecting 2 notes from the center line            |    Yes    |
|   TwoNoteAuto   |         2 Note Auto++          |       2 Note Auto        |      `3 notes`, `decent`, `far`, `amp`       |   Scores 3 notes in the Subwoofer, starting on the Amp Side of the Subwoofer and collecting the closest note to the Amp on the center line   |    Yes    |

## State Commands

All robot states begin by calling `ShowFromState` with the `RobotState` on the LedSubsystem

|        Method         |                                             Purpose                                             |  Timeout   |
| :-------------------: | :---------------------------------------------------------------------------------------------: | :--------: |
|       RunState        |       Returns a CommandPtr to run the current state and then return to the `Manual` state       |    N/A     |
|     StartIntaking     | Returns a CommandPtr to drive while running the intake until a note is present or until timeout | 5 seconds  |
|  StartScoringSpeaker  |                      Returns a CommandPtr to drive to the podium and score                      | 20 seconds |
|    StartScoringAmp    |                       Returns a CommandPtr to drive to the amp and score                        | 20 seconds |
| StartScoringSubwoofer |                    Returns a CommandPtr to drive to the subwoofer and score                     | 20 seconds |
|      StartManual      |                                   Switches to manual control                                    |    N/A     |
|      StartClimb       |              Returns a CommandPtr to drive to a stage location, climb, and balance              | 20 seconds |
|      StartSource      |                       Returns a CommandPtr to drive to a source location                        | 20 seconds |
|      StartFunni       |                                              Funni                                              |    N/A     |

## LED Commands

|       Method        |                                                      Purpose                                                       |
| :-----------------: | :----------------------------------------------------------------------------------------------------------------: |
|    ShowFromState    |                           Returns a CommandPtr to set the LEDs based on a `StateGetter`                            |
|      Intaking       | Sets left and right zones to red and a chase pattern, sets the front and back zones to red and a sine roll pattern |
|   ScoringSpeaker    |             Sets left and right and front zones to solid yellow, sets back zone to purple and a blink              |
|     ScoringAmp      |               Sets left right and back zones to solid yellow, sets front zone to purple and a blink                |
|       Loaded        |                                           Sets all zones to blink green                                            |
|       Idling        |                                           Sets all zones to breathe blue                                           |
|        Climb        |             Sets left and right zones to a blue sine roll, front and back zones to a yellow sine roll              |
|    AimbotEnable     |                                        Sets all LEDs to an acid green blink                                        |
|     OnTheFlyPP      |                                               Sets all LEDs to blue                                                |
|  SuccessfulIntake   |                                                 Sets LEDs to green                                                 |
| VisionNoteDetected  |                                             Sets all LEDs chase orange                                             |
|        Error        |                                         Sets all zones to red and a blink                                          |
| setZoneColorPattern |                                    Helper to set a zone to a color and pattern                                     |
|     createZones     |                                      Helper to create a std::vector of zones                                       |
|    syncAllZones     |                                              Helper to sync all zones                                              |

## Getting started

### Prerequisites

Install the following:

- [WPILib](https://docs.wpilib.org/en/stable/docs/getting-started/getting-started-frc-control-system/wpilib-setup.html)
- [Git](https://git-scm.com/download/win) (You don't need GitHub desktop)

## Making changes

### Cloning the repo

1. Install the [GitHub VS Code extension](vscode:extension/GitHub.vscode-pull-request-github)
2. Go to the GitHub tab and sign in using the account that has access to this repository
3. Click `Clone Repository` and search for `SubZero-Robotics/2024-CommandRobot`
4. Clone it into a known folder that has **no spaces** in its path
5. Open the repository in VS Code

### Issues

[GitHub issues](https://github.com/SubZero-Robotics/2024-CommandRobot/issues) are how we track which tasks need work along with who should be working on them. Pay attention to the issue number since this is how each one is uniquely identified. To create a new issue, visit the [issues tab in the repository](https://github.com/SubZero-Robotics/2023-swerve-base/issues) and click `New issue`. Give it a descriptive title and enough information in the comment area so that anyone working on the issue knows exactly what needs to be changed/fixed. Additionally, assignees (who is working on it) and labels (what type of issue is this) can be assigned on the right.

### Branches

Branches are simply named pointers that point to a commit. Our main branch is called `main`, but we don't make changes to it directly. Instead, a new branch should be created before any code is modified. Click on the current branch in the bottom-left corner in VS Code and then click `+ Create new branch...`. The following naming conventions should be followed:

For a feature branch (major change):
`feature-<issue number>-<a few words describing the change>`

For a smaller, individual/small group branch:
`<first name>-<issue number>-<a few words describing the change>`

### Adding commits

After finishing a small change, such as modifying a method or adding a new file, a new commit should be made immediately. Go to the `Source Control` tab on the left side of VS Code and add the changed/added/deleted files from the `Changes` dropdown to `Staging` by clicking the `+`. Once the changes are staged, Type a **descriptive** commit message and then click the `Commit` button.

### Pushing commits

Commiting only applies the new commit locally. So to make it available to others in the repo, it needs to be pushed to the `remote` (GitHub's servers in this case). Either click `publish` in the `Source Control` tab if the branch hasn't been pushed before or `push` it otherwise to send the new commit to the remote's branch (this will be called `origin/<your branch's name>`).

### Pulling branches

If someone else has made their branch available on the remote, you might be wondering how other people can get those same commits into their local repo. This process is called `pulling`, and it involves downloading the remote's commits and merging them with the local branch's commits.

Sometimes, you might see a `merge conflict` appear which can happen if more than one person has worked on the same line in a file, thus making git unable to automatically merge them. To fix this, click on the conflicted file (denoted by a `!`) that opens the merge conflict editor. Once at the line(s) in question, either accept the incoming (one that exists on GitHub), the HEAD (the one that you have locally), or attempt to manually merge the two together by directly editing the line(s). Repeat this process for each line/file until all conflicts are resolved. Once done, stage the now-fixed files, create a new commit, and push.

### Pull requests

Now that all of your changes have been made (and *tested!*), it's time to get them merged into the main branch. Either click the `Create Pull Request` button in the `Source Control` tab (inline with the `SOURCE CONTROL` text, fourth one from the left) or go to the [`Pull requests` tab on GitHub](https://github.com/SubZero-Robotics/2024-CommandRobot/pulls), click `New pull request`, and set the `compare` branch to your branch. Fill in the template with a good title, a detailed description, a list of changes made, and the issue number before creating it. Assign a reviewer(s) on the right and label the PR accordingly.

The reviewer is responsible for looking over the PR, testing the changes themselves, and adding comments to the code changes if necessary. There are three possible actions a reviewer can take:

- Approve (PR is good; required to merge and should be done by a mentor or team lead)
- Disapprove (PR needs changes before it can be merged; please address the comments, click `Resolve` under each comment once fixed, then re-request a review)
- Comment (simply add a comment(s) without approving or disapproving)

## Contact

If you would like to contact the subzero robotics programmers, you can do so at [5690programmers@gmail.com](mailto:5690Programmers@gmail.com)

To contact the team directly, you can find us on [facebook](https://www.facebook.com/Esko-SubZero-Robotics-Team-5690-695407257248414/) or email us at [subzerorobotics@goesko.com](mailto:subzerorobotics@goesko.com)

Project Link: [https://github.com/subzero-robotics/2024-CommandRobot](https://github.com/subzero-robotics/2024-CommandRobot)
