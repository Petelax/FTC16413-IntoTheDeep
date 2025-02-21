package org.firstinspires.ftc.teamcode.drive

import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Race
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.IfElse
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.teamcode.commands.Timeout
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.subsystems.Deposit
import org.firstinspires.ftc.teamcode.subsystems.Elevator
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtension
import org.firstinspires.ftc.teamcode.subsystems.VerticalArm
import org.firstinspires.ftc.teamcode.subsystems.VerticalWrist
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrivetrain
import org.firstinspires.ftc.teamcode.utils.BulkReads
import org.firstinspires.ftc.teamcode.utils.Globals
import org.firstinspires.ftc.teamcode.utils.LoopTimes
import org.firstinspires.ftc.teamcode.utils.Telemetry
import org.firstinspires.ftc.teamcode.utils.pathing.CurvePoint
import org.firstinspires.ftc.teamcode.utils.pathing.PurePursuitController

@Mercurial.Attach
@BulkReads.Attach
@LoopTimes.Attach
@Telemetry.Attach

@SwerveDrivetrain.Attach

@HorizontalExtension.Attach
//@HorizontalArm.Attach
//@HorizontalWrist.Attach
//@Intake.Attach

@Elevator.Attach
@VerticalArm.Attach
@VerticalWrist.Attach
@Deposit.Attach

@Autonomous
class FiveSpecimenAuto : OpMode() {
    val verticalSpecimenPickup = Parallel(
        Elevator.waitUntilSetPoint(VerticalConstants.ElevatorPositions.BOTTOM),
        Elevator.pidAuto(VerticalConstants.ElevatorPositions.BOTTOM),
        VerticalArm.specimen(),
        VerticalWrist.specimenPickup(),
    )

    val verticalSpecimenPlace = Parallel(
        Elevator.pidAutoTimeout(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE+1.0, 2.0),
        Elevator.waitUntilAboveArm(),
        Parallel(
            VerticalArm.specimen(),
            VerticalWrist.specimenPlace(),
        ),
    )

    val verticalRetract = IfElse( {!VerticalArm.isArmIntake()},
        Sequential(
            IfElse( {Elevator.getPosition() < VerticalConstants.ElevatorPositions.ARM},
                Sequential(
                    Elevator.pid(VerticalConstants.ElevatorPositions.ARM_TARGET+0.5),
                    Elevator.waitUntilAboveArm(),
                ),
                Sequential()
            ),
            IfElse( {VerticalArm.isArmSpecimen() && Elevator.getPosition() > VerticalConstants.ElevatorPositions.ARM},
                Sequential(
                    Parallel(
                        Wait(VerticalConstants.VerticalArmConstants.specimenToIntake),
                        VerticalArm.intake(),
                        VerticalWrist.intake(),
                        Deposit.open()
                    ),
                    Parallel(
                        Elevator.waitUntilSetPoint(VerticalConstants.ElevatorPositions.BOTTOM),
                        Elevator.pid(VerticalConstants.ElevatorPositions.BOTTOM),
                    )
                ),
                Sequential(
                    Deposit.open(),
                    Wait(0.100),
                    Parallel(
                        Wait(VerticalConstants.VerticalArmConstants.sampleToIntake),
                        VerticalArm.intake(),
                        VerticalWrist.intake(),
                        Deposit.open()
                    ),
                    Parallel(
                        Elevator.waitUntilSetPoint(VerticalConstants.ElevatorPositions.BOTTOM),
                        Elevator.pid(VerticalConstants.ElevatorPositions.BOTTOM),
                    )

                )
            )
        ),
        Parallel(
            Elevator.waitUntilSetPoint(VerticalConstants.ElevatorPositions.BOTTOM),
            Elevator.pid(VerticalConstants.ElevatorPositions.BOTTOM),
            VerticalArm.intake(),
            VerticalWrist.intake(),
            Deposit.open(),
        )

    )
    /*
    val horizontalExtend = Parallel(HorizontalExtension.waitUntilSetPoint(HorizontalConstants.HorizontalExtensionPositions.MID), HorizontalExtension.pid(
            HorizontalConstants.HorizontalExtensionPositions.MID), HorizontalArm.outHorizontalArm(), HorizontalWrist.outHorizontalWrist())
     */

    private val first = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(78.0, 7.375, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(78.0, 24.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(78.0, 25.5, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(78.0, 35.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 6.0),
    ), kFollowDistance = 12.0, kPID=0.1, kFF=0.9)


    private val second = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(78.0, 36.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(78.0, 34.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(78.0, 30.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(78.0, 25.5, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(82.0, 25.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(87.0, 25.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(90.0, 25.0, Rotation2d.fromDegrees(180.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(108.0, 30.0, Rotation2d.fromDegrees(-95.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(109.0, 53.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(113.0, 52.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(120.0, 40.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(120.0, 12.0, Rotation2d.fromDegrees(-90.0)), 1.0, 0.8, 6.0, 0.1, 0.9),
    ), kSmooth = 0.895, minFollowDistance = 4.0, kFollowDistance = 6.0, kCurvature = 0.15, spacing = 1.5, kPID=0.9, kFF=0.1)

    private val pickupFirst = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(137.25, 58.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(137.25, 50.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(137.25, 40.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0, 0.1, 0.9),
        CurvePoint(Pose2d(137.25, 24.0, Rotation2d.fromDegrees(-90.0)), 0.85, 0.2, 6.0, 0.1, 0.9),
        CurvePoint(Pose2d(137.25, 20.5, Rotation2d.fromDegrees(-90.0)), 0.45, 0.2, 6.0, 0.1, 0.9),
        CurvePoint(Pose2d(137.25, 16.5, Rotation2d.fromDegrees(-90.0)), 0.35, 0.2, 6.0, 0.1, 0.9),
        CurvePoint(Pose2d(137.25, 0.0, Rotation2d.fromDegrees(-90.0)), 0.20, 0.2, 6.0, 0.05, 0.95),
    ), kSmooth = 0.895, minFollowDistance = 4.5, kFollowDistance = 8.0, spacing = 1.5, kPID=0.5, kFF=0.5)

    private val placeFirst = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(137.25, 16.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(137.25, 20.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(125.89, 20.27, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(75.0, 22.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(75.0, 36.5, Rotation2d.fromDegrees(90.0)), 0.95, 1.0, 5.0),
    ), kSmooth = 0.95, kPID=0.9, kFF=0.1, kFollowDistance = 8.0, kCurvature = 0.075)

    private val pickupSecond = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(75.0, 36.0, Rotation2d.fromDegrees(90.0)), 0.9, 1.0, 5.0),
        CurvePoint(Pose2d(75.0, 30.0, Rotation2d.fromDegrees(90.0)), 0.9, 1.0, 5.0),
        CurvePoint(Pose2d(75.0, 26.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(75.0, 25.5, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(85.0, 26.5, Rotation2d.fromDegrees(170.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(95.0, 26.5, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(108.0, 26.5, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(108.0, 22.0, Rotation2d.fromDegrees(-90.0)), 0.50, 0.1, 6.0),
        CurvePoint(Pose2d(108.0, 16.5, Rotation2d.fromDegrees(-90.0)), 0.25, 0.1, 6.0, 0.3, 0.7),
        CurvePoint(Pose2d(108.0, 0.0, Rotation2d.fromDegrees(-90.0)), 0.20, 0.1, 6.0, 0.1, 0.9),
    ), kSmooth = 0.95, kCurvature = 0.08, kPID=0.9, kFF=0.1)

    private val placeSecond = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(108.0, 16.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(108.0, 20.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(107.09, 19.25, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(72.0, 22.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(72.0, 36.5, Rotation2d.fromDegrees(90.0)), 0.95, 1.0, 5.0),
    ), kSmooth = 0.95, kPID=0.9, kFF=0.1, kFollowDistance = 8.0, kCurvature = 0.073)

    private val pickupThird = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(72.0, 36.0, Rotation2d.fromDegrees(90.0)), 0.9, 1.0, 5.0),
        CurvePoint(Pose2d(72.0, 30.0, Rotation2d.fromDegrees(90.0)), 0.9, 1.0, 5.0),
        CurvePoint(Pose2d(72.0, 26.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(72.0, 26.5, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(85.0, 26.5, Rotation2d.fromDegrees(170.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(95.0, 26.5, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(108.0, 26.5, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(108.0, 22.0, Rotation2d.fromDegrees(-90.0)), 0.50, 0.1, 6.0),
        CurvePoint(Pose2d(108.0, 16.5, Rotation2d.fromDegrees(-90.0)), 0.25, 0.1, 6.0, 0.3, 0.7),
        CurvePoint(Pose2d(108.0, 0.0, Rotation2d.fromDegrees(-90.0)), 0.20, 0.1, 6.0, 0.1, 0.9),
    ), kSmooth = 0.95, kCurvature = 0.08, kPID=0.9, kFF=0.1)

    private val placeThird = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(108.0, 16.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(108.0, 20.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(107.0, 19.25, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(69.0, 22.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(69.0, 36.5, Rotation2d.fromDegrees(90.0)), 0.95, 1.0, 5.0),
    ), kSmooth = 0.95, kPID=0.9, kFF=0.1, kFollowDistance = 8.0, kCurvature = 0.073)

    private val pickupFourth = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(69.0, 36.0, Rotation2d.fromDegrees(90.0)), 0.9, 1.0, 5.0),
        CurvePoint(Pose2d(69.0, 30.0, Rotation2d.fromDegrees(90.0)), 0.9, 1.0, 5.0),
        CurvePoint(Pose2d(69.0, 26.5, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(69.0, 26.5, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(85.0, 26.5, Rotation2d.fromDegrees(170.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(95.0, 26.5, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(108.0, 26.5, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(108.0, 22.5, Rotation2d.fromDegrees(-90.0)), 0.50, 0.1, 6.0),
        CurvePoint(Pose2d(108.0, 16.5, Rotation2d.fromDegrees(-90.0)), 0.25, 0.1, 6.0, 0.3, 0.7),
        CurvePoint(Pose2d(108.0, 0.0, Rotation2d.fromDegrees(-90.0)), 0.20, 0.1, 6.0, 0.1, 0.9),
    ), kSmooth = 0.95, kCurvature = 0.08, kPID=0.9, kFF=0.1)

    private val placeFourth = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(108.0, 16.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(108.0, 20.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(107.0, 19.25, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(66.0, 22.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(66.0, 36.5, Rotation2d.fromDegrees(90.0)), 0.95, 1.0, 5.0),
    ), kSmooth = 0.95, kPID=0.9, kFF=0.1, kFollowDistance = 8.0, kCurvature = 0.073)

    private val park = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(108.0, 16.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(108.0, 20.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(66.0, 36.5, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 5.0),
    ), kSmooth = 0.95, kPID=0.9, kFF=0.1, kFollowDistance = 8.0, kCurvature = 0.075)

    val auto = Sequential(
        Parallel(
            VerticalArm.specimen(),
            VerticalWrist.specimenPlace(),
            Timeout(Elevator.pidAuto(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE+1.0), 1.0),
            Sequential(
                Timeout(SwerveDrivetrain.alignModules(Pose2d(78.0, 7.375, Rotation2d.fromDegrees(90.0))), 0.1),
                Timeout(PurePursuitController.followPathCommand(first, constants = DrivebaseConstants.noVelocity), 1.7),
            )
        ),

        //Wait(0.15),
        Deposit.open(),
        Wait(0.05),

        /*
         * second
         */

        Lambda("print-path").setInit{Telemetry.path = second},

        Race(
            Timeout(PurePursuitController.followPathCommand(second, constants=DrivebaseConstants.noVelocity), 15.0),
            Sequential(
                Timeout(Elevator.pidAuto(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE-4.0), 0.80),
                Timeout(
                    Parallel(
                        VerticalArm.setVerticalArm(VerticalConstants.VerticalArmPositions.AUTO_START),
                        VerticalWrist.specimenPickup(),
                        Elevator.pidAuto(VerticalConstants.ElevatorPositions.BOTTOM),
                    ),
                    3.0
                )
            ),
        ),

        Timeout(
            Sequential(
                SwerveDrivetrain.bnvp2p(Pose2d(122.0, 54.0, Rotation2d.fromDegrees(-90.0)), 1.0, DrivebaseConstants.noVelocity.copy(TranslationPositionTolerance = 2.0, RotationPositionTolerance = 0.1)),
                SwerveDrivetrain.bnvp2p(Pose2d(132.0, 54.0, Rotation2d.fromDegrees(-90.0)), 1.0, DrivebaseConstants.noVelocity.copy(RotationPositionTolerance = 0.1)),
                SwerveDrivetrain.bnvp2p(Pose2d(132.0, 12.0, Rotation2d.fromDegrees(-90.0)), 1.0, DrivebaseConstants.noVelocity.copy(TranslationPositionTolerance = 2.0, RotationPositionTolerance = 0.1)),

                VerticalArm.specimen(),
                SwerveDrivetrain.bnvp2p(Pose2d(132.0, 54.0, Rotation2d.fromDegrees(-90.0)), 1.0, DrivebaseConstants.noVelocity.copy(TranslationPositionTolerance = 2.0, RotationPositionTolerance = 0.1)),
                SwerveDrivetrain.bnvp2p(Pose2d(137.75, 54.0, Rotation2d.fromDegrees(-90.0)), 1.0),

            ),
            15.0
        ),

        Race( null,
            Sequential(Wait(0.2), Deposit.waitUntilHoldingPiece()),
            Timeout(PurePursuitController.followPathCommand(pickupFirst, constants=DrivebaseConstants.noVelocity), 4.0),
        ),

        Wait(0.05),
        Deposit.close(),
        Wait(0.10),
        /*
        Parallel(
            Elevator.pidAutoTimeout(VerticalConstants.ElevatorPositions.BOTTOM+0.1, 0.5),
            SwerveDrivetrain.forwardTime(-0.13, 0.175)
        ),
         */
        Parallel(
            VerticalArm.specimen(),
            VerticalWrist.specimenPlace(),
            Sequential(
                Elevator.pidAutoTimeout(VerticalConstants.ElevatorPositions.BOTTOM+1.0, 0.7),
                Elevator.pidAutoTimeout(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE+0.90, 2.0),
            ),
            Sequential(
                Wait(0.1),
                Timeout(PurePursuitController.followPathCommand(placeFirst, DrivebaseConstants.Measurements.velocityTimeout+300.0, constants=DrivebaseConstants.noVelocity), 4.0),
            )
        ),

        Deposit.open(),
        Wait(0.05),

        Race( null,
            Sequential(Wait(1.0), Deposit.waitUntilHoldingPiece()),
            Parallel(
                Timeout(PurePursuitController.followPathCommand(pickupSecond), 5.0),
                Sequential(
                    Timeout(Elevator.pidAuto(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE-4.0), 0.5),
                    Timeout(verticalSpecimenPickup, 3.0)
                ),
            ),
        ),


        Wait(0.05),
        Deposit.close(),
        Wait(0.100),
        /*
        Parallel(
            Elevator.pidAutoTimeout(VerticalConstants.ElevatorPositions.BOTTOM+0.1, 0.5),
            SwerveDrivetrain.forwardTime(-0.13, 0.175)
        ),

         */
        Parallel(
            VerticalArm.specimen(),
            VerticalWrist.specimenPlace(),
            Sequential(
                Elevator.pidAutoTimeout(VerticalConstants.ElevatorPositions.BOTTOM+1.0, 0.7),
                Elevator.pidAutoTimeout(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE+0.90, 2.0),
            ),
            Sequential(
                Wait(0.1),
                Timeout(PurePursuitController.followPathCommand(placeSecond, DrivebaseConstants.Measurements.velocityTimeout+300.0, constants=DrivebaseConstants.noVelocity), 4.0),
            )
        ),

        Deposit.open(),
        Wait(0.05),


        Race( null,
            Sequential(Wait(1.0), Deposit.waitUntilHoldingPiece()),
            Parallel(
                Timeout(PurePursuitController.followPathCommand(pickupThird), 5.0),
                Sequential(
                    Timeout(Elevator.pidAuto(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE-4.0), 0.5),
                    Timeout(verticalSpecimenPickup, 3.0)
                ),
            ),
        ),


        Wait(0.05),
        Deposit.close(),
        Wait(0.100),
        /*
        Parallel(
            Elevator.pidAutoTimeout(VerticalConstants.ElevatorPositions.BOTTOM+0.1, 0.5),
            SwerveDrivetrain.forwardTime(-0.13, 0.175)
        ),
         */
        Parallel(
            VerticalArm.specimen(),
            VerticalWrist.specimenPlace(),
            Sequential(
                Elevator.pidAutoTimeout(VerticalConstants.ElevatorPositions.BOTTOM+1.0, 0.7),
                Elevator.pidAutoTimeout(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE+0.90, 2.0),
            ),
            Sequential(
                Wait(0.1),
                Timeout(PurePursuitController.followPathCommand(placeThird, DrivebaseConstants.Measurements.velocityTimeout+300.0, constants=DrivebaseConstants.noVelocity), 4.0),
            )
        ),


        Deposit.open(),
        Wait(0.05),


        Race( null,
            Sequential(Wait(1.0), Deposit.waitUntilHoldingPiece()),
            Parallel(
                Timeout(PurePursuitController.followPathCommand(pickupFourth), 5.0),
                Sequential(
                    Timeout(Elevator.pidAuto(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE-4.0), 0.5),
                    Timeout(verticalSpecimenPickup, 3.0)
                ),
            ),
        ),


        Wait(0.05),
        Deposit.close(),
        Wait(0.100),
        /*
        Parallel(
            Elevator.pidAutoTimeout(VerticalConstants.ElevatorPositions.BOTTOM+0.1, 0.5),
            SwerveDrivetrain.forwardTime(-0.13, 0.175)
        ),

         */
        Parallel(
            VerticalArm.specimen(),
            VerticalWrist.specimenPlace(),
            Sequential(
                Wait(0.7),
                Elevator.pidAutoTimeout(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE+0.90, 2.0),
            ),
            Sequential(
                Elevator.pidAutoTimeout(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE+0.90, 2.0),
            ),
            Sequential(
                Timeout(PurePursuitController.followPathCommand(placeFourth, DrivebaseConstants.Measurements.velocityTimeout+300.0, constants=DrivebaseConstants.noVelocity), 4.0),
            )
        ),

        Deposit.open(),
        Wait(0.05),
        Timeout(Elevator.pidAuto(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE-4.0), 0.5),

        Race( null,
            Sequential(Wait(1.0), Deposit.waitUntilHoldingPiece()),
            Parallel(
                Timeout(PurePursuitController.followPathCommand(park), 5.0),
                Sequential(
                    Timeout(Elevator.pidAuto(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE-4.0), 0.5),
                    Timeout(verticalRetract, 3.0)
                ),
            ),
        ),

    )

    override fun init() {
        SwerveDrivetrain.setPose(PPSpecimenAuto.specimenAutoPoses.startPose)
        //SwerveDrivetrain.setPose(startPose)
        SwerveDrivetrain.alignModules(Pose2d(78.0, 7.375, Rotation2d.fromDegrees(90.0))).schedule()

        VerticalArm.setPosition(VerticalConstants.VerticalArmPositions.AUTO_START)
        Deposit.setPosition(VerticalConstants.DepositPositions.IN)
        VerticalWrist.setPosition(VerticalConstants.VerticalWristPositions.INTAKE)

        SwerveDrivetrain.defaultCommand = SwerveDrivetrain.stopCmd()
        Elevator.defaultCommand = null
        //HorizontalExtension.defaultCommand = HorizontalExtension.hold()

        Telemetry.path = first
        Telemetry.put("alliance colour", Globals.AllianceColour.name)
        //Telemetry.points.add( Pose2d(48.0, 0.0, Rotation2d()) )

    }

    override fun init_loop() {

    }

    override fun start() {
        auto.schedule()
    }

    override fun loop() {

    }


}