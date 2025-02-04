package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Race
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.teamcode.commands.Timeout
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
class PPSpecimenAuto : OpMode() {
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

    private val first = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(78.0, 7.375, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(78.0, 24.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(78.0, 25.5, Rotation2d.fromDegrees(90.0)), 0.4, 1.0, 6.0),
        CurvePoint(Pose2d(78.0, 35.0, Rotation2d.fromDegrees(90.0)), 0.2, 1.0, 6.0),
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
        CurvePoint(Pose2d(109.0, 60.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(120.0, 60.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(120.0, 50.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(120.0, 40.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0, 0.1, 0.9),
        CurvePoint(Pose2d(120.0, 35.0, Rotation2d.fromDegrees(-90.0)), 0.40, 0.1, 6.0, 0.1, 0.9),
        CurvePoint(Pose2d(120.0, 16.5, Rotation2d.fromDegrees(-90.0)), 0.30, 0.1, 6.0, 0.1, 0.9),
        CurvePoint(Pose2d(120.0, 0.0, Rotation2d.fromDegrees(-90.0)), 0.20, 0.1, 6.0, 0.05, 0.95),
    ), kSmooth = 0.895, minFollowDistance = 4.5, kFollowDistance = 6.0, kCurvature = 0.080, spacing = 1.5, kPID=0.9, kFF=0.1)

    private val third = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(120.0, 16.5, Rotation2d.fromDegrees(-90.0)), 0.8, 1.0, 6.0),
        CurvePoint(Pose2d(120.0, 20.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(118.19, 18.60, Rotation2d.fromDegrees(-180.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(116.10, 19.55, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(73.0, 21.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(73.0, 35.75, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
    ), kSmooth = 0.95, kCurvature = 0.08, kPID=0.9, kFF=0.1)

    private val fourth = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(73.0, 36.0, Rotation2d.fromDegrees(90.0)), 0.65, 1.0, 5.0),
        CurvePoint(Pose2d(73.0, 26.0, Rotation2d.fromDegrees(90.0)), 0.65, 1.0, 5.0),
        CurvePoint(Pose2d(82.0, 26.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 2.0),
        CurvePoint(Pose2d(90.50, 29.05, Rotation2d.fromDegrees(-180.0)), 1.0, 1.0, 2.0),
        CurvePoint(Pose2d(100.0, 32.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 2.0),
        CurvePoint(Pose2d(108.0, 40.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 2.0),
        CurvePoint(Pose2d(110.0, 45.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 2.0),
        CurvePoint(Pose2d(118.0, 57.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 2.0),
        CurvePoint(Pose2d(132.0, 57.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 2.0),
        CurvePoint(Pose2d(132.0, 40.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(132.0, 35.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0, 0.1, 0.9),
        CurvePoint(Pose2d(132.0, 30.0, Rotation2d.fromDegrees(-90.0)), 0.4, 0.1, 6.0, 0.1, 0.9),
        CurvePoint(Pose2d(132.0, 16.5, Rotation2d.fromDegrees(-90.0)), 0.3, 0.1, 6.0, 0.1, 0.9),
        CurvePoint(Pose2d(132.0, 0.0, Rotation2d.fromDegrees(-90.0)), 0.2, 0.1, 6.0, 0.05, 0.95),
    ), kSmooth = 0.95, kCurvature = 0.075)

    private val fifth = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(132.0, 16.5, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(132.0, 20.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(118.19, 20.5, Rotation2d.fromDegrees(-180.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(116.10, 21.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(70.0, 24.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(70.0, 35.75, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
    ), kSmooth = 0.95, kCurvature = 0.08, kPID=0.9, kFF=0.1)

    private val sixth = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(70.0, 36.0, Rotation2d.fromDegrees(90.0)), 0.9, 1.0, 5.0),
        CurvePoint(Pose2d(70.0, 30.0, Rotation2d.fromDegrees(90.0)), 0.9, 1.0, 5.0),
        CurvePoint(Pose2d(70.0, 25.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(70.0, 23.5, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(85.0, 24.0, Rotation2d.fromDegrees(170.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(95.0, 24.5, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(100.0, 24.5, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(120.0, 25.0, Rotation2d.fromDegrees(-90.0)), 0.30, 0.1, 6.0),
        CurvePoint(Pose2d(120.0, 16.5, Rotation2d.fromDegrees(-90.0)), 0.20, 0.1, 6.0, 0.3, 0.7),
        CurvePoint(Pose2d(120.0, 0.0, Rotation2d.fromDegrees(-90.0)), 0.20, 0.1, 6.0, 0.1, 0.9),
    ), kSmooth = 0.95, kCurvature = 0.08, kPID=0.9, kFF=0.1)

    private val seventh = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(120.0, 16.5, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(120.0, 20.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(118.19, 20.5, Rotation2d.fromDegrees(-180.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(116.10, 21.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(65.0, 22.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(65.0, 36.0, Rotation2d.fromDegrees(90.0)), 0.95, 1.0, 5.0),
    ), kSmooth = 0.95, kCurvature = 0.075, kPID=0.9, kFF=0.1)

    private val eighth = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(65.0, 36.0, Rotation2d.fromDegrees(90.0)), 0.9, 1.0, 5.0),
        CurvePoint(Pose2d(65.0, 23.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0),
        CurvePoint(Pose2d(120.0, 16.5, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 6.0),
    ), kSmooth = 0.95, kCurvature = 0.08, kPID=0.8, kFF=0.2)

    val auto = Sequential(
        Parallel(
            VerticalArm.specimen(),
            VerticalWrist.specimenPlace(),
            Timeout(Elevator.pidAuto(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE+1.0), 1.0),
            Sequential(
                Wait(0.35),
                Timeout(PurePursuitController.followPathCommand(first), 1.7),
            )
        ),

        //Wait(0.15),
        Deposit.open(),
        Wait(0.2),

        /*
         * second
         */

        Lambda("print-path").setInit{Telemetry.path = second},

        Race( null,
            Sequential(Wait(1.0), Deposit.waitUntilHoldingPiece()),
            Parallel(
                Timeout(PurePursuitController.followPathCommand(second), 10.0),
                Sequential(
                    Wait(0.80),
                    verticalSpecimenPickup
                ),
            ),
        ),

        /*
        Wait(0.05),
        //SwerveDrivetrain.forwardTime(0.12, 1.3),
        Timeout(SwerveDrivetrain.forwardSensor(0.3), 2.0),

         */

        Wait(0.10),
        Deposit.close(),
        Wait(0.200),

        /*
         * third
         */
        Lambda("print-path").setInit{Telemetry.path = third},

        Parallel(
            Elevator.pidAutoTimeout(VerticalConstants.ElevatorPositions.BOTTOM+1.0, 0.5),
            SwerveDrivetrain.forwardTime(-0.13, 0.175)
        ),

        Parallel(
            VerticalArm.specimen(),
            VerticalWrist.specimenPlace(),
            Sequential(
                Wait(0.7),
                Elevator.pidAutoTimeout(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE+1.0, 2.0),
            ),
            Sequential(
                Timeout(PurePursuitController.followPathCommand(third), 4.0),
            )

        ),

        /*
         * fourth
         */
        Lambda("print-path").setInit{Telemetry.path = fourth},

        //Wait(0.2),
        Deposit.open(),
        Wait(0.2),

        Race( null,
            Sequential(Wait(1.0), Deposit.waitUntilHoldingPiece()),
            Parallel(
                Timeout(PurePursuitController.followPathCommand(fourth), 4.5),
                Sequential(
                    Wait(0.75),
                    verticalSpecimenPickup
                ),
            ),
        ),

        /*
         * fifth
         */
        Lambda("print-path").setInit{Telemetry.path = fifth},

        /*
        Wait(0.05),
        //SwerveDrivetrain.forwardTime(0.12, 1.3),
        Timeout(SwerveDrivetrain.forwardSensor(0.2), 2.0),

         */

        Wait(0.10),
        Deposit.close(),
        Wait(0.200),

        Parallel(
            Elevator.pidAutoTimeout(VerticalConstants.ElevatorPositions.BOTTOM+1.0, 0.5),
            SwerveDrivetrain.forwardTime(-0.13, 0.175)
        ),

        Parallel(
            VerticalArm.specimen(),
            VerticalWrist.specimenPlace(),
            Sequential(
                Wait(1.25),
                Elevator.pidAutoTimeout(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE+1.0, 3.0),
            ),
            Sequential(
                Timeout(PurePursuitController.followPathCommand(fifth), 3.5),
            )

        ),

        /*
         * sixth
         */
        Lambda("print-path").setInit{Telemetry.path = sixth},

        //Wait(0.2),
        Deposit.open(),
        Wait(0.2),

        Race( null,
            Sequential(Wait(1.0), Deposit.waitUntilHoldingPiece()),
            Parallel(
                Timeout(PurePursuitController.followPathCommand(sixth), 5.0),
                Sequential(
                    Wait(0.5),
                    verticalSpecimenPickup
                ),
            ),
        ),

        /*
        Wait(0.05),
        //SwerveDrivetrain.forwardTime(0.12, 1.3),
        Timeout(SwerveDrivetrain.forwardSensor(0.2), 2.0),
         */

        /*
         * seventh
         */

        Wait(0.05),
        Deposit.close(),
        Wait(0.200),

        Parallel(
            Elevator.pidAutoTimeout(VerticalConstants.ElevatorPositions.BOTTOM+1.0, 0.5),
            SwerveDrivetrain.forwardTime(-0.13, 0.175)
        ),

        Parallel(
            VerticalArm.specimen(),
            VerticalWrist.specimenPlace(),
            Sequential(
                Wait(1.3),
                Elevator.pidAutoTimeout(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE+1.0, 2.0),
            ),
            Sequential(
                Timeout(PurePursuitController.followPathCommand(seventh), 3.5),
            )

        ),

        /*
         * eighth
         */

        Wait(0.2),
        Deposit.open(),
        Wait(0.2),

        Parallel(
            Timeout(PurePursuitController.followPathCommand(eighth), 5.0),
            Sequential(
                Wait(0.5),
                VerticalArm.intake(),
                VerticalWrist.intake(),
                Wait(0.7),
                Elevator.pidAuto(VerticalConstants.ElevatorPositions.BOTTOM),
            )
        )

        /*
        Parallel(
            Elevator.pidAutoTimeout(VerticalConstants.ElevatorPositions.BOTTOM+1.0, 0.5),
            SwerveDrivetrain.forwardTime(-0.13, 0.25)
        ),

        Wait(0.1),
        Parallel(
            verticalSpecimenPlace,
            Sequential(Wait(0.1), SwerveDrivetrain.bp2p(Pose2d(70.0, 36.0, Rotation2d.fromDegrees(90.0)), 1.5)),
            //changed
        ),
        Wait(0.1),
        Deposit.open(),
        Wait(0.1),

        Parallel(
            verticalSpecimenPickup,
            Sequential(
                SwerveDrivetrain.bp2p(Pose2d(104.0, 25.0, Rotation2d.fromDegrees(90.0)), 3.0),
                SwerveDrivetrain.bp2p(Pose2d(120.0, 56.0, Rotation2d.fromDegrees(-90.0)), 3.0).with(Wait(0.25)),
                SwerveDrivetrain.bp2p(Pose2d(129.0, 56.0, Rotation2d.fromDegrees(-90.0)), 3.0).with(Wait(0.25)),
            )
        ),

        SwerveDrivetrain.bp2p(Pose2d(129.0, 19.0, Rotation2d.fromDegrees(-90.0)), 3.0).with(Wait(0.25)),

        //changed
        SwerveDrivetrain.forwardTime(0.13, 1.3),

        Wait(0.05),
        Deposit.close(),
        Wait(0.200),
        Parallel(
            Elevator.pidAutoTimeout(VerticalConstants.ElevatorPositions.BOTTOM+1.0, 0.5),
            SwerveDrivetrain.forwardTime(-0.1, 0.25)
        ),
        Wait(0.1),
        Parallel(
            Sequential(
                Elevator.pidAutoTimeout(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE+0.5, 1.5),
                Elevator.waitUntilAboveArm(),
                Parallel(
                    VerticalArm.specimen(),
                    VerticalWrist.specimenPlace(),
                ),

            ),
            Sequential(Wait(0.1), SwerveDrivetrain.bp2p(Pose2d(69.0, 34.5, Rotation2d.fromDegrees(90.0)), 1.5)),
        ),

        Lambda("print-path").setInit{Telemetry.path = third},

        Wait(0.1),
        Deposit.open(),
        Wait(0.1),

        Parallel(
            SwerveDrivetrain.bp2p(Pose2d(120.0, 12.0, Rotation2d.fromDegrees(90.0)), 3.0),
            Wait(0.6),
            Sequential(
                Wait(0.4),
                VerticalArm.intake(),
                VerticalWrist.intake(),
                Wait(0.7),
                Elevator.pidAuto(VerticalConstants.ElevatorPositions.BOTTOM),
            )
        ),

        Wait(10.0)
        /*
        Wait(0.050),
        Deposit.open(),
        Parallel(
            SwerveDrivetrain.p2p(Pose2d(78.0, 42.0, Rotation2d())),
            VerticalArm.specimen()
        ),

         */

         */
    )

    override fun init() {
        SwerveDrivetrain.setPose(specimenAutoPoses.startPose)
        //SwerveDrivetrain.setPose(startPose)

        VerticalArm.setPosition(VerticalConstants.VerticalArmPositions.AUTO_START)
        Deposit.setPosition(VerticalConstants.DepositPositions.IN)
        VerticalWrist.setPosition(VerticalConstants.VerticalWristPositions.INTAKE)

        SwerveDrivetrain.defaultCommand = SwerveDrivetrain.stopCmd()
        Elevator.defaultCommand = null

        Telemetry.path = first
        Telemetry.put("alliance colour", Globals.AllianceColour.name)
        //Telemetry.points.add( Pose2d(48.0, 0.0, Rotation2d()) )

    }

    override fun init_loop() {

    }

    override fun start() {
        //SwerveDrivetrain.setPose(startPose)

        auto.schedule()

        /*
        Sequential(
            PurePursuitController.followPathCommand(first, 0.1, 0.9),
            Lambda("field print").setInit{Telemetry.path = second},
            Wait(0.2),
            PurePursuitController.followPathCommand(second),
        ).schedule()

         */
        //Sequential(
            //SwerveDrivetrain.alignModules(place),
            //SwerveDrivetrain.p2p(place)
        //).schedule()

    }

    override fun loop() {
        /*
        packet.put("lf delta", delta[0])
        packet.put("rf delta", delta[1])
        packet.put("lr delta", delta[2])
        packet.put("rr delta", delta[3])

         */
        //packet.put("elevator pos", Elevator.getPosition())
        //packet.put("elevator target", Elevator.targetPosition)
        //packet.put("elevator pid atSetPoint 2", Elevator.atSetPoint())
        //FtcDashboard.getInstance().sendTelemetryPacket(packet)

    }

    @Config
    object specimenAutoPoses {
        @JvmField var startPose = Pose2d(78.0, 7.689, Rotation2d.fromDegrees(90.0))
        @JvmField var place = Pose2d(78.0, 39.5, Rotation2d.fromDegrees(90.0))

    }

}