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
import dev.frozenmilk.mercurial.commands.util.IfElse
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.teamcode.commands.Timeout
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.subsystems.Deposit
import org.firstinspires.ftc.teamcode.subsystems.Elevator
import org.firstinspires.ftc.teamcode.subsystems.HorizontalArm
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtension
import org.firstinspires.ftc.teamcode.subsystems.HorizontalWrist
import org.firstinspires.ftc.teamcode.subsystems.Intake
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
        CurvePoint(Pose2d(120.0, 8.0, Rotation2d.fromDegrees(-90.0)), 1.0, 0.1, 6.0),
    ), kSmooth = 0.895, minFollowDistance = 4.5, kFollowDistance = 6.0, kCurvature = 0.080, spacing = 1.5, kPID=0.9, kFF=0.1)

    private val third = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(120.0, 8.0, Rotation2d.fromDegrees(-90.0)), 1.0, 0.5, 6.0),
        CurvePoint(Pose2d(120.0, 60.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(132.0, 57.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(132.0, 8.0, Rotation2d.fromDegrees(-90.0)), 1.0, 0.5, 6.0),
    ), kSmooth = 0.95, kPID=0.9, kFF=0.1, kFollowDistance = 8.0)

    private val fourth = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(132.0, 8.0, Rotation2d.fromDegrees(-90.0)), 1.0, 0.5, 6.0),
        CurvePoint(Pose2d(132.0, 60.0, Rotation2d.fromDegrees(-90.0)), 1.0, 0.5, 6.0),
        CurvePoint(Pose2d(137.0, 57.0, Rotation2d.fromDegrees(-90.0)), 1.0, 0.5, 6.0),
        CurvePoint(Pose2d(137.0, 8.0, Rotation2d.fromDegrees(-90.0)), 1.0, 0.5, 6.0),
    ), kSmooth = 0.95, kPID=0.9, kFF=0.1, kFollowDistance = 8.0)

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
            Sequential(Wait(15.0), Deposit.waitUntilHoldingPiece()),
            Parallel(
                Timeout(PurePursuitController.followPathCommand(second), 15.0),
                Sequential(
                    Wait(0.80),
                    Timeout(verticalRetract, 3.0)
                ),
            ),
        ),

        Timeout(PurePursuitController.followPathCommand(third), 15.0),
        Timeout(PurePursuitController.followPathCommand(fourth, 1000.0), 15.0),

        /*
        Wait(0.05),
        //SwerveDrivetrain.forwardTime(0.12, 1.3),
        Timeout(SwerveDrivetrain.forwardSensor(0.3), 2.0),

         */


        /*


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
                    Timeout(verticalSpecimenPickup, 3.0)
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
                    Timeout(verticalSpecimenPickup, 3.0)
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
            ),
            /*
            Sequential(
                Wait(0.5),
                horizontalExtend
            )
             */
        )


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
        //HorizontalExtension.defaultCommand = HorizontalExtension.hold()

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