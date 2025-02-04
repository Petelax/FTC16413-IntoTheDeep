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
@HorizontalArm.Attach
@HorizontalWrist.Attach
@Intake.Attach

@Elevator.Attach
@VerticalArm.Attach
@VerticalWrist.Attach
@Deposit.Attach

@Autonomous
class SampleAuto : OpMode() {
    val verticalSample = Parallel(
        Telemetry.putCommand("help", "me"),
        Elevator.pid(VerticalConstants.ElevatorPositions.TOP),
        Elevator.waitUntilSetPoint(VerticalConstants.ElevatorPositions.TOP),
        Sequential(
            Elevator.waitUntilAboveArm(),
            Parallel(
                VerticalArm.sample(),
                VerticalWrist.sample(),
            ),
        ),
    )

    val verticalSpecimenPlace = Parallel(
        Elevator.waitUntilSetPoint(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE),
        Elevator.pid(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE+2.0),
        Sequential(
            Elevator.waitUntilAboveArm(),
            Parallel(
                VerticalArm.specimen(),
                VerticalWrist.specimenPlace(),
            ),
        ),
    )

    val verticalSpecimenPickup = IfElse(
        {VerticalArm.isArmIntake()},
        Sequential(
            Parallel(
                Elevator.waitUntilAboveArm(),
                Elevator.pid(VerticalConstants.ElevatorPositions.ARM_TARGET+0.5)
            ),
            Parallel(
                Wait(VerticalConstants.VerticalArmConstants.intakeToSpecimen),
                VerticalArm.specimen(),
                VerticalWrist.specimenPickup(),
            ),
            Parallel(
                Elevator.waitUntilSetPoint(VerticalConstants.ElevatorPositions.BOTTOM),
                Elevator.pid(VerticalConstants.ElevatorPositions.BOTTOM),
            )
        ),
        Parallel(
            Elevator.waitUntilSetPoint(VerticalConstants.ElevatorPositions.BOTTOM),
            Elevator.pid(VerticalConstants.ElevatorPositions.BOTTOM),
            VerticalArm.specimen(),
            VerticalWrist.specimenPickup(),
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
            IfElse( {VerticalArm.isArmSpecimen()},
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

    val horizontalRetract = Parallel(HorizontalExtension.waitUntilSetPoint(HorizontalConstants.HorizontalExtensionPositions.BOTTOM), HorizontalExtension.pid(HorizontalConstants.HorizontalExtensionPositions.BOTTOM), HorizontalArm.inHorizontalArm(), HorizontalWrist.inHorizontalWrist(), Intake.stopIntake())

    val intake =
        Parallel(
            IfElse ( {HorizontalExtension.getPosition() < 1.0},
                Parallel(HorizontalExtension.waitUntilSetPoint(HorizontalConstants.HorizontalExtensionPositions.CLEAR), HorizontalExtension.pid(HorizontalConstants.HorizontalExtensionPositions.CLEAR), HorizontalArm.outHorizontalArm(), HorizontalWrist.outHorizontalWrist()),
                Parallel(HorizontalArm.outHorizontalArm(), HorizontalWrist.outHorizontalWrist()),
            ),
            VerticalArm.intake(),
            VerticalWrist.intake(),

        )

    val sample = Sequential(
        Timeout(Parallel(HorizontalExtension.waitUntilSetPoint(1.0), HorizontalExtension.pid(HorizontalConstants.HorizontalExtensionPositions.BOTTOM), HorizontalArm.inHorizontalArm(), HorizontalWrist.inHorizontalWrist(), Intake.stopIntake()), 1.5),

        IfElse(
            {HorizontalExtension.getPosition() > 1.0},
            Parallel(
                Sequential(
                    HorizontalExtension.waitUntilSetPoint(HorizontalConstants.HorizontalExtensionPositions.BOTTOM),
                    HorizontalExtension.spin(-0.3),
                ),
                Wait(0.250),
            ),
            Sequential()
        ),
        HorizontalExtension.spin(-0.4),
        Wait(0.01),
        Race( null,
            Intake.spinUntilHolding(),
            Wait(0.400),
        ),
        Deposit.halfClose(),
        Intake.runIntake(),
        Wait(0.05),
        Intake.stopIntake(),

        Deposit.close(),
        HorizontalExtension.spin(0.0),
        Wait(0.10),
        verticalSample
    )

    private val first = PurePursuitController.waypointsToPath(listOf(
        CurvePoint(Pose2d(29.75, 7.375, Rotation2d.fromDegrees(180.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(26.09, 12.12, Rotation2d.fromDegrees(180.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(20.0, 20.0, Rotation2d.fromDegrees(-135.0)), 1.0, 1.0, 6.0),
    ), kSmooth = 0.95, kPID = 0.9, kFF = 0.1)

    private val second = listOf(
        CurvePoint(Pose2d(20.0, 20.0, Rotation2d.fromDegrees(-135.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(20.0, 20.0, Rotation2d.fromDegrees(-96.0)), 1.0, 1.0, 6.0),
    )

    private val third = listOf(
        CurvePoint(Pose2d(20.0, 20.0, Rotation2d.fromDegrees(-96.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(20.0, 20.0, Rotation2d.fromDegrees(-135.0)), 1.0, 1.0, 6.0),
    )

    private val fourth = listOf(
        CurvePoint(Pose2d(20.0, 20.0, Rotation2d.fromDegrees(-135.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(20.0, 20.0, Rotation2d.fromDegrees(-75.0)), 1.0, 1.0, 6.0),
    )

    private val fifth = listOf(
        CurvePoint(Pose2d(20.0, 20.0, Rotation2d.fromDegrees(-75.0)), 1.0, 1.0, 6.0),
        CurvePoint(Pose2d(20.0, 20.0, Rotation2d.fromDegrees(-135.0)), 1.0, 1.0, 6.0),
    )

    val auto = Sequential(
        Parallel(
            Sequential(
                Wait(0.2),
                Timeout(verticalSample, 2.0),
            ),
            Timeout(PurePursuitController.followPathCommand(first), 2.0),
        ),
        Wait(0.2),
        verticalRetract,
        Parallel(
            Timeout(PurePursuitController.followPathCommand(second), 2.0),
            Race(
                Timeout(Intake.runIntakeStopping().then(Intake.backDrive()), 4.0),
                Sequential(
                    Timeout(intake, 1.0),
                    HorizontalExtension.spin(0.3),
                    Wait(1.5),
                    HorizontalExtension.spin(0.0),
                ),
            )
        ),
        Parallel(
            Timeout(PurePursuitController.followPathCommand(third), 2.0),
            sample,
        ),
        verticalRetract,
        Parallel(
            Timeout(PurePursuitController.followPathCommand(fourth), 2.0),
            Race(
                Timeout(Intake.runIntakeStopping().then(Intake.backDrive()), 4.0),
                Sequential(
                    Timeout(intake, 1.0),
                    HorizontalExtension.spin(0.4),
                    Wait(3.0),
                    HorizontalExtension.spin(0.0),
                ),
            )
        ),
        Parallel(
            Timeout(PurePursuitController.followPathCommand(fifth), 2.0),
            sample,
        ),
        verticalRetract,
    )

    override fun init() {
        SwerveDrivetrain.setPose(sampleAutoPoses.startPose)
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
    object sampleAutoPoses {
        @JvmField var startPose = Pose2d(29.75, 7.375, Rotation2d.fromDegrees(180.0))
        //@JvmField var place = Pose2d(78.0, 39.5, Rotation2d.fromDegrees(90.0))

    }

}