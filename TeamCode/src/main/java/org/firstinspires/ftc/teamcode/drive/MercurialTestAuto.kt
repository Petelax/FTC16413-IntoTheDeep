package org.firstinspires.ftc.teamcode.drive

import androidx.core.os.persistableBundleOf
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Race
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.IfElse
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.drive.SpecimenAuto.specimenAutoPoses.place
import org.firstinspires.ftc.teamcode.drive.SpecimenAuto.specimenAutoPoses.startPose
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
import org.firstinspires.ftc.teamcode.utils.LoopTimes

@Mercurial.Attach
@BulkReads.Attach
@LoopTimes.Attach

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
class MercurialTestAuto : OpMode() {
    val verticalSpecimenPickup = Parallel(
        Elevator.waitUntilSetPoint(VerticalConstants.ElevatorPositions.BOTTOM),
        Elevator.pidAuto(VerticalConstants.ElevatorPositions.BOTTOM),
        VerticalArm.specimen(),
        VerticalWrist.specimenPickup(),
    )

    val verticalSpecimenPlace = Parallel(
        Elevator.pidAutoTimeout(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE+0.5, 2.0),
        Elevator.waitUntilAboveArm(),
        Parallel(
            VerticalArm.specimen(),
            VerticalWrist.specimenPlace(),
        ),
    )

    val auto = Sequential(
        Parallel(
            VerticalArm.specimen(),
            VerticalWrist.specimenPlace(),
            Race( null,
                SwerveDrivetrain.alignModules(place),
                Wait(0.65)
            ),
            Wait(0.65)
        ),
        Parallel(
            //SwerveDrivetrain.bp2p(place, 3.0),
            Sequential(
                Wait(0.1),
                Parallel(
                    SwerveDrivetrain.bcp2p(place, 1.5),
                    Wait(0.90),
                    //changed
                )
            ),
            Race( null,
                Elevator.pidAuto(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE+1.0), Wait(1.0))
        ),
        Wait(0.25),

        /*
        Race(
            Wait(0.6),
            SwerveDrivetrain.forward(DrivebaseConstants.Measurements.MAX_VELOCITY*0.17),
        ),
        SwerveDrivetrain.stopCmd(),
         */

        Wait(0.1),
        Deposit.open(),
        Wait(0.2),

        Parallel(
            SwerveDrivetrain.bp2p(Pose2d(103.5, 16.0, Rotation2d.fromDegrees(90.0)), 3.0),
            Sequential(
                Wait(0.30),
                verticalSpecimenPickup
            ),
            Wait(0.15),
        ),

        SwerveDrivetrain.bp2p(Pose2d(108.0, 58.0, Rotation2d.fromDegrees(-85.0)), 3.0),
        SwerveDrivetrain.bp2p(Pose2d(120.0, 57.0, Rotation2d.fromDegrees(-90.0)), 3.0),
        Parallel(
            SwerveDrivetrain.bp2p(Pose2d(120.0, 16.5, Rotation2d.fromDegrees(-90.0)), 3.0),
            Wait(0.8)
        ),

        Race(
            Wait(1.2),
            SwerveDrivetrain.forward(DrivebaseConstants.Measurements.MAX_VELOCITY*0.12),
        ),
        SwerveDrivetrain.stopCmd(),

        Wait(0.05),
        Deposit.close(),
        Wait(0.200),
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
    )

    override fun init() {
        //SwerveDrivetrain.setPose(startPose)
        SwerveDrivetrain.setPose(startPose)

        VerticalArm.setPosition(VerticalConstants.VerticalArmPositions.AUTO_START)
        Deposit.setPosition(VerticalConstants.DepositPositions.IN)
        VerticalWrist.setPosition(VerticalConstants.VerticalWristPositions.INTAKE)

        SwerveDrivetrain.defaultCommand = SwerveDrivetrain.stopCmd()
        Elevator.defaultCommand = null

    }

    override fun init_loop() {

    }

    override fun start() {
        SwerveDrivetrain.setPose(startPose)

        auto.schedule()
        //Sequential(
            //SwerveDrivetrain.alignModules(place),
            //SwerveDrivetrain.p2p(place)
        //).schedule()

    }

    override fun loop() {
        val packet = TelemetryPacket()
        val pose = SwerveDrivetrain.getPose()
        val delta = SwerveDrivetrain.getDelta()
        //packet.put("x", pose.x)
        //packet.put("y", pose.y)
        //packet.put("heading", pose.heading)
        /*
        packet.put("lf delta", delta[0])
        packet.put("rf delta", delta[1])
        packet.put("lr delta", delta[2])
        packet.put("rr delta", delta[3])

         */
        packet.put("elevator pos", Elevator.getPosition())
        packet.put("elevator target", Elevator.targetPosition)
        packet.put("elevator pid atSetPoint 2", Elevator.atSetPoint())
        FtcDashboard.getInstance().sendTelemetryPacket(packet)

    }

    @Config
    object specimenAutoPoses {
        @JvmField var startPose = Pose2d(78.0, 7.375, Rotation2d.fromDegrees(90.0))
        @JvmField var place = Pose2d(78.0, 39.5, Rotation2d.fromDegrees(90.0))

    }

}