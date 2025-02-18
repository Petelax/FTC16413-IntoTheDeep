package org.firstinspires.ftc.teamcode.drive.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Race
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.subsystems.Deposit
import org.firstinspires.ftc.teamcode.subsystems.Elevator
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtension
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

@Autonomous(group = "test")
@Disabled
class SampleAuto : OpMode() {
    val verticalSpecimenPickup = Parallel(
        Elevator.waitUntilSetPoint(VerticalConstants.ElevatorPositions.BOTTOM),
        Elevator.pidAuto(VerticalConstants.ElevatorPositions.BOTTOM),
        VerticalArm.specimen(),
        VerticalWrist.specimenPickup(),
    )

    val verticalSpecimenPlace = Parallel(
        Elevator.pidAuto(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE),
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
            SwerveDrivetrain.alignModules(specimenAutoPoses.place),
            Wait(0.3)
        ),
        Parallel(
            SwerveDrivetrain.bp2p(specimenAutoPoses.place, 3.0),
            Race( null,
                Elevator.pidAuto(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE-0.5), Wait(1.0))
        ),
        Wait(0.1),

        Race(
            Wait(1.0),
            SwerveDrivetrain.forward(DrivebaseConstants.Measurements.MAX_VELOCITY*0.17),
        ),
        SwerveDrivetrain.stopCmd(),

        Wait(0.1),
        Deposit.open(),
        Wait(0.1),
        Parallel(
            SwerveDrivetrain.bp2p(Pose2d(108.0, 22.0, Rotation2d.fromDegrees(90.0)), 3.0),
            Sequential(
                Wait(0.15),
                verticalSpecimenPickup
            ),
            Wait(0.15),
        ),

        SwerveDrivetrain.bp2p(Pose2d(108.0, 58.0, Rotation2d.fromDegrees(-85.0)), 3.0),
        SwerveDrivetrain.bp2p(Pose2d(120.0, 57.0, Rotation2d.fromDegrees(-90.0)), 3.0),
        Parallel(
            SwerveDrivetrain.bp2p(Pose2d(120.0, 18.0, Rotation2d.fromDegrees(-90.0)), 3.0),
            Wait(0.15)
        ),

        Wait(10.0)

    )

    override fun init() {
        //SwerveDrivetrain.setPose(startPose)

        VerticalArm.setPosition(VerticalConstants.VerticalArmPositions.AUTO_START)
        Deposit.setPosition(VerticalConstants.DepositPositions.IN)
        VerticalWrist.setPosition(VerticalConstants.VerticalWristPositions.INTAKE)

        SwerveDrivetrain.defaultCommand = null
        Elevator.defaultCommand = null

    }

    override fun init_loop() {

    }

    override fun start() {
        //SwerveDrivetrain.setPose(startPose)

        auto.schedule()

    }

    override fun loop() {
        val packet = TelemetryPacket()
        val pose = SwerveDrivetrain.getPose()
        val delta = SwerveDrivetrain.getDelta()
        packet.put("x", pose.x)
        packet.put("y", pose.y)
        packet.put("heading", pose.heading)
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
        @JvmField var startPose = Pose2d(78.0, 7.5, Rotation2d.fromDegrees(90.0))
        @JvmField var place = Pose2d(78.0, 26.0, Rotation2d.fromDegrees(90.0))

    }

}