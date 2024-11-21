package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.IfElse
import dev.frozenmilk.mercurial.commands.util.Wait
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
import org.firstinspires.ftc.teamcode.utils.LoopTimes

@Mercurial.Attach
@BulkReads.Attach
@LoopTimes.Attach

@SwerveDrivetrain.Attach

@HorizontalExtension.Attach
@HorizontalArm.Attach
@HorizontalWrist.Attach
@Intake.Attach

@Elevator.Attach
@VerticalArm.Attach
@VerticalWrist.Attach
@Deposit.Attach

@TeleOp
class SpecimenAuto : OpMode() {
    override fun init() {
        val auto = Sequential(

        )


    }

    override fun loop() {
        val packet = TelemetryPacket()
        val pose = SwerveDrivetrain.getPose()
        packet.put("x", pose.x)
        packet.put("y", pose.y)
        packet.put("heading", pose.heading)
        packet.put("elevator pos", Elevator.getPosition())
        packet.put("elevator target", Elevator.targetPosition)
        packet.put("elevator pid atSetPoint 2", Elevator.atSetPoint())
        FtcDashboard.getInstance().sendTelemetryPacket(packet)

    }

}