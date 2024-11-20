package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.dairy.pasteurized.SDKGamepad
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.bindings.BoundGamepad
import dev.frozenmilk.mercurial.commands.groups.Parallel
import org.firstinspires.ftc.teamcode.commands.ElevatorPID
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
class MercurialTeleOp : OpMode() {
    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val mechanismGamepad = BoundGamepad(SDKGamepad(gamepad2))
        val driveGamepad = BoundGamepad(SDKGamepad(gamepad1))

        //mechanismGamepad.dpadUp.onTrue(Parallel(VerticalArm.sample(), VerticalWrist.sample(), Deposit.close()))
        //mechanismGamepad.dpadDown.onTrue(Parallel(VerticalArm.intake(), VerticalWrist.intake(), Deposit.open()))
        mechanismGamepad.dpadUp.onTrue(Elevator.pid(VerticalConstants.ElevatorPositions.TOP))
        mechanismGamepad.dpadDown.onTrue(Elevator.pid(VerticalConstants.ElevatorPositions.BOTTOM))

        mechanismGamepad.a.onTrue(HorizontalWrist.inHorizontalWrist())
        mechanismGamepad.y.onTrue(HorizontalWrist.outHorizontalWrist())

        mechanismGamepad.x.onTrue(Parallel(HorizontalArm.inHorizontalArm(), HorizontalWrist.inHorizontalWrist(), Intake.stopIntake()))
        mechanismGamepad.b.onTrue(Parallel(HorizontalArm.outHorizontalArm(), HorizontalWrist.outHorizontalWrist(), Intake.runIntake()))

        mechanismGamepad.rightBumper.onTrue(Deposit.open())
        mechanismGamepad.leftBumper.onTrue(Deposit.close())

    }

    override fun loop() {

    }

}