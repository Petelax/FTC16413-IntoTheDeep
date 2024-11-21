package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.dairy.pasteurized.SDKGamepad
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.bindings.BoundGamepad
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
class MercurialTeleOp : OpMode() {
    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val mechanismGamepad = BoundGamepad(SDKGamepad(gamepad2))
        val driveGamepad = BoundGamepad(SDKGamepad(gamepad1))

        //mechanismGamepad.dpadUp.onTrue(Parallel(VerticalArm.sample(), VerticalWrist.sample(), Deposit.close()))
        //mechanismGamepad.dpadDown.onTrue(Parallel(VerticalArm.intake(), VerticalWrist.intake(), Deposit.open()))

        val verticalSample = Parallel(
            Lambda("print").setInit{telemetry.addLine("help me"); telemetry.update()},
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
            Elevator.pid(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE),
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
                    Elevator.pid(VerticalConstants.ElevatorPositions.ARM)
                ),
                Parallel(
                    Wait(VerticalConstants.VerticalArmConstants.intakeToSpecimen),
                    VerticalArm.specimen(),
                    VerticalWrist.specimenPickup(),
                ),
                Elevator.pid(VerticalConstants.ElevatorPositions.BOTTOM),
            ),
            Parallel(
                Elevator.pid(VerticalConstants.ElevatorPositions.BOTTOM),
                VerticalArm.specimen(),
                VerticalWrist.specimenPickup(),
            ),
        )

        val verticalRetract = IfElse( {!VerticalArm.isArmIntake()},
            Sequential(
                IfElse( {Elevator.getPosition() < VerticalConstants.ElevatorPositions.ARM},
                    Sequential(
                        Elevator.pid(VerticalConstants.ElevatorPositions.ARM),
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

        val horizontalRetract = Parallel(HorizontalExtension.pid(HorizontalConstants.HorizontalExtensionPositions.BOTTOM), HorizontalArm.inHorizontalArm(), HorizontalWrist.inHorizontalWrist(), Intake.stopIntake())

        val sample = Sequential(
            horizontalRetract,
            Parallel(
                HorizontalExtension.waitUntilSetPoint(HorizontalConstants.HorizontalExtensionPositions.BOTTOM),
                Wait(0.500),
            ),
            Deposit.halfClose(),
            Intake.runIntake(),
            Wait(0.400),
            Intake.stopIntake(),
            Deposit.close(),
            verticalSample
        )

        val climb = Sequential(

            Elevator.pid(VerticalConstants.ElevatorPositions.CLIMB_ONE),
            Elevator.waitUntilSetPoint(VerticalConstants.ElevatorPositions.CLIMB_ONE),

            Wait(0.250),

            Elevator.pid(VerticalConstants.ElevatorPositions.CLIMB_TWO),
            Elevator.waitUntilSetPoint(VerticalConstants.ElevatorPositions.CLIMB_TWO),

            Wait(0.250),

            Elevator.pid(VerticalConstants.ElevatorPositions.CLIMB_THREE),
            Elevator.waitUntilSetPoint(VerticalConstants.ElevatorPositions.CLIMB_THREE),

            Wait(0.250),

            Elevator.pid(VerticalConstants.ElevatorPositions.BOTTOM),

        )

        mechanismGamepad.dpadUp.onTrue(Sequential(Parallel(Intake.runIntake(), Wait(0.400), Intake.stopIntake()), verticalSample))
        mechanismGamepad.dpadDown.onTrue(verticalRetract)

        mechanismGamepad.dpadLeft.onTrue(verticalSpecimenPickup)
        mechanismGamepad.dpadRight.onTrue(verticalSpecimenPlace)

        mechanismGamepad.y.onTrue(sample)

        mechanismGamepad.x.onTrue(Parallel(HorizontalExtension.pid(HorizontalConstants.HorizontalExtensionPositions.BOTTOM), HorizontalArm.inHorizontalArm(), HorizontalWrist.inHorizontalWrist(), Intake.stopIntake()))
        mechanismGamepad.b.onTrue(
            IfElse ( {HorizontalExtension.getPosition() < 0.5},
                Parallel(HorizontalExtension.pid(HorizontalConstants.HorizontalExtensionPositions.CLEAR), HorizontalArm.outHorizontalArm(), HorizontalWrist.outHorizontalWrist()),
                Parallel(HorizontalArm.outHorizontalArm(), HorizontalWrist.outHorizontalWrist()),
            )
        )
        mechanismGamepad.b.onTrue(Intake.runIntakeStopping().then(Intake.backDrive()))

        mechanismGamepad.a.onTrue(Parallel(HorizontalExtension.pid(HorizontalConstants.HorizontalExtensionPositions.BOTTOM), HorizontalArm.setPositionCommand(HorizontalConstants.HorizontalArmPositions.MID)))

        mechanismGamepad.rightBumper.onTrue(Deposit.open())
        mechanismGamepad.leftBumper.onTrue(Deposit.close())

        //telemetry.addData("state", mechanismGamepad.rightTrigger.state )
        val intakeSpeed = {mechanismGamepad.rightTrigger.state - mechanismGamepad.leftTrigger.state}
        mechanismGamepad.rightTrigger.conditionalBindState().greaterThan(0.05).bind().onTrue(Lambda("clear").addRequirements(Intake))
        mechanismGamepad.leftTrigger.conditionalBindState().greaterThan(0.05).bind().onTrue(Lambda("clear").addRequirements(Intake))

        //mechanismGamepad.rightTrigger.conditionalBindState().greaterThan(0.05).bind()
        //mechanismGamepad.leftTrigger.conditionalBindState().greaterThan(0.05).bind()

        val elevatorSpeed = {mechanismGamepad.leftStickY.state}
        mechanismGamepad.leftStickY.conditionalBindState().lessThan(-0.05).bind().onTrue(Elevator.disableController())
        mechanismGamepad.leftStickY.conditionalBindState().greaterThan(0.05).bind().onTrue(Elevator.disableController())

        val hev = {mechanismGamepad.rightStickY.state}
        mechanismGamepad.rightStickY.conditionalBindState().lessThan(-0.05).bind().whileTrue(HorizontalExtension.drive(hev)).onFalse(HorizontalExtension.drive{0.0})
        mechanismGamepad.rightStickY.conditionalBindState().greaterThan(0.05).bind().whileTrue(HorizontalExtension.drive(hev)).onFalse(HorizontalExtension.drive{0.0})

    }

    override fun loop() {
        val packet = TelemetryPacket()
        //packet.put("elevator pid speed", Elevator.controller.velocity)
        //packet.put("elevator pid state", Elevator.controller.state)
        packet.put("elevator pos", Elevator.getPosition())
        //packet.put("elevator pid atSetPoint", Elevator.controller.finished())
        packet.put("elevator pid atSetPoint 2", Elevator.atSetPoint())
        FtcDashboard.getInstance().sendTelemetryPacket(packet)

    }

}