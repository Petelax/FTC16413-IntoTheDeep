package org.firstinspires.ftc.teamcode.drive.test

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.dairy.pasteurized.SDKGamepad
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.bindings.BoundGamepad
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Race
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.teamcode.commands.Timeout
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.subsystems.Elevator
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtension
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.VerticalArm
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrivetrain
import org.firstinspires.ftc.teamcode.utils.BulkReads
import org.firstinspires.ftc.teamcode.utils.Globals
import org.firstinspires.ftc.teamcode.utils.LoopTimes
import org.firstinspires.ftc.teamcode.utils.Telemetry

@Mercurial.Attach
@BulkReads.Attach
@LoopTimes.Attach
@Telemetry.Attach

//@SwerveDrivetrain.Attach

//@HorizontalExtension.Attach
//@HorizontalArm.Attach
//@HorizontalWrist.Attach
//@Intake.Attach

@Elevator.Attach
@VerticalArm.Attach
//@VerticalWrist.Attach
//@Deposit.Attach

@TeleOp
class RealClimbTest : OpMode() {
    override fun init() {
        val mechanismGamepad = BoundGamepad(SDKGamepad(gamepad2))
        val driveGamepad = BoundGamepad(SDKGamepad(gamepad1))


        val climb = Sequential(
            Parallel(
                Sequential(
                    Wait(0.100),
                    Parallel(
                        Elevator.pid(VerticalConstants.ElevatorPositions.CLIMB_ONE),
                        Race(
                            null,
                            Elevator.waitUntilSetPoint(VerticalConstants.ElevatorPositions.CLIMB_ONE),
                            Wait(1.0)
                        )
                    )
                ),
                //Wait(0.100).then(Elevator.pid(VerticalConstants.ElevatorPositions.CLIMB_ONE).with(Elevator.waitUntilSetPoint(VerticalConstants.ElevatorPositions.CLIMB_ONE))),
                VerticalArm.sample()
            ),
            Wait(0.1),
            //Lambda("delete").addRequirements(Elevator).setInit{Elevator.defaultCommand = null},
            //Elevator.cancel(),

            //Elevator.driveAndClimb({gamepad2.left_stick_y.toDouble() + if(gamepad1.right_bumper) {-1000.0} else {0.0}}, {if (gamepad1.dpad_up) {1.0} else {0.0} + if (gamepad1.dpad_down) {-1.0} else {0.0}})
        )
        driveGamepad.a.onTrue( climb )

        driveGamepad.b.onTrue(
            Elevator.climb(VerticalConstants.ClimbPositions.L2, VerticalConstants.ElevatorPositions.CLIMB_TWO),
        )

        driveGamepad.y.onTrue(Sequential(
            Elevator.climbUp(VerticalConstants.ClimbPositions.L3, VerticalConstants.ElevatorPositions.CLIMB_THREE),
            VerticalArm.setVerticalArm(VerticalConstants.VerticalArmPositions.AUTO_START)
        ))

        //driveGamepad.a.onTrue(Elevator.drive{-1.0})

        Telemetry.put("alliance colour", Globals.AllianceColour.name)

        Elevator.hardReset()
    }

    override fun start() {
        VerticalArm.setPosition(VerticalConstants.VerticalArmPositions.SPECIMEN)


    }

    override fun loop() {

        Telemetry.put("elevator pos", Elevator.getPosition())
        Telemetry.put("horizontal pos", HorizontalExtension.getPosition())
        Telemetry.put("climber pos", Elevator.getClimbPosition())

        Telemetry.put("intake piece", Intake.getGamePiece().name)

    }

}