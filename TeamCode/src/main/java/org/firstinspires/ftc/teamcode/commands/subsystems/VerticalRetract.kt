package org.firstinspires.ftc.teamcode.commands.subsystems

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.subsystems.Deposit
import org.firstinspires.ftc.teamcode.subsystems.DepositCommand
import org.firstinspires.ftc.teamcode.subsystems.Elevator
import org.firstinspires.ftc.teamcode.subsystems.HorizontalArm
import org.firstinspires.ftc.teamcode.subsystems.HorizontalArmCommand
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtension
import org.firstinspires.ftc.teamcode.subsystems.HorizontalWrist
import org.firstinspires.ftc.teamcode.subsystems.HorizontalWristCommand
import org.firstinspires.ftc.teamcode.subsystems.VerticalArm
import org.firstinspires.ftc.teamcode.subsystems.VerticalArmCommand
import org.firstinspires.ftc.teamcode.subsystems.VerticalWrist
import org.firstinspires.ftc.teamcode.subsystems.VerticalWristCommand

class VerticalRetract(private val elevator: Elevator, private val verticalArm: VerticalArm, private val verticalWrist: VerticalWrist, private val deposit: Deposit) : SequentialCommandGroup() {
    init {
        addRequirements(elevator, verticalArm, verticalWrist, deposit)

        addCommands(
            ParallelCommandGroup(
                ElevatorPIDCommand(elevator, VerticalConstants.ElevatorPositions.LOWER_LIMIT),
                VerticalArmCommand(verticalArm, VerticalConstants.VerticalArmPositions.INTAKE),
                VerticalWristCommand(verticalWrist, VerticalConstants.VerticalWristPositions.INTAKE),
                DepositCommand(deposit, VerticalConstants.DepositPositions.OUT)
            )
        )
    }
}