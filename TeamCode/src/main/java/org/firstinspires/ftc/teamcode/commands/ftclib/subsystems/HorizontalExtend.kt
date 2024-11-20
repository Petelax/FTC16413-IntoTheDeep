package org.firstinspires.ftc.teamcode.commands.ftclib.subsystems

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import org.firstinspires.ftc.teamcode.subsystems.ftclib.HorizontalArm
import org.firstinspires.ftc.teamcode.subsystems.ftclib.HorizontalArmCommand
import org.firstinspires.ftc.teamcode.subsystems.ftclib.HorizontalExtension
import org.firstinspires.ftc.teamcode.subsystems.ftclib.HorizontalWrist
import org.firstinspires.ftc.teamcode.subsystems.ftclib.HorizontalWristCommand

class HorizontalExtend(private val horizontalExtension: HorizontalExtension, private val horizontalArm: HorizontalArm, private val horizontalWrist: HorizontalWrist) : SequentialCommandGroup() {
    init {
        addRequirements(horizontalExtension, horizontalArm, horizontalWrist)

        addCommands(
            ParallelCommandGroup(
                org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.HorizontalExtensionPIDCommand(
                    horizontalExtension,
                    HorizontalConstants.HorizontalExtensionPositions.TOP
                ),
                HorizontalArmCommand(horizontalArm, HorizontalConstants.HorizontalArmPositions.OUT),
                HorizontalWristCommand(horizontalWrist, HorizontalConstants.HorizontalWristPositions.OUT)
            )
        )
    }
}