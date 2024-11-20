package org.firstinspires.ftc.teamcode.commands.subsystems

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import org.firstinspires.ftc.teamcode.subsystems.HorizontalArm
import org.firstinspires.ftc.teamcode.subsystems.HorizontalArmCommand
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtension
import org.firstinspires.ftc.teamcode.subsystems.HorizontalWrist
import org.firstinspires.ftc.teamcode.subsystems.HorizontalWristCommand

class HorizontalExtend(private val horizontalExtension: HorizontalExtension, private val horizontalArm: HorizontalArm, private val horizontalWrist: HorizontalWrist) : SequentialCommandGroup() {
    init {
        addRequirements(horizontalExtension, horizontalArm, horizontalWrist)

        addCommands(
            ParallelCommandGroup(
                HorizontalExtensionPIDCommand(horizontalExtension, HorizontalConstants.HorizontalExtensionPositions.TOP),
                HorizontalArmCommand(horizontalArm, HorizontalConstants.HorizontalArmPositions.OUT),
                HorizontalWristCommand(horizontalWrist, HorizontalConstants.HorizontalWristPositions.OUT)
            )
        )
    }
}