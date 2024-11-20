package org.firstinspires.ftc.teamcode.commands.ftclib.subsystems

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import org.firstinspires.ftc.teamcode.subsystems.ftclib.HorizontalArm
import org.firstinspires.ftc.teamcode.subsystems.ftclib.HorizontalArmCommand
import org.firstinspires.ftc.teamcode.subsystems.ftclib.HorizontalWrist
import org.firstinspires.ftc.teamcode.subsystems.ftclib.HorizontalWristCommand
import org.firstinspires.ftc.teamcode.subsystems.ftclib.Intake

class IntakeArmExtend(private val horizontalArm: HorizontalArm, private val horizontalWrist: HorizontalWrist, private val intake: Intake) : SequentialCommandGroup() {
    init {
        addRequirements(horizontalArm, horizontalWrist, intake)

        addCommands(
            ParallelCommandGroup(
                HorizontalArmCommand(horizontalArm, HorizontalConstants.HorizontalArmPositions.OUT),
                HorizontalWristCommand(horizontalWrist, HorizontalConstants.HorizontalWristPositions.OUT),
                org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.IntakeRun(intake)
            )
        )
    }
}