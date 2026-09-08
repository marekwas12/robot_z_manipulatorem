package com.example.robotcontroller

import android.content.pm.ActivityInfo
import android.os.Bundle

import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent

import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue

import androidx.lifecycle.viewmodel.compose.viewModel

import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.WindowInsetsControllerCompat

import com.example.robotcontroller.models.AppScreen

import com.example.robotcontroller.ui.screens.AiCubeDetectionScreen
import com.example.robotcontroller.ui.screens.AutonomousDriveScreen
import com.example.robotcontroller.ui.screens.ArucoCubeDetectionScreen
import com.example.robotcontroller.ui.screens.AutonomousMenuScreen
import com.example.robotcontroller.ui.screens.CubePickScreen
import com.example.robotcontroller.ui.screens.DepthMapScreen
import com.example.robotcontroller.ui.screens.DriveControlScreen
import com.example.robotcontroller.ui.screens.SemiAutonomousDriveScreen
import com.example.robotcontroller.ui.screens.HomeScreen
import com.example.robotcontroller.ui.screens.ManipulatorControlScreen
import com.example.robotcontroller.ui.screens.StereoCameraScreen

import com.example.robotcontroller.ui.theme.RobotControllerTheme
import com.example.robotcontroller.viewmodel.MainViewModel


class MainActivity : ComponentActivity() {

    private fun hideSystemBars() {
        WindowCompat.setDecorFitsSystemWindows(
            window,
            false
        )

        val controller =
            WindowCompat.getInsetsController(
                window,
                window.decorView
            )

        controller.hide(
            WindowInsetsCompat.Type.systemBars()
        )

        controller.systemBarsBehavior =
            WindowInsetsControllerCompat
                .BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
    }



    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        hideSystemBars()

        setContent {
            RobotControllerTheme {

                val mainViewModel: MainViewModel = viewModel()

                var currentScreenName by rememberSaveable {
                    mutableStateOf(AppScreen.HOME.name)
                }

                val currentScreen = AppScreen.valueOf(currentScreenName)

                LaunchedEffect(currentScreen) {
                    requestedOrientation =
                        when (currentScreen) {
                            AppScreen.DRIVE,
                            AppScreen.AUTONOMOUS_STEREO,
                            AppScreen.AUTONOMOUS_DEPTH,
                            AppScreen.AUTONOMOUS_AI,
                            AppScreen.AUTONOMOUS_ARUCO,
                            AppScreen.AUTONOMOUS_PICK,
                            AppScreen.AUTONOMOUS_DRIVE -> {
                                ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE
                            }

                            AppScreen.HOME,
                            AppScreen.MANIPULATOR,
                            AppScreen.AUTONOMOUS -> {
                                ActivityInfo.SCREEN_ORIENTATION_PORTRAIT
                            }
                        }
                }

                when (currentScreen) {

                    AppScreen.HOME -> {
                        HomeScreen(
                            isConnected = mainViewModel.isConnected,
                            isConnecting = mainViewModel.isConnecting,
                            upperTemperature = mainViewModel.upperTemperature,
                            lowerTemperature = mainViewModel.lowerTemperature,
                            upperFanEnabled = mainViewModel.upperFanEnabled,
                            lowerFanEnabled = mainViewModel.lowerFanEnabled,

                            onConnectionClick = {
                                mainViewModel.toggleConnection()
                            },

                            onUpperFanClick = {
                                mainViewModel.toggleUpperFan()
                            },

                            onLowerFanClick = {
                                mainViewModel.toggleLowerFan()
                            },

                            onBuzzerChanged = { enabled ->
                                mainViewModel.setBuzzer(enabled)
                            },

                            onManualClick = {
                                currentScreenName = AppScreen.MANIPULATOR.name
                            },

                            onAutonomousClick = {
                                currentScreenName = AppScreen.AUTONOMOUS.name
                            }
                        )
                    }

                    AppScreen.DRIVE -> {
                        DriveControlScreen(
                            cameraBitmap = mainViewModel.cameraBitmap,
                            isConnected = mainViewModel.isConnected,

                            onCommandClick = { command ->
                                mainViewModel.sendManipulatorCommand(command)
                            },

                            onBackClick = {
                                currentScreenName = AppScreen.MANIPULATOR.name
                            }
                        )
                    }

                    AppScreen.MANIPULATOR -> {
                        ManipulatorControlScreen(
                            jointAnglesDeg =
                            mainViewModel.jointAnglesDeg,

                            tcpXmm =
                            mainViewModel.tcpXmm,

                            tcpYmm =
                            mainViewModel.tcpYmm,

                            tcpZmm =
                            mainViewModel.tcpZmm,

                            onJointJog = { jointIndex, direction ->
                                mainViewModel.jogManualJoint(
                                    jointIndex,
                                    direction
                                )
                            },

                            onCommandClick = { command ->
                                mainViewModel.sendManipulatorCommand(command)
                            },

                            onPresetClick = { preset ->
                                mainViewModel.sendManipulatorPreset(preset)
                            },

                            onIkJogClick = { command ->
                                mainViewModel.sendIkJog(command)
                            },

                            onDriveClick = {
                                currentScreenName = AppScreen.DRIVE.name
                            },

                            onBackClick = {
                                currentScreenName = AppScreen.HOME.name
                            }
                        )
                    }

                    AppScreen.AUTONOMOUS -> {
                        AutonomousMenuScreen(
                            onStereoClick = {
                                mainViewModel.stopDepthPreview()
                                mainViewModel.stopAiPreview()
                                mainViewModel.stopArucoPreview()
                                currentScreenName =
                                    AppScreen.AUTONOMOUS_STEREO.name
                            },

                            onDepthClick = {
                                mainViewModel.stopStereoPreview()
                                mainViewModel.stopAiPreview()
                                mainViewModel.stopArucoPreview()
                                currentScreenName =
                                    AppScreen.AUTONOMOUS_DEPTH.name
                            },

                            onAiClick = {
                                mainViewModel.stopStereoPreview()
                                mainViewModel.stopDepthPreview()
                                mainViewModel.stopArucoPreview()
                                currentScreenName =
                                    AppScreen.AUTONOMOUS_AI.name
                            },

                            onArucoClick = {
                                mainViewModel.stopStereoPreview()
                                mainViewModel.stopDepthPreview()
                                mainViewModel.stopAiPreview()
                                currentScreenName =
                                    AppScreen.AUTONOMOUS_ARUCO.name
                            },

                            onCubePickClick = {
                                mainViewModel.stopStereoPreview()
                                mainViewModel.stopDepthPreview()
                                mainViewModel.stopAiPreview()
                                mainViewModel.stopArucoPreview()
                                currentScreenName =
                                    AppScreen.AUTONOMOUS_PICK.name
                            },

                            onAutonomousDriveClick = {
                                mainViewModel.stopStereoPreview()
                                mainViewModel.stopDepthPreview()
                                mainViewModel.stopAiPreview()
                                mainViewModel.stopArucoPreview()
                                currentScreenName =
                                    AppScreen.AUTONOMOUS_DRIVE.name
                            },

                            onBackClick = {
                                mainViewModel.stopStereoPreview()
                                mainViewModel.stopDepthPreview()
                                mainViewModel.stopAiPreview()
                                mainViewModel.stopArucoPreview()
                                currentScreenName = AppScreen.HOME.name
                            }
                        )
                    }

                    AppScreen.AUTONOMOUS_DRIVE -> {

                        LaunchedEffect(Unit) {
                            mainViewModel
                                .startSemiAutonomousMode()
                        }

                        DisposableEffect(Unit) {
                            onDispose {
                                mainViewModel
                                    .stopSemiAutonomousMode()
                            }
                        }

                        SemiAutonomousDriveScreen(
                            aiBitmap =
                            mainViewModel.aiBitmap,

                            isConnected =
                            mainViewModel.isConnected,

                            status =
                            mainViewModel.autonomousDriveStatus,

                            cubePickStatus =
                            mainViewModel.cubePickStatus,

                            depthDistanceM =
                            mainViewModel.depthCenterDistanceM,

                            aiCubeDistanceM =
                            mainViewModel.aiDriveDistanceM,

                            arucoDetected =
                            mainViewModel.arucoDetected,

                            arucoCubeId =
                            mainViewModel.arucoCubeId,

                            arucoDistanceM =
                            mainViewModel.arucoCubeDistanceM,

                            arucoYawDeg =
                            mainViewModel.arucoCubeYawDeg,

                            frontLeftCm =
                            mainViewModel.ultrasonicFrontLeft,

                            frontCenterCm =
                            mainViewModel.ultrasonicFrontCenter,

                            frontRightCm =
                            mainViewModel.ultrasonicFrontRight,

                            leftCm =
                            mainViewModel.ultrasonicLeft,

                            rightCm =
                            mainViewModel.ultrasonicRight,

                            rearCm =
                            mainViewModel.ultrasonicRear,

                            onCommandClick = { command ->
                                mainViewModel
                                    .sendSemiAutonomousCommand(
                                        command
                                    )
                            },

                            onPickClick = {
                                mainViewModel
                                    .startSemiAutonomousPick()
                            },

                            onBackClick = {
                                currentScreenName =
                                    AppScreen.AUTONOMOUS.name
                            }
                        )
                    }


                    AppScreen.AUTONOMOUS_STEREO -> {
                        LaunchedEffect(Unit) {
                            if (!mainViewModel.stereoEnabled) {
                                mainViewModel.toggleStereoPreview()
                            }
                        }

                        StereoCameraScreen(
                            stereoBitmap = mainViewModel.stereoBitmap,
                            stereoEnabled = mainViewModel.stereoEnabled,

                            onStereoClick = {
                                mainViewModel.toggleStereoPreview()
                            },

                            onBackClick = {
                                mainViewModel.stopStereoPreview()
                                currentScreenName = AppScreen.AUTONOMOUS.name
                            }
                        )
                    }

                    AppScreen.AUTONOMOUS_DEPTH -> {
                        LaunchedEffect(Unit) {
                            if (!mainViewModel.depthEnabled) {
                                mainViewModel.toggleDepthPreview()
                            }
                        }

                        DepthMapScreen(
                            depthBitmap = mainViewModel.depthBitmap,
                            depthEnabled = mainViewModel.depthEnabled,

                            onDepthClick = {
                                mainViewModel.toggleDepthPreview()
                            },

                            onBackClick = {
                                mainViewModel.stopDepthPreview()
                                currentScreenName = AppScreen.AUTONOMOUS.name
                            }
                        )
                    }

                    AppScreen.AUTONOMOUS_AI -> {
                        LaunchedEffect(Unit) {
                            if (!mainViewModel.aiEnabled) {
                                mainViewModel.toggleAiPreview()
                            }
                        }

                        AiCubeDetectionScreen(
                            aiBitmap = mainViewModel.aiBitmap,
                            aiEnabled = mainViewModel.aiEnabled,

                            onAiClick = {
                                mainViewModel.toggleAiPreview()
                            },

                            onBackClick = {
                                mainViewModel.stopAiPreview()
                                currentScreenName = AppScreen.AUTONOMOUS.name
                            }
                        )
                    }


                    AppScreen.AUTONOMOUS_PICK -> {

                        LaunchedEffect(Unit) {
                            mainViewModel.startCubePickScreen()
                        }

                        CubePickScreen(
                            arucoBitmap = mainViewModel.arucoBitmap,
                            arucoDetected = mainViewModel.arucoDetected,
                            arucoCubeId = mainViewModel.arucoCubeId,
                            arucoCubeX = mainViewModel.arucoCubeX,
                            arucoCubeY = mainViewModel.arucoCubeY,
                            arucoCubeDistanceM = mainViewModel.arucoCubeDistanceM,
                            arucoCubeYawDeg = mainViewModel.arucoCubeYawDeg,
                            arucoQuality = mainViewModel.arucoQuality,
                            cubePickStatus = mainViewModel.cubePickStatus,
                            cubePlaceStatus = mainViewModel.cubePlaceStatus,

                            onPickClick = {
                                mainViewModel.startCubePickFull()
                            },

                            onPlaceClick = {
                                mainViewModel.sendCubeToTrunk()
                            },

                            onBackClick = {
                                mainViewModel.stopCubePickScreen()
                                currentScreenName =
                                    AppScreen.AUTONOMOUS.name
                            }
                        )
                    }


                    AppScreen.AUTONOMOUS_ARUCO -> {

                        LaunchedEffect(Unit) {
                            mainViewModel.startArucoPreview()
                        }

                        ArucoCubeDetectionScreen(
                            arucoBitmap =
                            mainViewModel.arucoBitmap,

                            arucoEnabled =
                            mainViewModel.arucoEnabled,

                            arucoDetected =
                            mainViewModel.arucoDetected,

                            arucoCubeId =
                            mainViewModel.arucoCubeId,

                            arucoCubeX =
                            mainViewModel.arucoCubeX,

                            arucoCubeY =
                            mainViewModel.arucoCubeY,

                            arucoCubeDistanceM =
                            mainViewModel.arucoCubeDistanceM,

                            arucoBearingDeg =
                            mainViewModel.arucoBearingDeg,

                            arucoCubeYawDeg =
                            mainViewModel.arucoCubeYawDeg,

                            arucoQuality =
                            mainViewModel.arucoQuality,

                            onBackClick = {
                                mainViewModel.stopArucoPreview()

                                currentScreenName =
                                    AppScreen.AUTONOMOUS.name
                            }
                        )
                    }
                }
            }
        }
    }

    override fun onWindowFocusChanged(
        hasFocus: Boolean
    ) {
        super.onWindowFocusChanged(
            hasFocus
        )

        if (hasFocus) {
            hideSystemBars()
        }
    }

}