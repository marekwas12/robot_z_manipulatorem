package com.example.robotcontroller.ui.screens

import android.app.Activity

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.RoundedCornerShape

import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Card
import androidx.compose.material3.Text

import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue

import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp

import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsCompat

import java.util.Locale


private enum class ManipulatorMode {
    MENU,
    JOINTS,
    IK
}


private val ManualGreen = Color(0xFF2E7D32)
private val ManualBlue = Color(0xFF506AA3)
private val StopRed = Color(0xFFC62828)


@Composable
fun ManipulatorControlScreen(
    jointAnglesDeg: List<Double?>,

    tcpXmm: Double?,
    tcpYmm: Double?,
    tcpZmm: Double?,

    onJointJog: (Int, Int) -> Unit,

    onCommandClick: (String) -> Unit,

    onPresetClick: (String) -> Unit,

    onIkJogClick: (String) -> Unit,

    onDriveClick: () -> Unit,

    onBackClick: () -> Unit
) {

    ManipulatorFullscreenEffect()

    var mode by remember {
        mutableStateOf(ManipulatorMode.MENU)
    }

    when (mode) {

        ManipulatorMode.MENU -> {

            ManualModeMenu(
                onDriveClick = onDriveClick,

                onJointClick = {
                    mode = ManipulatorMode.JOINTS
                },

                onIkClick = {
                    mode = ManipulatorMode.IK
                },

                onHomingClick = {
                    onCommandClick("H")
                },

                onReadyClick = {
                    onPresetClick("ready")
                },

                onPickClick = {
                    onPresetClick("pick")
                },

                onTrunkClick = {
                    onPresetClick("trunk")
                },

                onPionClick = {
                    onPresetClick("pion")
                },

                onBackClick = onBackClick
            )
        }


        ManipulatorMode.JOINTS -> {

            JointControlScreen(
                jointAnglesDeg = jointAnglesDeg,

                onJointJog = onJointJog,

                onCommandClick = onCommandClick,

                onStopClick = {
                    onCommandClick("0")
                    onCommandClick("z")
                },

                onModeBack = {
                    mode = ManipulatorMode.MENU
                }
            )
        }


        ManipulatorMode.IK -> {

            InverseKinematicsScreen(
                tcpXmm = tcpXmm,
                tcpYmm = tcpYmm,
                tcpZmm = tcpZmm,

                onIkJogClick = onIkJogClick,

                onModeBack = {
                    mode = ManipulatorMode.MENU
                }
            )
        }
    }
}


// =============================================================
// PIONOWE MENU TRYBU MANUALNEGO
// =============================================================

@Composable
private fun ManualModeMenu(
    onDriveClick: () -> Unit,
    onJointClick: () -> Unit,
    onIkClick: () -> Unit,
    onHomingClick: () -> Unit,
    onReadyClick: () -> Unit,
    onPickClick: () -> Unit,
    onTrunkClick: () -> Unit,
    onPionClick: () -> Unit,
    onBackClick: () -> Unit
) {

    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(horizontal = 28.dp)
            .padding(
                top = 42.dp,
                bottom = 20.dp
            ),

        horizontalAlignment = Alignment.CenterHorizontally
    ) {

        Text(
            text = "Tryb manualny",
            fontSize = 29.sp,
            fontWeight = FontWeight.Bold
        )

        Spacer(
            modifier = Modifier.height(24.dp)
        )


        ManualMenuButton(
            text = "Jazda",
            onClick = onDriveClick
        )

        Spacer(
            modifier = Modifier.height(12.dp)
        )


        ManualMenuButton(
            text = "Sterowanie przegubowe",
            onClick = onJointClick
        )

        Spacer(
            modifier = Modifier.height(12.dp)
        )


        ManualMenuButton(
            text = "Kinematyka odwrotna",
            onClick = onIkClick
        )

        Spacer(
            modifier = Modifier.height(26.dp)
        )


        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(10.dp)
        ) {

            CompactPresetButton(
                text = "HOMING",
                onClick = onHomingClick,
                modifier = Modifier.weight(1f)
            )

            CompactPresetButton(
                text = "READY",
                onClick = onReadyClick,
                modifier = Modifier.weight(1f)
            )
        }


        Spacer(
            modifier = Modifier.height(10.dp)
        )


        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(10.dp)
        ) {

            CompactPresetButton(
                text = "PICK",
                onClick = onPickClick,
                modifier = Modifier.weight(1f)
            )

            CompactPresetButton(
                text = "BAGAŻNIK",
                onClick = onTrunkClick,
                modifier = Modifier.weight(1f)
            )
        }


        Spacer(
            modifier = Modifier.height(10.dp)
        )


        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(10.dp)
        ) {

            CompactPresetButton(
                text = "PION",
                onClick = onPionClick,
                modifier = Modifier.weight(1f)
            )

            Spacer(
                modifier = Modifier.weight(1f)
            )
        }


        Spacer(
            modifier = Modifier.weight(1f)
        )


        Button(
            onClick = onBackClick,

            modifier = Modifier
                .fillMaxWidth()
                .height(54.dp),

            shape = RoundedCornerShape(15.dp),

            colors = ButtonDefaults.buttonColors(
                // ZMIANA: 80%
                containerColor = ManualBlue.copy(
                    alpha = 1.0f
                )
            )
        ) {

            Text(
                text = "Wróć",
                fontSize = 16.sp,
                fontWeight = FontWeight.Bold
            )
        }
    }
}


@Composable
private fun ManualMenuButton(
    text: String,
    onClick: () -> Unit
) {

    Button(
        onClick = onClick,

        modifier = Modifier
            .fillMaxWidth()
            .height(56.dp),

        shape = RoundedCornerShape(15.dp),

        colors = ButtonDefaults.buttonColors(
            // ZMIANA: 72% -> 80%
            containerColor = ManualGreen.copy(
                alpha = 1.0f
            )
        )
    ) {

        Text(
            text = text,
            fontSize = 17.sp,
            fontWeight = FontWeight.Bold
        )
    }
}


@Composable
private fun CompactPresetButton(
    text: String,
    onClick: () -> Unit,
    modifier: Modifier = Modifier
) {

    Button(
        onClick = onClick,

        modifier = modifier
            .height(48.dp),

        shape = RoundedCornerShape(24.dp),

        colors = ButtonDefaults.buttonColors(
            containerColor = ManualGreen.copy(
                alpha = 1.0f
            )
        )
    ) {

        Text(
            text = text,
            fontSize = 14.sp,
            fontWeight = FontWeight.Bold
        )
    }
}


// =============================================================
// STEROWANIE PRZEGUBOWE - PIONOWO
// =============================================================

@Composable
private fun JointControlScreen(
    jointAnglesDeg: List<Double?>,

    onJointJog: (Int, Int) -> Unit,

    onCommandClick: (String) -> Unit,

    onStopClick: () -> Unit,

    onModeBack: () -> Unit
) {

    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(horizontal = 16.dp)
            .padding(
                top = 34.dp,
                bottom = 14.dp
            ),

        horizontalAlignment = Alignment.CenterHorizontally
    ) {

        Text(
            text = "Sterowanie przegubowe",
            fontSize = 25.sp,
            fontWeight = FontWeight.Bold
        )


        Spacer(
            modifier = Modifier.height(8.dp)
        )


        Text(
            text = "Aktualne kąty przegubów [°] • kroki: ±1°, ±5°, ±10°",
            fontSize = 14.sp,
            color = Color.DarkGray
        )


        Spacer(
            modifier = Modifier.height(14.dp)
        )


        JointControlRow(
            index = 0,
            title = "Oś 1 (obrót podstawy)",
            angleDeg = jointAnglesDeg.getOrNull(0),
            onJointJog = onJointJog
        )


        JointControlRow(
            index = 1,
            title = "Oś 2 (bark)",
            angleDeg = jointAnglesDeg.getOrNull(1),
            onJointJog = onJointJog
        )


        JointControlRow(
            index = 2,
            title = "Oś 3 (łokieć)",
            angleDeg = jointAnglesDeg.getOrNull(2),
            onJointJog = onJointJog
        )


        JointControlRow(
            index = 3,
            title = "Oś 4 (nadgarstek)",
            angleDeg = jointAnglesDeg.getOrNull(3),
            onJointJog = onJointJog
        )


        JointControlRow(
            index = 4,
            title = "Oś 5 (obrót chwytaka)",
            angleDeg = jointAnglesDeg.getOrNull(4),
            onJointJog = onJointJog
        )


        Spacer(
            modifier = Modifier.height(4.dp)
        )


        GripperControlRow(
            onCommandClick = onCommandClick
        )


        Spacer(
            modifier = Modifier.weight(1f)
        )


        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(12.dp)
        ) {

            Button(
                onClick = onStopClick,

                modifier = Modifier
                    .weight(1f)
                    .height(52.dp),

                colors = ButtonDefaults.buttonColors(
                    containerColor = StopRed.copy(
                        alpha = 0.82f
                    )
                )
            ) {

                Text(
                    text = "STOP",
                    fontWeight = FontWeight.Bold
                )
            }


            Button(
                onClick = onModeBack,

                modifier = Modifier
                    .weight(1f)
                    .height(52.dp),

                colors = ButtonDefaults.buttonColors(
                    containerColor = ManualBlue.copy(
                        alpha = 0.78f
                    )
                )
            ) {

                Text(
                    text = "Wróć",
                    fontWeight = FontWeight.Bold
                )
            }
        }
    }
}


// =============================================================
// CHWYTAK
// =============================================================

@Composable
private fun GripperControlRow(
    onCommandClick: (String) -> Unit
) {

    // Serwo chwytaka:
    // k = -1 stopień (otwieranie)
    // l = +1 stopień (zamykanie)
    //
    // ±5 i ±10 wysyłają odpowiednio 5 lub 10 znaków.

    Row(
        modifier = Modifier
            .fillMaxWidth()
            .padding(vertical = 7.dp),

        verticalAlignment = Alignment.CenterVertically,

        horizontalArrangement = Arrangement.SpaceBetween
    ) {

        Row(
            horizontalArrangement = Arrangement.spacedBy(3.dp)
        ) {

            JointStepButton(
                text = "-10"
            ) {
                repeat(10) {
                    onCommandClick("k")
                }
            }


            JointStepButton(
                text = "-5"
            ) {
                repeat(5) {
                    onCommandClick("k")
                }
            }


            JointStepButton(
                text = "-1"
            ) {
                onCommandClick("k")
            }
        }


        Column(
            modifier = Modifier
                .weight(1f)
                .padding(horizontal = 6.dp),

            horizontalAlignment = Alignment.CenterHorizontally
        ) {

            Text(
                text = "Chwytak",
                fontSize = 13.sp,
                fontWeight = FontWeight.SemiBold,
                textAlign = TextAlign.Center
            )


            Text(
                text = "− otwórz  /  + zamknij",
                fontSize = 10.sp,
                color = ManualGreen,
                textAlign = TextAlign.Center
            )
        }


        Row(
            horizontalArrangement = Arrangement.spacedBy(3.dp)
        ) {

            JointStepButton(
                text = "+1"
            ) {
                onCommandClick("l")
            }


            JointStepButton(
                text = "+5"
            ) {
                repeat(5) {
                    onCommandClick("l")
                }
            }


            JointStepButton(
                text = "+10"
            ) {
                repeat(10) {
                    onCommandClick("l")
                }
            }
        }
    }
}


// =============================================================
// WIERSZ STEROWANIA OSIĄ
// =============================================================

@Composable
private fun JointControlRow(
    index: Int,
    title: String,
    angleDeg: Double?,
    onJointJog: (Int, Int) -> Unit
) {

    Row(
        modifier = Modifier
            .fillMaxWidth()
            .padding(vertical = 7.dp),

        verticalAlignment = Alignment.CenterVertically,

        horizontalArrangement = Arrangement.SpaceBetween
    ) {

        Row(
            horizontalArrangement = Arrangement.spacedBy(3.dp)
        ) {

            JointStepButton(
                text = "-10"
            ) {
                onJointJog(index, -10)
            }


            JointStepButton(
                text = "-5"
            ) {
                onJointJog(index, -5)
            }


            JointStepButton(
                text = "-1"
            ) {
                onJointJog(index, -1)
            }
        }


        Column(
            modifier = Modifier
                .weight(1f)
                .padding(horizontal = 6.dp),

            horizontalAlignment = Alignment.CenterHorizontally
        ) {

            Text(
                text = title,
                fontSize = 13.sp,
                fontWeight = FontWeight.SemiBold,
                textAlign = TextAlign.Center
            )


            Text(
                text = angleDeg?.let {
                    String.format(
                        Locale.US,
                        "%+.1f°",
                        it
                    )
                } ?: "---",

                fontSize = 14.sp,
                fontWeight = FontWeight.Bold,
                color = ManualGreen,
                textAlign = TextAlign.Center
            )
        }


        Row(
            horizontalArrangement = Arrangement.spacedBy(3.dp)
        ) {

            JointStepButton(
                text = "+1"
            ) {
                onJointJog(index, +1)
            }


            JointStepButton(
                text = "+5"
            ) {
                onJointJog(index, +5)
            }


            JointStepButton(
                text = "+10"
            ) {
                onJointJog(index, +10)
            }
        }
    }
}


@Composable
private fun JointStepButton(
    text: String,
    onClick: () -> Unit
) {

    Button(
        onClick = onClick,

        modifier = Modifier
            .width(34.dp)
            .height(34.dp),

        shape = RoundedCornerShape(11.dp),

        colors = ButtonDefaults.buttonColors(
            containerColor = ManualGreen.copy(
                alpha = 0.72f
            ),

            contentColor = Color.White
        ),

        contentPadding = PaddingValues(0.dp)
    ) {

        Text(
            text = text,
            fontSize = 10.sp,
            fontWeight = FontWeight.Bold,
            color = Color.White
        )
    }
}


// =============================================================
// KINEMATYKA ODWROTNA - PIONOWO
// =============================================================

@Composable
private fun InverseKinematicsScreen(
    tcpXmm: Double?,
    tcpYmm: Double?,
    tcpZmm: Double?,

    onIkJogClick: (String) -> Unit,

    onModeBack: () -> Unit
) {

    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(horizontal = 22.dp)
            .padding(
                top = 38.dp,
                bottom = 16.dp
            ),

        horizontalAlignment = Alignment.CenterHorizontally
    ) {

        Text(
            text = "Kinematyka odwrotna",
            fontSize = 26.sp,
            fontWeight = FontWeight.Bold
        )


        Spacer(
            modifier = Modifier.height(8.dp)
        )


        Text(
            text = "Aktualna pozycja punktu roboczego chwytaka względem podstawy robota",
            fontSize = 15.sp
        )


        Spacer(
            modifier = Modifier.height(12.dp)
        )


        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {

            TcpValueCard(
                label = "X",
                valueMm = tcpXmm,
                modifier = Modifier.weight(1f)
            )


            TcpValueCard(
                label = "Y",
                valueMm = tcpYmm,
                modifier = Modifier.weight(1f)
            )


            TcpValueCard(
                label = "Z",
                valueMm = tcpZmm,
                modifier = Modifier.weight(1f)
            )
        }


        Spacer(
            modifier = Modifier.height(22.dp)
        )


        AxisJogRow(
            axis = "X",

            onMinus = {
                onIkJogClick("x-")
            },

            onPlus = {
                onIkJogClick("x+")
            }
        )


        AxisJogRow(
            axis = "Y",

            onMinus = {
                onIkJogClick("y-")
            },

            onPlus = {
                onIkJogClick("y+")
            }
        )


        AxisJogRow(
            axis = "Z",

            onMinus = {
                onIkJogClick("z-")
            },

            onPlus = {
                onIkJogClick("z+")
            }
        )


        Spacer(
            modifier = Modifier.weight(1f)
        )


        Button(
            onClick = onModeBack,

            modifier = Modifier
                .fillMaxWidth()
                .height(52.dp),

            colors = ButtonDefaults.buttonColors(
                containerColor = ManualBlue.copy(
                    alpha = 0.78f
                )
            )
        ) {

            Text(
                text = "Wróć",
                fontWeight = FontWeight.Bold
            )
        }
    }
}


// =============================================================
// TCP VALUE
// =============================================================

@Composable
private fun TcpValueCard(
    label: String,
    valueMm: Double?,
    modifier: Modifier = Modifier
) {

    Card(
        modifier = modifier,
        shape = RoundedCornerShape(14.dp)
    ) {

        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(vertical = 12.dp),

            horizontalAlignment = Alignment.CenterHorizontally
        ) {

            Text(
                text = label,
                fontSize = 16.sp,
                fontWeight = FontWeight.Bold
            )


            Text(
                text = valueMm?.let {
                    String.format(
                        Locale.US,
                        "%+.1f mm",
                        it
                    )
                } ?: "---",

                fontSize = 16.sp,
                fontWeight = FontWeight.Bold
            )
        }
    }
}


// =============================================================
// IK AXIS JOG
// =============================================================

@Composable
private fun AxisJogRow(
    axis: String,
    onMinus: () -> Unit,
    onPlus: () -> Unit
) {

    Row(
        modifier = Modifier
            .fillMaxWidth()
            .padding(vertical = 6.dp),

        horizontalArrangement = Arrangement.Center,

        verticalAlignment = Alignment.CenterVertically
    ) {

        Button(
            onClick = onMinus,

            modifier = Modifier.width(108.dp)
        ) {

            Text(
                text = "$axis −5 mm"
            )
        }


        Text(
            text = axis,

            modifier = Modifier.padding(
                horizontal = 20.dp
            ),

            fontSize = 20.sp,
            fontWeight = FontWeight.Bold
        )


        Button(
            onClick = onPlus,

            modifier = Modifier.width(108.dp)
        ) {

            Text(
                text = "$axis +5 mm"
            )
        }
    }
}


// =============================================================
// FULLSCREEN
// =============================================================

@Composable
private fun ManipulatorFullscreenEffect() {

    val context = LocalContext.current
    val activity = context as? Activity

    DisposableEffect(activity) {

        if (activity == null) {

            onDispose { }

        } else {

            val controller =
                WindowCompat.getInsetsController(
                    activity.window,
                    activity.window.decorView
                )

            controller.hide(
                WindowInsetsCompat.Type.systemBars()
            )

            onDispose {

                controller.show(
                    WindowInsetsCompat.Type.systemBars()
                )
            }
        }
    }
}