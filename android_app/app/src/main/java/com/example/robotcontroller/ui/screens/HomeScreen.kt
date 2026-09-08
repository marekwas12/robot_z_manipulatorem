package com.example.robotcontroller.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.navigationBarsPadding
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.statusBarsPadding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Card
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import java.util.Locale
import androidx.compose.foundation.layout.Box
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.Shadow
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.text.TextStyle
import androidx.compose.material3.MaterialTheme


private val ActiveGreen = Color(0xFF2E7D32)
private val TitleGreen = Color(0xFF388E3C)
private val InactiveRed = Color(0xFFC62828)
private val ConnectingOrange = Color(0xFFFFA000)
private val BuzzerYellow = Color(0xFFFFC107)

@Composable
fun HomeScreen(
    isConnected: Boolean,
    isConnecting: Boolean,

    upperTemperature: Float?,
    lowerTemperature: Float?,

    upperFanEnabled: Boolean,
    lowerFanEnabled: Boolean,

    onConnectionClick: () -> Unit,
    onUpperFanClick: () -> Unit,
    onLowerFanClick: () -> Unit,
    onBuzzerChanged: (Boolean) -> Unit,

    onManualClick: () -> Unit,
    onAutonomousClick: () -> Unit
) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .statusBarsPadding()
            .navigationBarsPadding()
            .padding(
                start = 16.dp,
                end = 16.dp,
                top = 22.dp,
                bottom = 18.dp
            ),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Spacer(
            modifier = Modifier.height(10.dp)
        )

        TitleHeader()

        Spacer(
            modifier = Modifier.weight(0.55f)
        )

        ConnectionSection(
            isConnected = isConnected,
            isConnecting = isConnecting,
            onConnectionClick = onConnectionClick
        )

        Spacer(
            modifier = Modifier.height(20.dp)
        )

        WorkModeSection(
            isConnected = isConnected,
            onManualClick = onManualClick,
            onAutonomousClick = onAutonomousClick
        )

        Spacer(
            modifier = Modifier.weight(0.15f)
        )

        RobotStatusCard(
            upperTemperature = upperTemperature,
            lowerTemperature = lowerTemperature,
            upperFanEnabled = upperFanEnabled,
            lowerFanEnabled = lowerFanEnabled,
            controlsEnabled = isConnected,
            onUpperFanClick = onUpperFanClick,
            onLowerFanClick = onLowerFanClick,
            onBuzzerChanged = onBuzzerChanged
        )

        Spacer(
            modifier = Modifier.height(10.dp)
        )
    }
}

@Composable
private fun TitleHeader() {
    Surface(
        modifier = Modifier
            .fillMaxWidth()
            .height(95.dp),
        shape = RoundedCornerShape(20.dp),
        color = TitleGreen,
        shadowElevation = 6.dp
    ) {
        Box(
            modifier = Modifier.fillMaxSize(),
            contentAlignment = Alignment.Center
        ) {
            Text(
                text = "Robot mobilny z manipulatorem i systemem wizyjnym do zdalnych operacji",
                modifier = Modifier.padding(horizontal = 16.dp),
                color = Color.White,
                textAlign = TextAlign.Center,
                fontSize = 20.sp,
                lineHeight = 22.sp,
                fontWeight = FontWeight.Bold,
                style = TextStyle(
                    shadow = Shadow(
                        color = Color.Black,
                        offset = Offset(3f, 3f),
                        blurRadius = 6f
                    )
                )
            )
        }
    }
}

@Composable
private fun ConnectionSection(
    isConnected: Boolean,
    isConnecting: Boolean,
    onConnectionClick: () -> Unit
) {
    Column(
        modifier = Modifier.fillMaxWidth(),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Button(
            onClick = onConnectionClick,
            enabled = !isConnecting,
            modifier = Modifier.height(50.dp),
            shape = RoundedCornerShape(15.dp),
            contentPadding = ButtonDefaults.ContentPadding,
            colors = ButtonDefaults.buttonColors(
                containerColor = if (isConnected) {
                    Color(0xFFFFCDD2) // jasny czerwony
                } else {
                    Color(0xFFC8E6C9) // jasny zielony
                },
                contentColor = Color.Black,
                disabledContainerColor = Color(0xFFFFE0B2),
                disabledContentColor = Color.DarkGray
            )
        ) {
            Text(
                text = when {
                    isConnecting -> "Łączenie..."
                    isConnected -> "Rozłącz"
                    else -> "Połącz"
                },
                fontSize = 15.sp,
                fontWeight = FontWeight.Bold
            )
        }

    }
}



@Composable
private fun RobotStatusCard(
    upperTemperature: Float?,
    lowerTemperature: Float?,
    upperFanEnabled: Boolean,
    lowerFanEnabled: Boolean,
    controlsEnabled: Boolean,
    onUpperFanClick: () -> Unit,
    onLowerFanClick: () -> Unit,
    onBuzzerChanged: (Boolean) -> Unit
) {
    Card(
        modifier = Modifier.fillMaxWidth(),
        shape = RoundedCornerShape(18.dp)
    ) {
        Column(
            modifier = Modifier.padding(
                horizontal = 15.dp,
                vertical = 14.dp
            ),
            verticalArrangement = Arrangement.spacedBy(12.dp)
        ) {
            TemperatureRow(
                label = "Temperatura 1 poziom",
                value = lowerTemperature
            )

            TemperatureRow(
                label = "Temperatura 2 poziom",
                value = upperTemperature
            )

            CompactFanRow(
                label = "Wiatraki 1 poziom",
                fanEnabled = lowerFanEnabled,
                controlsEnabled = controlsEnabled,
                onClick = onLowerFanClick
            )

            CompactFanRow(
                label = "Wiatraki 2 poziom",
                fanEnabled = upperFanEnabled,
                controlsEnabled = controlsEnabled,
                onClick = onUpperFanClick
            )

            Row(
                modifier = Modifier.fillMaxWidth(),
                verticalAlignment = Alignment.CenterVertically
            ) {
                Text(
                    text = "Buzzer",
                    modifier = Modifier.weight(1f),
                    fontSize = 15.sp,
                    fontWeight = FontWeight.Medium
                )

                BuzzerButton(
                    enabled = controlsEnabled,
                    onBuzzerChanged = onBuzzerChanged
                )
            }
        }
    }
}

@Composable
private fun TemperatureRow(
    label: String,
    value: Float?
) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.SpaceBetween
    ) {
        Text(
            text = label,
            fontSize = 16.sp,
            fontWeight = FontWeight.Medium
        )

        Text(
            text = formatTemperature(value),
            fontSize = 16.sp,
            fontWeight = FontWeight.Bold,
            color = if (value == null) {
                MaterialTheme.colorScheme.onSurface.copy(alpha = 0.38f)
            } else {
                Color.Black
            }
        )
    }
}

@Composable
private fun CompactFanRow(
    label: String,
    fanEnabled: Boolean,
    controlsEnabled: Boolean,
    onClick: () -> Unit
) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(
            text = label,
            modifier = Modifier.weight(1f),
            fontSize = 15.sp,
            fontWeight = FontWeight.Medium
        )


        Button(
            onClick = onClick,
            enabled = controlsEnabled,
            modifier = Modifier.height(40.dp),
            contentPadding = ButtonDefaults.ContentPadding,
            colors = ButtonDefaults.buttonColors(
                containerColor = if (fanEnabled) {
                    InactiveRed
                } else {
                    ActiveGreen
                }
            )
        ) {
            Text(
                text = if (fanEnabled) {
                    "Wyłącz"
                } else {
                    "Włącz"
                },
                fontSize = 13.sp,
                fontWeight = FontWeight.Bold
            )
        }
    }
}

@Composable
private fun BuzzerButton(
    enabled: Boolean,
    onBuzzerChanged: (Boolean) -> Unit
) {
    Surface(
        modifier = Modifier
            .height(40.dp)
            .pointerInput(enabled) {
                detectTapGestures(
                    onPress = {
                        if (!enabled) {
                            return@detectTapGestures
                        }

                        onBuzzerChanged(true)

                        try {
                            tryAwaitRelease()
                        } finally {
                            onBuzzerChanged(false)
                        }
                    }
                )
            },
        shape = RoundedCornerShape(20.dp),
        color = if (enabled) {
            ActiveGreen
        } else {
            MaterialTheme.colorScheme.onSurface.copy(
                alpha = 0.12f
            )
        }
    ) {
        Box(
            modifier = Modifier.padding(
                horizontal = 24.dp,
                vertical = 10.dp
            ),
            contentAlignment = Alignment.Center
        ) {
            Text(
                text = "Włącz",
                fontSize = 13.sp,
                fontWeight = FontWeight.Bold,
                color = if (enabled) {
                    Color.White
                } else {
                    MaterialTheme.colorScheme.onSurface.copy(
                        alpha = 0.38f
                    )
                }
            )
        }
    }
}

@Composable
private fun WorkModeSection(
    isConnected: Boolean,
    onManualClick: () -> Unit,
    onAutonomousClick: () -> Unit
) {
    Column(
        modifier = Modifier.fillMaxWidth(),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {


        Button(
            onClick = onManualClick,
            enabled = isConnected,
            modifier = Modifier
                .fillMaxWidth()
                .height(50.dp),
            shape = RoundedCornerShape(15.dp),
            colors = ButtonDefaults.buttonColors(
                containerColor = Color.LightGray,
                contentColor = Color.Black,
            )
        ) {
            Text(
                text = "Tryb manualny",
                fontSize = 17.sp,
                fontWeight = FontWeight.Bold
            )
        }

        Spacer(
            modifier = Modifier.height(10.dp)
        )

        Button(
            onClick = onAutonomousClick,
            enabled = isConnected,
            modifier = Modifier
                .fillMaxWidth()
                .height(50.dp),
            shape = RoundedCornerShape(15.dp),
            colors = ButtonDefaults.buttonColors(
                containerColor = Color.LightGray,
                contentColor = Color.Black,
            )
        ) {
            Text(
                text = "Tryb autonomiczny",
                fontSize = 17.sp,
                fontWeight = FontWeight.Bold
            )
        }
    }
}

private fun formatTemperature(
    value: Float?
): String {
    return if (value == null) {
        "°C"
    } else {
        String.format(
            Locale.getDefault(),
            "%.1f °C",
            value
        )
    }
}