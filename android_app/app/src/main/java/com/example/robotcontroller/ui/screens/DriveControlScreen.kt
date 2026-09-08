package com.example.robotcontroller.ui.screens

import android.app.Activity
import android.graphics.Bitmap

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape

import androidx.compose.material3.Surface
import androidx.compose.material3.Text

import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect

import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.alpha
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.graphics.graphicsLayer
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp

import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsCompat

import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch


private val ControlPanelWidth = 238.dp
private val ControlPanelHeight = 108.dp


// =============================================================
// GŁÓWNY EKRAN JAZDY
// =============================================================

@Composable
fun DriveControlScreen(

    cameraBitmap: Bitmap?,

    isConnected: Boolean,

    onCommandClick: (
        String
    ) -> Unit,

    onBackClick: () -> Unit

) {

    FullscreenEffect()


    Box(
        modifier =
        Modifier.fillMaxSize()
    ) {

        CameraBackground(
            cameraBitmap =
            cameraBitmap,

            isConnected =
            isConnected
        )


        // =====================================================
        // LEWA STRONA - JAZDA
        // =====================================================

        DriveButtons(
            onCommandClick =
            onCommandClick,

            modifier =
            Modifier
                .align(
                    Alignment.CenterStart
                )
                .padding(
                    start =
                    22.dp
                )
        )


        // =====================================================
        // PRAWA STRONA
        //
        // Kamera i Skręt kół mają dokładnie taki sam rozmiar.
        // =====================================================

        Column(
            modifier =
            Modifier
                .align(
                    Alignment.CenterEnd
                )
                .padding(
                    end =
                    22.dp
                ),

            verticalArrangement =
            Arrangement.spacedBy(
                14.dp
            ),

            horizontalAlignment =
            Alignment.End
        ) {

            CameraTiltButtons(
                onCommandClick =
                onCommandClick,

                modifier =
                Modifier
                    .width(
                        ControlPanelWidth
                    )
                    .height(
                        ControlPanelHeight
                    )
            )


            SteeringButtons(
                onCommandClick =
                onCommandClick,

                modifier =
                Modifier
                    .width(
                        ControlPanelWidth
                    )
                    .height(
                        ControlPanelHeight
                    )
            )
        }


        // =====================================================
        // PRAWY DOLNY RÓG
        // STOP obok WRÓĆ.
        // =====================================================

        Row(
            modifier =
            Modifier
                .align(
                    Alignment.BottomEnd
                )
                .padding(
                    end =
                    22.dp,

                    bottom =
                    14.dp
                ),

            horizontalArrangement =
            Arrangement.spacedBy(
                12.dp
            ),

            verticalAlignment =
            Alignment.CenterVertically
        ) {

            OverlayButton(
                text =
                "STOP",

                onClick = {

                    onCommandClick(
                        "0"
                    )

                    onCommandClick(
                        "z"
                    )
                }
            )


            OverlayButton(
                text =
                "Wróć",

                onClick = {

                    onCommandClick(
                        "0"
                    )

                    onBackClick()
                }
            )
        }
    }
}


// =============================================================
// JAZDA PÓŁAUTONOMICZNA
//
// Sterowanie jest takie samo jak w manualu:
//   1/2 = jazda, 0 = stop,
//   i/u = skręt,
//   t/y = kamera.
//
// Różnica: komendy idą przez autonomous_drive_node, który pilnuje
// HC-SR04 i depth < 10 cm.
// =============================================================

@Composable
fun SemiAutonomousDriveScreen(

    aiBitmap: Bitmap?,

    isConnected: Boolean,
    status: String,
    cubePickStatus: String,

    depthDistanceM: Float?,
    aiCubeDistanceM: Float?,

    arucoDetected: Boolean,
    arucoCubeId: Int?,
    arucoDistanceM: Float?,
    arucoYawDeg: Float?,

    frontLeftCm: Float?,
    frontCenterCm: Float?,
    frontRightCm: Float?,
    leftCm: Float?,
    rightCm: Float?,
    rearCm: Float?,

    onCommandClick: (String) -> Unit,
    onPickClick: () -> Unit,
    onBackClick: () -> Unit

) {

    FullscreenEffect()

    Box(
        modifier =
        Modifier
            .fillMaxSize()
            .background(
                Color.White
            )
    ) {

        // =====================================================
        // OBRAZ: TYLKO AI DETECTION CUBE
        // Boki pozostają białe dzięki ContentScale.Fit.
        // =====================================================

        Box(
            modifier =
            Modifier
                .fillMaxSize()
                .background(
                    Color.White
                ),

            contentAlignment =
            Alignment.Center
        ) {

            if (aiBitmap != null) {

                Image(
                    bitmap =
                    aiBitmap
                        .asImageBitmap(),

                    contentDescription =
                    "AI detection cube",

                    modifier =
                    Modifier
                        .fillMaxSize()
                        .graphicsLayer {
                            rotationZ = 0f
                        },

                    contentScale =
                    ContentScale.Fit
                )

            } else {

                Text(
                    text =
                    if (!isConnected) {
                        "Brak połączenia z robotem"
                    } else {
                        "Czekam na AI detection cube..."
                    },

                    color =
                    Color.Black,

                    fontSize =
                    18.sp,

                    fontWeight =
                    FontWeight.Bold
                )
            }
        }


        // =====================================================
        // LEWA STRONA - JAZDA
        // =====================================================

        DriveButtons(
            onCommandClick =
            onCommandClick,

            modifier =
            Modifier
                .align(
                    Alignment.CenterStart
                )
                .padding(
                    start =
                    18.dp
                )
        )


        // =====================================================
        // PRAWA STRONA - KAMERA + SKRĘT
        // =====================================================

        Column(
            modifier =
            Modifier
                .align(
                    Alignment.CenterEnd
                )
                .padding(
                    end =
                    18.dp
                ),

            verticalArrangement =
            Arrangement.spacedBy(
                12.dp
            ),

            horizontalAlignment =
            Alignment.End
        ) {

            CameraTiltButtons(
                onCommandClick =
                onCommandClick,

                modifier =
                Modifier
                    .width(
                        ControlPanelWidth
                    )
                    .height(
                        ControlPanelHeight
                    )
            )


            SteeringButtons(
                onCommandClick =
                onCommandClick,

                modifier =
                Modifier
                    .width(
                        ControlPanelWidth
                    )
                    .height(
                        ControlPanelHeight
                    )
            )
        }


        // =====================================================
        // PANEL ODCZYTÓW
        // =====================================================

        SemiAutonomousSensorPanel(
            modifier =
            Modifier
                .align(
                    Alignment.TopCenter
                )
                .padding(
                    top =
                    8.dp
                ),

            status =
            status,

            depthDistanceM =
            depthDistanceM,

            aiCubeDistanceM =
            aiCubeDistanceM,

            arucoDetected =
            arucoDetected,

            arucoCubeId =
            arucoCubeId,

            arucoDistanceM =
            arucoDistanceM,

            arucoYawDeg =
            arucoYawDeg,

            frontLeftCm =
            frontLeftCm,

            frontCenterCm =
            frontCenterCm,

            frontRightCm =
            frontRightCm,

            leftCm =
            leftCm,

            rightCm =
            rightCm,

            rearCm =
            rearCm
        )


        // =====================================================
        // DÓŁ - PICK + WIDOK
        // =====================================================

        Row(
            modifier =
            Modifier
                .align(
                    Alignment.BottomCenter
                )
                .padding(
                    end =
                    55.dp,
                    bottom =
                    14.dp
                ),

            verticalAlignment =
            Alignment.CenterVertically
        ) {

            OverlayButton(
                text =
                "Podnoszenie\nkostki",

                onClick = {
                    onCommandClick(
                        "0"
                    )
                    onPickClick()
                }
            )
        }


        // =====================================================
        // WRÓĆ
        // =====================================================

        Row(
            modifier =
            Modifier
                .align(
                    Alignment.BottomEnd
                )
                .padding(
                    end =
                    18.dp,

                    bottom =
                    14.dp
                ),

            horizontalArrangement =
            Arrangement.spacedBy(
                10.dp
            )
        ) {

            OverlayButton(
                text =
                "Wróć",

                onClick = {
                    onCommandClick(
                        "0"
                    )
                    onBackClick()
                }
            )
        }
    }
}


@Composable
private fun SemiAutonomousSensorPanel(

    modifier: Modifier,

    status: String,

    depthDistanceM: Float?,
    aiCubeDistanceM: Float?,

    arucoDetected: Boolean,
    arucoCubeId: Int?,
    arucoDistanceM: Float?,
    arucoYawDeg: Float?,

    frontLeftCm: Float?,
    frontCenterCm: Float?,
    frontRightCm: Float?,
    leftCm: Float?,
    rightCm: Float?,
    rearCm: Float?

) {

    Surface(
        modifier =
        modifier
            .width(
                430.dp
            )
            .alpha(
                0.86f
            ),

        shape =
        RoundedCornerShape(
            14.dp
        ),

        tonalElevation =
        6.dp
    ) {

        Column(
            modifier =
            Modifier
                .fillMaxWidth()
                .padding(
                    horizontal =
                    10.dp,

                    vertical =
                    6.dp
                ),

            horizontalAlignment =
            Alignment.CenterHorizontally
        ) {

            Text(
                text =
                status,

                color =
                Color.Black,

                fontSize =
                11.sp,

                fontWeight =
                FontWeight.Bold,

                textAlign =
                TextAlign.Center
            )


            Text(
                text =
                "DEPTH: ${formatMetersAsCm(depthDistanceM)}   |   " +
                        "KOSTKA AI: ${formatMetersAsCm(aiCubeDistanceM)}   |   " +
                        "ARUCO: " +
                        if (
                            arucoDetected
                            &&
                            arucoDistanceM != null
                        ) {
                            "ID ${arucoCubeId ?: -1} | " +
                                    formatMetersAsCm(
                                        arucoDistanceM
                                    ) +
                                    " | yaw " +
                                    (
                                            arucoYawDeg
                                                ?.let {
                                                    "%+.1f°".format(it)
                                                }
                                                ?: "---"
                                            )
                        } else {
                            "---"
                        },

                color =
                Color.Black,

                fontSize =
                10.sp,

                textAlign =
                TextAlign.Center
            )


            Text(
                text =
                "HC-SR04 [cm]  " +
                        "FL=${formatCm(frontLeftCm)}  " +
                        "FC=${formatCm(frontCenterCm)}  " +
                        "FR=${formatCm(frontRightCm)}  " +
                        "L=${formatCm(leftCm)}  " +
                        "R=${formatCm(rightCm)}  " +
                        "TYŁ=${formatCm(rearCm)}",

                color =
                Color.Black,

                fontSize =
                10.sp,

                fontWeight =
                FontWeight.Bold,

                textAlign =
                TextAlign.Center
            )
        }
    }
}


private fun formatCm(
    value: Float?
): String {

    return value
        ?.takeIf {
            it.isFinite()
                    &&
                    it > 0f
        }
        ?.let {
            "%.1f".format(it)
        }
        ?: "---"
}


private fun formatMetersAsCm(
    value: Float?
): String {

    return value
        ?.takeIf {
            it.isFinite()
                    &&
                    it > 0f
        }
        ?.let {
            "%.1f cm".format(
                it * 100f
            )
        }
        ?: "---"
}


// =============================================================
// TŁO KAMERY
// =============================================================

@Composable
private fun CameraBackground(

    cameraBitmap: Bitmap?,

    isConnected: Boolean

) {

    Box(
        modifier =
        Modifier.fillMaxSize(),

        contentAlignment =
        Alignment.Center
    ) {

        if (
            cameraBitmap
            !=
            null
        ) {

            Image(
                bitmap =
                cameraBitmap
                    .asImageBitmap(),

                contentDescription =
                "Obraz z kamery robota",

                modifier =
                Modifier
                    .fillMaxSize()
                    .graphicsLayer {
                        rotationZ = 180f
                        scaleX = -1f
                    },

                contentScale =
                ContentScale.Fit
            )

        } else {

            Text(
                text =
                if (
                    isConnected
                ) {

                    "Oczekiwanie na obraz z kamery..."

                } else {

                    "Brak połączenia z robotem"
                },

                fontSize =
                20.sp,

                fontWeight =
                FontWeight.Bold
            )
        }
    }
}


// =============================================================
// JAZDA PRZÓD / TYŁ
// =============================================================

@Composable
private fun DriveButtons(

    onCommandClick: (
        String
    ) -> Unit,

    modifier: Modifier =
        Modifier

) {

    Column(
        modifier =
        modifier,

        horizontalAlignment =
        Alignment.CenterHorizontally,

        verticalArrangement =
        Arrangement.spacedBy(
            10.dp
        )
    ) {

        Text(
            text =
            "Jazda",

            fontSize =
            18.sp,

            fontWeight =
            FontWeight.Bold
        )


        HoldButton(
            pointsUp =
            true,

            startCommand =
            "1",

            stopCommand =
            "0",

            onCommandClick =
            onCommandClick
        )


        HoldButton(
            pointsUp =
            false,

            startCommand =
            "2",

            stopCommand =
            "0",

            onCommandClick =
            onCommandClick
        )
    }
}


// =============================================================
// SKRĘT KÓŁ
// =============================================================

@Composable
private fun SteeringButtons(

    onCommandClick: (
        String
    ) -> Unit,

    modifier: Modifier =
        Modifier

) {

    Column(
        modifier =
        modifier,

        horizontalAlignment =
        Alignment.CenterHorizontally,

        verticalArrangement =
        Arrangement.spacedBy(
            8.dp
        )
    ) {

        Text(
            text =
            "Skręt kół",

            fontSize =
            17.sp,

            fontWeight =
            FontWeight.Bold
        )


        Row(
            horizontalArrangement =
            Arrangement.spacedBy(
                14.dp
            )
        ) {

            SteeringArrowButton(
                pointsLeft =
                true,

                command =
                "i",

                onCommandClick =
                onCommandClick
            )


            SteeringArrowButton(
                pointsLeft =
                false,

                command =
                "u",

                onCommandClick =
                onCommandClick
            )
        }
    }
}


// =============================================================
// KAMERA
// =============================================================

@Composable
private fun CameraTiltButtons(

    onCommandClick: (
        String
    ) -> Unit,

    modifier: Modifier =
        Modifier

) {

    Column(
        modifier =
        modifier,

        horizontalAlignment =
        Alignment.CenterHorizontally,

        verticalArrangement =
        Arrangement.spacedBy(
            8.dp
        )
    ) {

        Text(
            text =
            "Kamera",

            fontSize =
            17.sp,

            fontWeight =
            FontWeight.Bold
        )


        Row(
            horizontalArrangement =
            Arrangement.spacedBy(
                14.dp
            )
        ) {

            ArrowTapButton(
                pointsUp =
                false,

                command =
                "t",

                onCommandClick =
                onCommandClick
            )


            ArrowTapButton(
                pointsUp =
                true,

                command =
                "y",

                onCommandClick =
                onCommandClick
            )
        }
    }
}


// =============================================================
// PRZYCISK JAZDY
//
// Przytrzymanie:
// - wysyła komendę jazdy
// - po puszczeniu wysyła STOP
//
// Strzałka jest rysowana Canvasem.
// =============================================================

@Composable
private fun HoldButton(

    pointsUp: Boolean,

    startCommand: String,

    stopCommand: String,

    onCommandClick: (
        String
    ) -> Unit

) {

    Surface(
        modifier =
        Modifier
            .size(
                78.dp
            )
            .pointerInput(
                startCommand,
                stopCommand
            ) {

                detectTapGestures(

                    onPress = {

                        onCommandClick(
                            startCommand
                        )

                        try {

                            tryAwaitRelease()

                        } finally {

                            onCommandClick(
                                stopCommand
                            )
                        }
                    }
                )
            },

        shape =
        CircleShape,

        tonalElevation =
        6.dp
    ) {

        Box(
            modifier =
            Modifier.fillMaxSize(),

            contentAlignment =
            Alignment.Center
        ) {

            VerticalArrow(
                pointsUp =
                pointsUp,

                modifier =
                Modifier.size(
                    46.dp
                )
            )
        }
    }
}


// =============================================================
// STRZAŁKI SKRĘTU KÓŁ
//
// Grubość = 5.dp
// =============================================================

@Composable
private fun SteeringArrowButton(

    pointsLeft: Boolean,

    command: String,

    onCommandClick: (
        String
    ) -> Unit

) {

    Surface(
        modifier =
        Modifier
            .size(
                62.dp
            )
            .pointerInput(
                command
            ) {

                detectTapGestures(

                    onPress = {

                        coroutineScope {

                            val repeatJob =
                                launch {

                                    // Pierwsza komenda od razu.
                                    onCommandClick(
                                        command
                                    )

                                    // Potem ciąg komend podczas trzymania.
                                    while (true) {

                                        delay(
                                            90L
                                        )

                                        onCommandClick(
                                            command
                                        )
                                    }
                                }

                            try {

                                tryAwaitRelease()

                            } finally {

                                repeatJob.cancel()
                            }
                        }
                    }
                )
            },

        shape =
        CircleShape,

        tonalElevation =
        6.dp
    ) {

        Canvas(
            modifier =
            Modifier
                .fillMaxSize()
                .padding(
                    16.dp
                )
        ) {

            // =================================================
            // WSPÓLNA GRUBOŚĆ WSZYSTKICH STRZAŁEK
            // =================================================

            val stroke =
                5.dp.toPx()


            val centerY =
                size.height / 2f


            val startX =
                if (
                    pointsLeft
                ) {

                    size.width * 0.82f

                } else {

                    size.width * 0.18f
                }


            val endX =
                if (
                    pointsLeft
                ) {

                    size.width * 0.18f

                } else {

                    size.width * 0.82f
                }


            // Główna linia

            drawLine(
                color =
                Color.Black,

                start =
                androidx.compose.ui.geometry.Offset(
                    startX,
                    centerY
                ),

                end =
                androidx.compose.ui.geometry.Offset(
                    endX,
                    centerY
                ),

                strokeWidth =
                stroke,

                cap =
                StrokeCap.Round
            )


            val headX =
                if (
                    pointsLeft
                ) {

                    size.width * 0.18f

                } else {

                    size.width * 0.82f
                }


            val innerX =
                if (
                    pointsLeft
                ) {

                    size.width * 0.46f

                } else {

                    size.width * 0.54f
                }


            // Górna część grotu

            drawLine(
                color =
                Color.Black,

                start =
                androidx.compose.ui.geometry.Offset(
                    headX,
                    centerY
                ),

                end =
                androidx.compose.ui.geometry.Offset(
                    innerX,
                    size.height * 0.25f
                ),

                strokeWidth =
                stroke,

                cap =
                StrokeCap.Round
            )


            // Dolna część grotu

            drawLine(
                color =
                Color.Black,

                start =
                androidx.compose.ui.geometry.Offset(
                    headX,
                    centerY
                ),

                end =
                androidx.compose.ui.geometry.Offset(
                    innerX,
                    size.height * 0.75f
                ),

                strokeWidth =
                stroke,

                cap =
                StrokeCap.Round
            )
        }
    }
}


// =============================================================
// PRZYCISKI KAMERY
//
// Przytrzymanie dalej wysyła komendę co 90 ms.
// Strzałka jest teraz rysowana tak samo jak skręt.
// =============================================================

@Composable
private fun ArrowTapButton(

    pointsUp: Boolean,

    command: String,

    onCommandClick: (
        String
    ) -> Unit

) {

    Surface(
        modifier =
        Modifier
            .size(
                62.dp
            )
            .pointerInput(
                command
            ) {

                detectTapGestures(

                    onPress = {

                        coroutineScope {

                            val repeatJob =
                                launch {

                                    onCommandClick(
                                        command
                                    )

                                    while (true) {

                                        delay(
                                            90L
                                        )

                                        onCommandClick(
                                            command
                                        )
                                    }
                                }

                            try {

                                tryAwaitRelease()

                            } finally {

                                repeatJob.cancel()
                            }
                        }
                    }
                )
            },

        shape =
        CircleShape,

        tonalElevation =
        6.dp
    ) {

        Box(
            modifier =
            Modifier.fillMaxSize(),

            contentAlignment =
            Alignment.Center
        ) {

            VerticalArrow(
                pointsUp =
                pointsUp,

                modifier =
                Modifier.size(
                    38.dp
                )
            )
        }
    }
}


// =============================================================
// WSPÓLNA PIONOWA STRZAŁKA
//
// Używana przez:
// - jazdę ↑ ↓
// - kamerę ↑ ↓
//
// stroke = 5.dp, czyli dokładnie tyle samo,
// ile strzałki skrętu kół.
// =============================================================

@Composable
private fun VerticalArrow(

    pointsUp: Boolean,

    modifier: Modifier =
        Modifier

) {

    Canvas(
        modifier =
        modifier
    ) {

        val stroke =
            5.dp.toPx()


        val centerX =
            size.width / 2f


        val startY =
            if (
                pointsUp
            ) {

                size.height * 0.82f

            } else {

                size.height * 0.18f
            }


        val endY =
            if (
                pointsUp
            ) {

                size.height * 0.18f

            } else {

                size.height * 0.82f
            }


        // =====================================================
        // GŁÓWNA LINIA
        // =====================================================

        drawLine(
            color =
            Color.Black,

            start =
            androidx.compose.ui.geometry.Offset(
                centerX,
                startY
            ),

            end =
            androidx.compose.ui.geometry.Offset(
                centerX,
                endY
            ),

            strokeWidth =
            stroke,

            cap =
            StrokeCap.Round
        )


        val headY =
            if (
                pointsUp
            ) {

                size.height * 0.18f

            } else {

                size.height * 0.82f
            }


        val innerY =
            if (
                pointsUp
            ) {

                size.height * 0.46f

            } else {

                size.height * 0.54f
            }


        // =====================================================
        // LEWA CZĘŚĆ GROTU
        // =====================================================

        drawLine(
            color =
            Color.Black,

            start =
            androidx.compose.ui.geometry.Offset(
                centerX,
                headY
            ),

            end =
            androidx.compose.ui.geometry.Offset(
                size.width * 0.25f,
                innerY
            ),

            strokeWidth =
            stroke,

            cap =
            StrokeCap.Round
        )


        // =====================================================
        // PRAWA CZĘŚĆ GROTU
        // =====================================================

        drawLine(
            color =
            Color.Black,

            start =
            androidx.compose.ui.geometry.Offset(
                centerX,
                headY
            ),

            end =
            androidx.compose.ui.geometry.Offset(
                size.width * 0.75f,
                innerY
            ),

            strokeWidth =
            stroke,

            cap =
            StrokeCap.Round
        )
    }
}


// =============================================================
// STOP / WRÓĆ
// =============================================================

@Composable
private fun OverlayButton(

    text: String,

    onClick: () -> Unit

) {

    Surface(
        modifier =
        Modifier
            .width(
                112.dp
            )
            .height(
                48.dp
            )
            .alpha(
                0.88f
            )
            .pointerInput(
                text
            ) {

                detectTapGestures(

                    onTap = {

                        onClick()
                    }
                )
            },

        shape =
        RoundedCornerShape(
            18.dp
        ),

        tonalElevation =
        5.dp
    ) {

        Box(
            modifier =
            Modifier.fillMaxSize(),

            contentAlignment =
            Alignment.Center
        ) {

            Text(
                text =
                text,

                fontWeight =
                FontWeight.Bold
            )
        }
    }
}


// =============================================================
// FULLSCREEN
// =============================================================

@Composable
private fun FullscreenEffect() {

    val context =
        LocalContext.current


    val activity =
        context as? Activity


    DisposableEffect(
        activity
    ) {

        if (
            activity
            ==
            null
        ) {

            onDispose { }

        } else {

            val controller =
                WindowCompat
                    .getInsetsController(
                        activity.window,
                        activity.window.decorView
                    )


            controller.hide(
                WindowInsetsCompat
                    .Type
                    .systemBars()
            )


            onDispose {

                controller.show(
                    WindowInsetsCompat
                        .Type
                        .systemBars()
                )
            }
        }
    }
}