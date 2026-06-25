package com.avatar.dashboard

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.Box
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Build
import androidx.compose.material.icons.filled.Info
import androidx.compose.material.icons.filled.List
import androidx.compose.material3.Icon
import androidx.compose.material3.NavigationBar
import androidx.compose.material3.NavigationBarItem
import androidx.compose.material3.NavigationBarItemDefaults
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.*
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.viewmodel.compose.viewModel

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            AvatarTheme {
                val vm: DashboardViewModel = viewModel()
                val ui by vm.ui.collectAsStateWithLifecycle()
                DashboardRoot(ui)
            }
        }
    }
}

private enum class Tab(val label: String, val icon: ImageVector) {
    DIAG("Diagnostics", Icons.Filled.Info),
    JOINTS("Joints", Icons.Filled.List),
    ACTIVE("Motors", Icons.Filled.Build),
}

@Composable
private fun DashboardRoot(ui: UiState) {
    var tab by rememberSaveable { mutableStateOf(Tab.DIAG) }

    Scaffold(
        topBar = { StatusStrip(ui) },
        bottomBar = {
            NavigationBar(containerColor = SurfaceDark) {
                Tab.entries.forEach { t ->
                    NavigationBarItem(
                        selected = tab == t,
                        onClick = { tab = t },
                        icon = { Icon(t.icon, contentDescription = t.label) },
                        label = { Text(t.label, maxLines = 1) },
                        colors = NavigationBarItemDefaults.colors(
                            selectedIconColor = Accent,
                            selectedTextColor = Accent,
                            indicatorColor = SurfaceHi,
                            unselectedIconColor = TextDim,
                            unselectedTextColor = TextDim,
                        )
                    )
                }
            }
        }
    ) { inner ->
        Box(Modifier.fillMaxSize().padding(inner)) {
            when (tab) {
                Tab.DIAG   -> DiagnosticsScreen(ui)
                Tab.JOINTS -> JointsScreen(ui)
                Tab.ACTIVE -> ActiveMotorsScreen(ui)
            }
        }
    }
}
