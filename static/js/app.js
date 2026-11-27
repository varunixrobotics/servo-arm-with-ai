/**
 * =============================================================================
 * S.A.M. Robotic Mission Control - Frontend Orchestrator
 * =============================================================================
 */

document.addEventListener("DOMContentLoaded", () => {
    // 1. Initialize Three.js 3D Twin
    let arm3D = null;
    try {
        arm3D = new Arm3DVisualizer("twin-canvas-container");
    } catch (e) {
        console.error("Three.js Twin initialization error:", e);
    }

    // 2. DOM Elements
    const videoStream = document.getElementById("video-stream");
    const statFps = document.getElementById("stat-fps");
    const statSerial = document.getElementById("stat-serial");
    const statGemini = document.getElementById("stat-gemini");
    const hudStatusMsg = document.getElementById("hud-status-msg");
    const voiceLastHeard = document.getElementById("voice-last-heard");
    const terminalLogs = document.getElementById("terminal-logs");
    
    // Sliders & Values (6 Joints: Base, Shoulder, Elbow, Wrist Pitch, Wrist Roll, Gripper)
    const sliders = [
        document.getElementById("slider-j1"),
        document.getElementById("slider-j2"),
        document.getElementById("slider-j3"),
        document.getElementById("slider-j4"),
        document.getElementById("slider-j5"),
        document.getElementById("slider-j6"),
    ];
    const valDisplays = [
        document.getElementById("val-j1"),
        document.getElementById("val-j2"),
        document.getElementById("val-j3"),
        document.getElementById("val-j4"),
        document.getElementById("val-j5"),
        document.getElementById("val-j6"),
    ];

    // Target Cards
    const cubeCards = {
        RED: document.getElementById("card-cube-red"),
        GREEN: document.getElementById("card-cube-green"),
        BLUE: document.getElementById("card-cube-blue"),
        BLACK: document.getElementById("card-cube-black")
    };

    // Sensor Cards
    const gripperMeta = document.getElementById("gripper-meta");
    const handMeta = document.getElementById("hand-meta");
    const hudErrorText = document.getElementById("hud-error-text");

    let isUserDraggingSlider = false;
    let activeCube = "BLUE";

    // -------------------------------------------------------------
    // Telemetry Polling Loop
    // -------------------------------------------------------------
    async function updateTelemetry() {
        try {
            const res = await fetch("/api/status");
            if (!res.ok) return;
            const data = await res.json();

            // Update FPS & Badges
            if (statFps) statFps.textContent = `${data.fps} FPS`;
            if (statSerial) {
                const s = data.serial;
                statSerial.textContent = s.connected ? (s.mock ? "SIMULATION" : s.port) : "OFFLINE";
                statSerial.parentElement.classList.toggle("offline", !s.connected);
            }
            if (statGemini) {
                statGemini.textContent = data.gemini.configured ? "ONLINE" : "OFFLINE";
                statGemini.parentElement.classList.toggle("offline", !data.gemini.configured);
            }

            // HUD Message
            if (hudStatusMsg) hudStatusMsg.textContent = data.hud_message || "System Active";

            // Voice status
            if (voiceLastHeard) {
                voiceLastHeard.textContent = data.voice.last_heard ? `"${data.voice.last_heard}"` : "Listening for 'give it to me'...";
            }

            // Error display
            if (hudErrorText) {
                const err = data.error;
                hudErrorText.textContent = `Offset: (${err.ex}, ${err.ey}) | Dist: ${err.dist}px ${err.aligned ? '[ALIGNED]' : ''}`;
            }

            // Update Cubes
            activeCube = data.active_cube;
            for (const [color, card] of Object.entries(cubeCards)) {
                if (!card) continue;
                const info = data.cubes[color];
                const statusEl = card.querySelector(".target-status");
                const coordsEl = card.querySelector(".target-coords");

                card.classList.toggle("active-target", color === activeCube);

                if (info && info.detected) {
                    statusEl.textContent = `TRACKED | Dist: ${info.dist !== null ? info.dist + 'px' : '--'}`;
                    statusEl.style.color = "var(--accent-green)";
                    coordsEl.textContent = `X: ${info.x}  Y: ${info.y}`;
                } else {
                    statusEl.textContent = "SEARCHING...";
                    statusEl.style.color = "var(--text-dim)";
                    coordsEl.textContent = "X: --  Y: --";
                }
            }

            // Update Gripper Info
            if (gripperMeta) {
                if (data.gripper) {
                    gripperMeta.textContent = `(${data.gripper.center[0]}, ${data.gripper.center[1]}) | ${data.gripper.angle}° | ${data.gripper.dots_count}/4 dots`;
                    gripperMeta.style.color = "var(--accent-cyan)";
                } else {
                    gripperMeta.textContent = "Dots not visible in frame";
                    gripperMeta.style.color = "var(--text-dim)";
                }
            }

            // Update Hand Info
            if (handMeta) {
                if (data.hand) {
                    handMeta.textContent = `IN WORKSPACE (${data.hand.center[0]}, ${data.hand.center[1]})`;
                    handMeta.style.color = "var(--accent-green)";
                } else {
                    handMeta.textContent = "CLEAR (No hand in view)";
                    handMeta.style.color = "var(--text-dim)";
                }
            }

            // Update 3D Model & Sliders (if not actively being dragged by user)
            if (arm3D && data.angles) {
                arm3D.setAngles(data.angles);
            }
            if (!isUserDraggingSlider && data.angles) {
                data.angles.forEach((deg, idx) => {
                    if (sliders[idx]) sliders[idx].value = deg;
                    if (valDisplays[idx]) valDisplays[idx].textContent = `${deg}°`;
                });
            }

            // Update Logs
            if (terminalLogs && data.logs) {
                terminalLogs.innerHTML = "";
                data.logs.forEach(l => {
                    const row = document.createElement("div");
                    row.className = "log-entry";
                    row.innerHTML = `<span class="log-time">[${l.time}]</span> <span class="log-src ${l.source}">&lt;${l.source}&gt;</span> <span class="log-msg">${l.msg}</span>`;
                    terminalLogs.appendChild(row);
                });
                terminalLogs.scrollTop = terminalLogs.scrollHeight;
            }

        } catch (e) {
            console.debug("Telemetry fetch error:", e);
        }
    }

    // Start 100ms telemetry interval
    setInterval(updateTelemetry, 100);

    // -------------------------------------------------------------
    // Slider Handlers (Manual Jogging)
    // -------------------------------------------------------------
    function onSliderChange() {
        const currentVals = sliders.map(s => parseInt(s.value, 10));
        currentVals.forEach((v, idx) => {
            if (valDisplays[idx]) valDisplays[idx].textContent = `${v}°`;
        });
        if (arm3D) arm3D.setAngles(currentVals);

        // Send to backend
        fetch("/api/control/angles", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ angles: currentVals })
        });
    }

    sliders.forEach(slider => {
        if (!slider) return;
        slider.addEventListener("mousedown", () => { isUserDraggingSlider = true; });
        slider.addEventListener("touchstart", () => { isUserDraggingSlider = true; });
        slider.addEventListener("input", onSliderChange);
        slider.addEventListener("mouseup", () => {
            isUserDraggingSlider = false;
            onSliderChange();
        });
        slider.addEventListener("touchend", () => {
            isUserDraggingSlider = false;
            onSliderChange();
        });
    });

    // -------------------------------------------------------------
    // Preset Buttons
    // -------------------------------------------------------------
    window.applyPreset = (pose) => {
        let angles = [90, 90, 90, 90, 90, 100];
        if (pose === 'home') {
            fetch("/api/control/home", { method: "POST" });
            return;
        } else if (pose === 'pick') {
            angles = [90, 50, 130, 90, 90, 100];
        } else if (pose === 'rest') {
            angles = [90, 140, 45, 90, 90, 100];
        } else if (pose === 'grip_toggle') {
            const gripIdx = sliders[5] ? 5 : 4;
            const curGrip = sliders[gripIdx] ? parseInt(sliders[gripIdx].value, 10) : 100;
            const open = curGrip < 60;
            fetch("/api/control/gripper", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ open })
            });
            return;
        }
        fetch("/api/control/angles", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ angles })
        });
    };

    // -------------------------------------------------------------
    // Cube Target Selection
    // -------------------------------------------------------------
    window.selectCube = (color) => {
        fetch("/api/control/target_cube", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ color })
        });
    };

    // Master Delivery Trigger
    window.triggerDelivery = () => {
        fetch("/api/control/trigger_delivery", { method: "POST" });
    };

    // Emergency Stop
    window.emergencyStop = () => {
        fetch("/api/control/home", { method: "POST" });
    };

    // -------------------------------------------------------------
    // View Mode Tabs
    // -------------------------------------------------------------
    document.querySelectorAll(".view-tab").forEach(tab => {
        tab.addEventListener("click", () => {
            document.querySelectorAll(".view-tab").forEach(t => t.classList.remove("active"));
            tab.classList.add("active");
            const mode = tab.dataset.mode || "ai";
            fetch("/api/control/view_mode", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ mode })
            });
            if (videoStream) {
                videoStream.src = `/video_feed?mode=${mode}&t=${Date.now()}`;
            }
        });
    });

    // -------------------------------------------------------------
    // Settings & API Key Modal
    // -------------------------------------------------------------
    const settingsModal = document.getElementById("settings-modal");
    window.openSettings = async () => {
        if (!settingsModal) return;
        settingsModal.classList.add("open");

        // Load available ports
        try {
            const res = await fetch("/api/ports");
            const data = await res.json();
            const portSelect = document.getElementById("setting-port");
            if (portSelect && data.ports) {
                portSelect.innerHTML = '<option value="AUTO">AUTO Detect</option>';
                data.ports.forEach(p => {
                    const opt = document.createElement("option");
                    opt.value = p.device;
                    opt.textContent = `${p.device} (${p.description})`;
                    portSelect.appendChild(opt);
                });
            }
        } catch (e) {
            console.error(e);
        }
    };

    window.closeSettings = () => {
        if (settingsModal) settingsModal.classList.remove("open");
    };

    window.saveSettings = async () => {
        const apiKey = document.getElementById("setting-gemini-key").value.trim();
        const port = document.getElementById("setting-port").value;
        const mock = document.getElementById("setting-mock").checked;

        if (apiKey) {
            await fetch("/api/settings/gemini", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ api_key: apiKey })
            });
        }

        await fetch("/api/settings/serial", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ port, baud: 115200, mock })
        });

        closeSettings();
    };
});
