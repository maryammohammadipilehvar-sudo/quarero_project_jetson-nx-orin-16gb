        // Globale Variablen
        let map, robotMarker, homeMarker, chargeMarker;
        
        // Route visualization (main map) - completely separate from editor
        const routeVisualization = {
            path: null,
            markers: []
        };
        
        // Editor visualization - completely separate from main route
        const editorVisualization = {
            path: null,
            markers: [],
            waypoints: [],
            mode: null  // 'new' or 'edit'
        };
        
        let currentPosition = { latitude: 0, longitude: 0, yaw: 0 };
        let homePosition = null;
        let mapCentered = false;
        let currentRoute = null;
        let activeRouteName = null; // Track active route name from robot state (legacy string format)
        let activeRouteMode = null; // Track active route mode from robot state (0=ONCE, 1=LOOP, 2=PING_PONG, 4=STOP)
        let activeRouteData = null; // Track full active route object from robot state (with waypoints and mode)
        let lightStatus = false;
        let lightPending = false;
        let autonomousOperationEnabled = false;
        let currentCameraStream = 'main';
        let wsCamera = null;
        let isSwitchingCamera = false; // Flag to prevent auto-reconnect during camera switch
        let lastErrorTime = {};
        let lastCameraFrameTime = 0;
        let lastDisplayedFrameTimestamp = 0; // Track last displayed frame timestamp to skip old frames
        // Fixposition Status (Rohwerte vom Backend)
        let fusionRawState = {
            fusion_status: null,
            imu_status: null,
            gnss_status: null,
            rtk_status: null
        };

        // Roboter-Pfeil-Icon erstellen
        const robotIcon = L.divIcon({
            className: 'robot-marker',
            html: `<div style="width: 30px; height: 30px; position: relative;">
                <svg viewBox="0 0 30 30" style="position: absolute; top: -15px; left: -15px;">
                    <circle cx="15" cy="15" r="10" fill="#f44336" stroke="#fff" stroke-width="2"/>
                    <path d="M15,10 L20,20 L15,18 L10,20 Z" fill="#fff"/>
                </svg>
            </div>`,
            iconSize: [30, 30],
            iconAnchor: [15, 15]
        });

        // Home-Icon erstellen - MIT KORREKTER POSITIONIERUNG
        const homeIcon = L.divIcon({
            className: 'home-marker',
            html: `<div style="font-size: 32px; text-align: center; line-height: 1;">🏠</div>`,
            iconSize: [8, 8],      // 50 % der alten Größe
            iconAnchor: [4, 4]       // zentriert zur neuen Größe
        });

        // Leaflet Karte initialisieren
        async function initMap() {
            //Änderungen Leon:

            // STANDARD-WERTE FÜR STARTPOSITION
            let startLat = 46.971737848071115;
            let startLng = 8.460399166448894;
            let startZoom = 18;
            
            // Home Position zuerst laden (kann einige Sekunden dauern)
            try {
                const response = await fetch('/api/settings/home-position');
                const data = await response.json();
                
                console.log('Loaded home position data for map init:', data);
                
                // Priorität: home_point > charge_point > Standard-Koordinaten
                if (data.home_point && data.home_point.latitude) {
                    console.log("Starte Karte bei Home Position...");
                    startLat = data.home_point.latitude;
                    startLng = data.home_point.longitude;
                    homePosition = data.home_point;
                } else if (data.charge_point && data.charge_point.latitude) {
                    console.log("Starte Karte bei bekannter Ladestation...");
                    startLat = data.charge_point.latitude;
                    startLng = data.charge_point.longitude;
                }
            } catch (e) {
                console.warn('Failed to load home position for map init, using defaults:', e);
            }

            map = L.map('map', {
                maxZoom: 22
            }).setView([startLat, startLng], startZoom);
            
            // Änderung Leon: 
            // Definiere die "Standard"-Karte (OpenStreetMap)
            var osmLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '© OpenStreetMap contributors',
                maxNativeZoom: 19,
                maxZoom: 22
            });

            // Definiere die Satelliten-Karte (Esri)
            var satelliteLayer = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
                attribution: 'Tiles &copy; Esri &mdash; Source: Esri...',
                maxNativeZoom: 19,
                maxZoom: 22
            });

            // Füge die Karte hinzu, die beim Start sichtbar sein soll (hier: Satellit)
            satelliteLayer.addTo(map);

            // Erstelle das Umschalt-Menü
            var baseMaps = {
                "Satellit": satelliteLayer,
                "Karte (Standard)": osmLayer
            };

            // Füge das Menü oben rechts zur Karte hinzu
            L.control.layers(baseMaps).addTo(map);

            map.on('click', (e) => {
                if (editorVisualization.mode) {
                    addEditorWaypoint(e.latlng.lat, e.latlng.lng);
                }
            });

            window.addEventListener('resize', () => {
                setTimeout(() => map.invalidateSize(), 100);
            });
            
            // Home Position vollständig laden (für Marker und weitere Updates)
            loadHomePosition();
        }
        function goToCharge() {
            // Show confirmation modal
            const modal = document.getElementById('confirm-modal');
            modal.classList.add('show');
        }

        function closeConfirmModal() {
            const modal = document.getElementById('confirm-modal');
            modal.classList.remove('show');
        }

        async function confirmGoToCharge() {
            // Close modal first
            closeConfirmModal();
            
            try {
                const response = await fetch('/api/control/charge/go_to_charge', { 
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' }
                });
                
                if (!response.ok) {
                    throw new Error(`HTTP error! status: ${response.status}`);
                }
                
                const data = await response.json();
                
                if (data.status === 'success') {
                    // Backend will send log message via WebSocket
                } else {
                    // Server returned error response - backend already logged the error
                    alert('Fehler: ' + data.message);
                }
            } catch (e) {
                // Communication/network error
                addLogEntry({
                    timestamp: new Date().toLocaleTimeString(),
                    message: `❌ Kommunikationsfehler: ${e.message}`,
                    isError: true
                });
                alert('Fehler: ' + e.message);
            }
        }

        // Home Position laden und anzeigen
       // Beide Marker anzeigen
        async function loadHomePosition() {
            try {
                const response = await fetch('/api/settings/home-position');
                const data = await response.json();
                
                console.log('Loaded home position data:', data); // DEBUG
                
            // Charge Point (Ladestation) – mit Haus-Icon anzeigen
            if (data.charge_point && data.charge_point.latitude) {
                if (chargeMarker && map) map.removeLayer(chargeMarker);

                // Charge Point nutzt das verkleinerte Haus-Symbol
                if (map) {
                    chargeMarker = L.marker([data.charge_point.latitude, data.charge_point.longitude], {
                        icon: homeIcon,
                        title: 'Ladestation'
                    }).addTo(map);
                    chargeMarker.bindPopup('<b>🏠 Ladestation</b><br>Charge Point');
                }
            }

            // Home Point nur für Logik (Route Editor) verwenden, aber NICHT anzeigen
            if (data.home_point && data.home_point.latitude) {
                // KEIN Marker mehr auf der Karte, nur die Koordinaten speichern
                homePosition = data.home_point;
                console.log('Set homePosition to:', homePosition);
            } else {
                console.warn('No home_point in response!');
                homePosition = null;
            }

            } catch (e) {
                console.error('Failed to load positions:', e);
                homePosition = null;
            }
        }
        
        // WebSocket für Position
        const wsPosition = new WebSocket(`ws://${window.location.host}/ws/position`);
        wsPosition.onmessage = (event) => {
            const data = JSON.parse(event.data);
            if (data.type === 'position') {
                updatePosition(data.data);
            } else if (data.type === 'event') {
                addLogEntry(data.data);
            } else if (data.type === 'light_status') {
                updateLightButton(data.data);
            } else if (data.type === 'charging_status') {
                updateChargingButton(data.data);
            } else if (data.type === 'autonomous_status') {
                autonomousOperationEnabled = data.data;
                updateStopButton();
            } else if (data.type === 'waypoint_saved') {
                onWaypointSaved(data.data);
            } else if (data.type === 'fusion_status') {
                fusionRawState = data.data || fusionRawState;
                updateFusionBadges(fusionRawState);
            } else if (data.type === 'logs_cleared') {
                // Backend cleared logs, reload them
                loadLogs();
            } else if (data.type === 'select_button_pressed') {
                // Handle select button press from controller
                handleSelectButtonPress(data.data);
            }
        };

        // WebSocket für Kamera
        connectCameraWebSocket();

        // Track intervals for cleanup
        const intervals = [];
        
        const cameraCheckInterval = setInterval(() => {
            const container = document.getElementById('camera-container');
            const img = document.getElementById('camera-feed');
            if (!lastCameraFrameTime || Date.now() - lastCameraFrameTime > 3000) {
                container.classList.remove('has-frame');
                img.removeAttribute('src');
            }
        }, 1000);
        intervals.push(cameraCheckInterval);
        
        // WebSocket für Robot State
        const wsRobotState = new WebSocket(`ws://${window.location.host}/ws/robot_state`);
        let lastRobotStateTime = 0; // Initialize to 0 so timeout check immediately detects no message
        const ROBOT_TIMEOUT = 5000; // 5 seconds timeout
        let robotStateReceived = false; // Track if we've ever received a valid robot state
        let lastBackendConnectionStatus = null; // Track backend's connection status
        
        wsRobotState.onopen = () => {
            console.log('Robot state WebSocket connected');
        };
        
        wsRobotState.onerror = (error) => {
            console.error('Robot state WebSocket error:', error);
            updateRobotConnectionStatus(false);
        };
        
        wsRobotState.onclose = () => {
            console.log('Robot state WebSocket closed');
            updateRobotConnectionStatus(false);
        };
        
        wsRobotState.onmessage = (event) => {
            const data = JSON.parse(event.data);
            if (data.type === 'robot_state') {
                const stateData = data.data;
                // Use backend's robot_connected status if available (this is the source of truth)
                if (stateData.robot_connected !== undefined) {
                    // Backend knows if robot controller node is actually publishing
                    lastBackendConnectionStatus = stateData.robot_connected;
                    updateRobotConnectionStatus(stateData.robot_connected);
                    if (stateData.robot_connected) {
                        robotStateReceived = true;
                        lastRobotStateTime = Date.now();
                    } else {
                        // If backend says disconnected, reset our tracking
                        robotStateReceived = false;
                        lastRobotStateTime = 0;
                    }
                } else {
                    // Fallback: check if we have valid robot data
                    if (stateData.battery !== undefined || stateData.velocity !== undefined || stateData.error_status !== undefined) {
                        robotStateReceived = true;
                        lastRobotStateTime = Date.now();
                        updateRobotConnectionStatus(true);
                    }
                }
                
                // Handle active_route from robot state
                if (stateData.active_route !== undefined) {
                    console.log('[DEBUG] active_route received:', JSON.stringify(stateData.active_route, null, 2));
                    const previousActiveRoute = activeRouteName;
                    const previousActiveMode = activeRouteMode;
                    const previousActiveRouteData = activeRouteData;
                    
                    // Handle active_route as object (with name, waypoints and mode) or string (route name)
                    if (stateData.active_route && typeof stateData.active_route === 'object') {
                        // New format: object with name, waypoints and mode
                        activeRouteData = stateData.active_route;
                        activeRouteName = stateData.active_route.name || null;
                        
                        console.log('[DEBUG] Active route parsed:', {
                            name: activeRouteName,
                            mode: stateData.active_route.mode,
                            waypointsCount: stateData.active_route.waypoints?.length || 0,
                            waypoints: stateData.active_route.waypoints
                        });
                        
                        // Convert mode string to integer for visualization
                        // Mode strings: 'none', 'once', 'loop', 'ping_pong'
                        // Mode integers: 0=ONCE, 1=LOOP, 2=PING_PONG, 4=STOP
                        const modeString = stateData.active_route.mode;
                        if (modeString === 'none') {
                            activeRouteMode = 4; // STOP
                        } else if (modeString === 'once') {
                            activeRouteMode = 0; // ONCE
                        } else if (modeString === 'loop') {
                            activeRouteMode = 1; // LOOP
                        } else if (modeString === 'ping_pong') {
                            activeRouteMode = 2; // PING_PONG
                        } else {
                            activeRouteMode = null;
                        }
                        
                        // Update route info display
                        updateRouteInfoDisplay(stateData.active_route);
                    } else {
                        // Legacy: active_route is a string (route name)
                        activeRouteName = stateData.active_route || null;
                        activeRouteMode = null; // Will use route's loop_mode
                        activeRouteData = null;
                        // Hide route info for legacy format
                        updateRouteInfoDisplay(null);
                    }
                    
                    // If no route is selected and active route changed, reload it
                    // BUT: Skip if we're in route editor mode to prevent clearing editor waypoints
                    const routeSelect = document.getElementById('route-select');
                    if (!routeSelect.value && !editorVisualization.mode) {
                        const routeChanged = activeRouteName !== previousActiveRoute || 
                                           activeRouteMode !== previousActiveMode ||
                                           JSON.stringify(activeRouteData) !== JSON.stringify(previousActiveRouteData);
                        if (routeChanged) {
                            loadSelectedRoute();
                        }
                    }
                } else {
                    // No active route
                    console.log('[DEBUG] No active_route in stateData (undefined)');
                    activeRouteName = null;
                    activeRouteData = null;
                    activeRouteMode = null;
                    updateRouteInfoDisplay(null);
                }
                
                // Always update route visualization if no manual route is selected
                // This ensures active routes are shown even if route data didn't change
                // BUT: Skip if we're in route editor mode to prevent clearing editor waypoints
                const routeSelect = document.getElementById('route-select');
                console.log('[DEBUG] Route visualization check:', {
                    routeSelectValue: routeSelect?.value,
                    hasActiveRouteData: !!activeRouteData,
                    hasActiveRouteName: !!activeRouteName,
                    editorMode: editorVisualization.mode,
                    currentRoute: currentRoute?.name,
                    shouldUpdate: !routeSelect?.value && (activeRouteData || activeRouteName) && !editorVisualization.mode
                });
                
                if (!routeSelect?.value && (activeRouteData || activeRouteName) && !editorVisualization.mode) {
                    // Check if we need to update the visualization
                    // (either no currentRoute shown, or different from active route)
                    const needsUpdate = !currentRoute || 
                        currentRoute.name !== activeRouteName ||
                        JSON.stringify(currentRoute.waypoints) !== JSON.stringify(activeRouteData?.waypoints);
                    
                    console.log('[DEBUG] Route visualization update check:', {
                        needsUpdate: needsUpdate,
                        hasCurrentRoute: !!currentRoute,
                        currentRouteName: currentRoute?.name,
                        activeRouteName: activeRouteName,
                        waypointsMatch: currentRoute ? JSON.stringify(currentRoute.waypoints) === JSON.stringify(activeRouteData?.waypoints) : false
                    });
                    
                    if (needsUpdate) {
                        console.log('[DEBUG] Calling loadSelectedRoute() for active route');
                        loadSelectedRoute();
                    }
                }
                
                updateRobotStatus(stateData);
            }
        };

        // Check robot connection status periodically (only as fallback if backend status not available)
        const robotStateCheckInterval = setInterval(() => {
            // Check WebSocket connection status
            if (wsRobotState.readyState !== WebSocket.OPEN) {
                updateRobotConnectionStatus(false);
                return;
            }
            
            // If backend provides connection status, trust it completely
            // Only use timeout check if backend status is not available
            if (lastBackendConnectionStatus === null) {
                const timeSinceLastState = Date.now() - lastRobotStateTime;
                if (!robotStateReceived || lastRobotStateTime === 0 || timeSinceLastState > ROBOT_TIMEOUT) {
                    updateRobotConnectionStatus(false);
                }
            }
            // Otherwise, backend status is already being used in onmessage handler
        }, 1000);
        intervals.push(robotStateCheckInterval);

        // Initialize connection status on page load - delay showing disconnected to prevent flickering
        // Wait for first WebSocket message before showing disconnected status
        setTimeout(() => {
            // Only show disconnected if we haven't received any connection status from backend
            // and haven't received any robot state messages
            if (lastBackendConnectionStatus === null && !robotStateReceived) {
                updateRobotConnectionStatus(false);
            }
        }, 1500); // Wait 1.5 seconds before showing disconnected to allow WebSocket to connect

        function updateRobotConnectionStatus(connected) {
            const statusEl = document.getElementById('robot-connection-status');
            if (!statusEl) return;
            
            if (connected) {
                statusEl.classList.remove('disconnected');
                statusEl.classList.add('connected');
                statusEl.style.display = 'none'; // Hide when connected
            } else {
                statusEl.classList.remove('connected');
                statusEl.classList.add('disconnected');
                statusEl.style.display = 'flex'; // Show when disconnected
            }
        }

        // UI sprint #1: drive the big "what is the robot doing right now?" card.
        // See PLAN_UI_CUSTOMER_HANDOFF.md §7 item #1.
        function updateMainStatusPanel(state) {
            const panel = document.getElementById('main-status-panel');
            if (!panel) return;
            const iconEl = document.getElementById('main-status-icon');
            const headlineEl = document.getElementById('main-status-headline');
            const detailEl = document.getElementById('main-status-detail');
            const battery = Number(state.battery || state.battery_percentage || 0);
            const charging = !!(state.charging_state);
            const autonomousOn = !!(state.autonomous_enabled);
            const route = state.active_route && typeof state.active_route === 'object'
                ? state.active_route : null;
            const routeName = (route && route.name) ? route.name : '';
            const wpIdx = (route && route.current_waypoint_index != null) ? route.current_waypoint_index : null;
            const wpTotal = (route && Array.isArray(route.waypoints)) ? route.waypoints.length : null;

            // Compute headline + icon based on what the robot is actually doing,
            // in order of operational interest (charging > driving > waiting).
            let icon = '⏳', headline = 'Roboter startet — bitte warten', detail = '';
            let color = 'gray';

            if (charging) {
                icon = '🔌';
                headline = `Lädt — ${Math.round(battery)} %`;
                if (battery < 80) {
                    detail = `Missionen ab 80 % möglich. Aktuell: ${Math.round(battery)} %`;
                } else {
                    detail = 'Roboter steht an der Ladestation.';
                }
                color = 'green';
            } else if (battery < 20 && autonomousOn && !routeName) {
                icon = '🪫';
                headline = `Akku zu niedrig — ${Math.round(battery)} %`;
                detail = 'Keine Missionen möglich. Roboter muss erst auf 80 % laden.';
                color = 'red';
            } else if (routeName) {
                icon = '🚗';
                headline = `Fährt Route „${routeName}”`;
                if (wpIdx != null && wpTotal) {
                    detail = `Wegpunkt ${wpIdx + 1} von ${wpTotal}`;
                } else {
                    detail = 'Autonome Fahrt läuft.';
                }
                color = 'blue';
            } else if (autonomousOn) {
                icon = '🛡';
                headline = 'Bereit für autonomen Betrieb';
                detail = 'Wartet auf Route oder Zeitplan.';
                color = 'green';
            } else {
                icon = '🕹';
                headline = 'Manueller Modus';
                detail = 'Autonomie ist aus. Roboter wartet auf Steuerung.';
                color = 'gray';
            }

            iconEl.textContent = icon;
            headlineEl.textContent = headline;
            detailEl.textContent = detail;
            panel.classList.remove('main-status-gray', 'main-status-green', 'main-status-blue', 'main-status-red');
            panel.classList.add(`main-status-${color}`);

            // Battery bar — color stays neutral but red when very low
            const bLabel = document.getElementById('main-battery-label');
            const bFill = document.getElementById('main-battery-fill');
            if (bLabel && bFill) {
                bLabel.textContent = (battery > 0)
                    ? `Akku ${Math.round(battery)} %`
                    : 'Akku —';
                const pct = Math.max(0, Math.min(100, battery));
                bFill.style.width = pct + '%';
                bFill.classList.remove('main-battery-low', 'main-battery-mid', 'main-battery-high');
                if (pct < 20) bFill.classList.add('main-battery-low');
                else if (pct < 50) bFill.classList.add('main-battery-mid');
                else bFill.classList.add('main-battery-high');
            }

            const lockBanner = document.getElementById('dashboard-battery-lock');
            if (lockBanner) {
                lockBanner.style.display = (battery < 20 && !charging) ? 'flex' : 'none';
            }
        }

        function updateRobotStatus(state) {
            // UI sprint #1: feed the simplified top-of-page status card first.
            try { updateMainStatusPanel(state); } catch (e) { console.warn('main status panel update:', e); }

            const battery = state.battery || 0;
            const batteryEl = document.getElementById('battery-level');
            batteryEl.textContent = `${battery}%`;
            
            const batteryCard = batteryEl.closest('.status-card');
            if (battery < 20) {
                batteryCard.style.borderLeftColor = '#f44336';
            } else if (battery < 50) {
                batteryCard.style.borderLeftColor = '#FF9800';
            } else {
                batteryCard.style.borderLeftColor = '#4CAF50';
            }
            
            // NEU: Geschwindigkeit in km/h anzeigen
        const velocity_kmh = state.velocity_kmh || 0;
        document.getElementById('velocity-value').textContent = `${velocity_kmh.toFixed(1)} km/h`;
            
            // Verbl. Einsatzzeit anzeigen (in Minuten, ohne Nachkommastellen)
            const runtimeEl = document.getElementById('runtime-value');
            const remainingRuntime = state.remaining_runtime_minutes;
            if (remainingRuntime !== null && remainingRuntime !== undefined) {
                runtimeEl.textContent = `${remainingRuntime} min`;
            } else {
                runtimeEl.textContent = '--';
            }
            if (state.error_status && state.error_status.trim() !== '') {
                handleError(state.error_status);
            }
            
            // Sync button states from robot (single source of truth)
            if (state.light_state !== undefined) {
                updateLightButton(state.light_state);
            }
            if (state.charging_state !== undefined) {
                updateChargingButton(state.charging_state);
            }
            if (state.autonomous_enabled !== undefined) {
                autonomousOperationEnabled = state.autonomous_enabled;
                updateStopButton();
            }
        }

    function updateFusionBadges(state) {

        // Änderung Leon:
        if (!state) {
            
            const offlineColor = 'red'; 
            
            setBadgeColor('badge-imu', offlineColor);
            setBadgeColor('badge-gnss1', offlineColor); 
            setBadgeColor('badge-gnss2', offlineColor);  
            setBadgeColor('badge-fusion', offlineColor);
            return; // Wir brechen hier ab, da es keine Zahlen zum Prüfen gibt
    }

        const imu = Number(state.imu_status);
        const gnss1 = Number(state.gnss1_status);  // Umbenannt
        const gnss2 = Number(state.gnss2_status);  // NEU
        const fusion = Number(state.fusion_status);

        // IMU-Farbe bestimmen: Grün ab warmstarted (1), rot nur bei echten Fehlern (0, null, undefined)
        let imuColor = 'red';
        if (imu >= 1) {           // IMU_STATUS_WARMSTARTED (1), CONVERGING (2), CONVERGED (3)
            imuColor = 'green';
        }

        // GNSS1-Farbe bestimmen
        let gnss1Color = 'red';
        if (gnss1 === 5) {          // GNSS_STATUS_RTK_FLOAT
            gnss1Color = 'orange';
        } else if (gnss1 === 8) {   // GNSS_STATUS_RTK_FIXED
            gnss1Color = 'green';
        } else if (gnss1 === 1) {   // GNSS_STATUS_SPP
            gnss1Color = 'orange';
        }

        // GNSS2-Farbe bestimmen
        let gnss2Color = 'red';
        if (gnss2 === 5) {          // GNSS_STATUS_RTK_FLOAT
            gnss2Color = 'orange';
        } else if (gnss2 === 8) {   // GNSS_STATUS_RTK_FIXED
            gnss2Color = 'green';
        } else if (gnss2 === 1) {   // GNSS_STATUS_SPP
            gnss2Color = 'orange';
        }

        // Fusion-Farbe bestimmen
        let fusionColor = 'red';
        if (fusion === 2 || fusion === 3 || fusion === 4) { // IMU_GNSS / VIO_GNSS
            fusionColor = 'green';
        }

        setBadgeColor('badge-imu', imuColor);
        setBadgeColor('badge-gnss1', gnss1Color); 
        setBadgeColor('badge-gnss2', gnss2Color);  
        setBadgeColor('badge-fusion', fusionColor);
    }

        /**
         * Setzt die CSS-Farbe eines Badges anhand der Ampel-Farbe.
         */
        function setBadgeColor(id, color) {
            const el = document.getElementById(id);
            if (!el) return;

            el.classList.remove('red', 'orange', 'green');

            // alle undefinierten / unbekannten Farben => rot
            if (color !== 'orange' && color !== 'green') {
                color = 'red';
            }
            el.classList.add(color);
        }
        function handleError(errorMsg) {
            const now = Date.now();
            const lastShown = lastErrorTime[errorMsg] || 0;
            
            if (now - lastShown > 60000) {
                lastErrorTime[errorMsg] = now;
                addLogEntry({
                    timestamp: new Date().toLocaleTimeString(),
                    message: `⚠️ FEHLER: ${errorMsg}`,
                    isError: true
                });
            }
        }

        // Position aktualisieren - MIT INVERTIERTER ROBOTER-RICHTUNG (FIX 1)
        function updatePosition(pos) {
            const lat = pos.latitude;
            const lon = pos.longitude;
            const yaw = pos.yaw || 0;
            
            if (lat === 0 && lon === 0) return;
            
            currentPosition = pos;

            if (robotMarker) {
                robotMarker.setLatLng([lat, lon]);
                // Rotation des Pfeils - NEGIERT für korrekte Ausrichtung
                const markerEl = robotMarker.getElement();
                if (markerEl) {
                    const svg = markerEl.querySelector('svg');
                    if (svg) {
                        const headingDeg = 90 - yaw;
                        svg.style.transform = `rotate(${headingDeg}deg)`;
                    }
                }
            } else {
                robotMarker = L.marker([lat, lon], {
                    icon: robotIcon,
                    rotationAngle: 90 - yaw  
                }).addTo(map);
            }
            
            if (!mapCentered) {
                map.setView([lat, lon], 18);
                mapCentered = true;
            }
        }

        // Routen laden
        async function loadRoutes() {
            const response = await fetch('/api/routes');
            const data = await response.json();
            
            const select = document.getElementById('route-select');
            select.innerHTML = '<option value="">-- Aktive Route --</option>';
            
            data.routes.forEach(route => {
                const option = document.createElement('option');
                option.value = route;
                option.textContent = route;
                select.appendChild(option);
            });
        }

        // Route laden - FIX: Alte Marker entfernen
        async function loadSelectedRoute() {
            const routeName = document.getElementById('route-select').value;
            
            console.log('[DEBUG] loadSelectedRoute called:', { routeName, activeRouteName, hasActiveRouteData: !!activeRouteData });
            
            // Clear route visualization (but NOT editor visualization)
            clearRouteVisualization();
            
            if (!routeName) {
                // No route selected - show active route if available
                // Always use waypoints directly from robot state (they may differ from stored route)
                console.log('[DEBUG] No route selected, checking active route:', {
                    hasActiveRouteData: !!activeRouteData,
                    hasWaypoints: !!(activeRouteData && activeRouteData.waypoints),
                    waypointsCount: activeRouteData?.waypoints?.length || 0
                });
                
                if (activeRouteData && activeRouteData.waypoints && activeRouteData.waypoints.length > 0) {
                    // Waypoints are already in correct format: {latitude, longitude, altitude}
                    // Use them directly from robot state (they reflect the actual current route)
                    const waypoints = activeRouteData.waypoints.map(wp => ({
                        latitude: wp.latitude,
                        longitude: wp.longitude
                        // altitude is available but not needed for visualization
                    }));
                    
                    console.log('[DEBUG] Using active route waypoints:', waypoints.length, 'waypoints');
                    
                    if (waypoints.length > 0) {
                        currentRoute = {
                            waypoints: waypoints,
                            loop_mode: false, // Will be overridden by mode
                            mode: activeRouteMode,
                            name: activeRouteName || 'Active Route' // Use name for display if available
                        };
                        console.log('[DEBUG] Calling updateRouteVisualization with currentRoute:', currentRoute);
                        updateRouteVisualization();
                    } else {
                        console.log('[DEBUG] No waypoints after mapping, setting currentRoute to null');
                        currentRoute = null;
                    }
                } else if (activeRouteName) {
                    // Fallback: if we have name but no waypoints, try to load from API
                    // (This shouldn't normally happen, but handles edge cases)
                    try {
                        const response = await fetch(`/api/routes/${activeRouteName}`);
                        const data = await response.json();
                        currentRoute = data;
                        // Override mode if we have it from robot state
                        if (activeRouteMode !== null) {
                            currentRoute.mode = activeRouteMode;
                        }
                        updateRouteVisualization();
                    } catch (e) {
                        console.error('Failed to load active route:', e);
                        currentRoute = null;
                    }
                } else {
                    currentRoute = null;
                }
                return;
            }
            
            // Route selected - show the selected route
            const response = await fetch(`/api/routes/${routeName}`);
            const data = await response.json();
            
            currentRoute = data;
            
            updateRouteVisualization();
        }

        // Helper: Clear route visualization only (not editor)
        function clearRouteVisualization() {
            if (routeVisualization.path) {
                map.removeLayer(routeVisualization.path);
                routeVisualization.path = null;
            }
            routeVisualization.markers.forEach(m => map.removeLayer(m));
            routeVisualization.markers = [];
        }
        
        // Helper: Clear editor visualization only (not route)
        function clearEditorVisualization() {
            if (editorVisualization.path) {
                map.removeLayer(editorVisualization.path);
                editorVisualization.path = null;
            }
            editorVisualization.markers.forEach(m => map.removeLayer(m));
            editorVisualization.markers = [];
        }

        function updateRouteInfoDisplay(activeRoute) {
            const routeInfoEl = document.getElementById('route-info');
            const routeNameEl = document.getElementById('route-name-display');
            const routeDistanceEl = document.getElementById('route-distance');
            
            if (activeRoute && activeRoute.name && activeRoute.distance_meters !== undefined && activeRoute.distance_meters !== null) {
                // Show route info
                routeInfoEl.style.display = 'flex';
                routeNameEl.textContent = activeRoute.name;
                
                // Format distance: show in meters, or km if > 1000m
                const distance = activeRoute.distance_meters;
                if (distance >= 1000) {
                    routeDistanceEl.textContent = `${(distance / 1000).toFixed(2)} km`;
                } else {
                    routeDistanceEl.textContent = `${Math.round(distance)} m`;
                }
            } else {
                // Hide route info
                routeInfoEl.style.display = 'none';
                routeNameEl.textContent = '';
                routeDistanceEl.textContent = '';
            }
        }

        function updateRouteVisualization() {
            console.log('[DEBUG] updateRouteVisualization called, currentRoute:', currentRoute);
            if (!currentRoute) {
                console.log('[DEBUG] updateRouteVisualization: No currentRoute, returning');
                return;
            }
            
            // Clear and redraw route visualization (not editor)
            clearRouteVisualization();
            
            // Determine loop mode: use mode from robot state if available, otherwise use route's loop_mode
            // Mode values: 0=ONCE, 1=LOOP, 2=PING_PONG, 4=STOP
            let isLoopMode = false;
            if (currentRoute.mode !== undefined && currentRoute.mode !== null) {
                isLoopMode = (currentRoute.mode === 1); // LOOP mode
            } else {
                isLoopMode = currentRoute.loop_mode || false;
            }
            
            const waypoints = currentRoute.waypoints;
            
            waypoints.forEach((wp, idx) => {
                const marker = L.circleMarker([wp.latitude, wp.longitude], {
                    radius: 8,
                    fillColor: '#2196F3',
                    color: '#fff',
                    weight: 2,
                    fillOpacity: 0.8
                }).addTo(map);
                
                marker.bindPopup(`<b>Punkt ${idx + 1}</b><br>${wp.latitude.toFixed(6)}, ${wp.longitude.toFixed(6)}`);
                routeVisualization.markers.push(marker);
                
                const icon = L.divIcon({
                    className: 'waypoint-label',
                    html: `<div style="background: #2196F3; color: white; border-radius: 50%; width: 20px; height: 20px; display: flex; align-items: center; justify-content: center; font-weight: bold; font-size: 12px;">${idx + 1}</div>`,
                    iconSize: [20, 20]
                });
                const numMarker = L.marker([wp.latitude, wp.longitude], { icon }).addTo(map);
                routeVisualization.markers.push(numMarker);
            });
            
            const coords = waypoints.map(wp => [wp.latitude, wp.longitude]);
            
            // For LOOP mode (1), connect last point with first point
            if (isLoopMode && coords.length > 0) {
                coords.push(coords[0]);
            }
            
            if (coords.length > 1) {
                // Determine line style based on mode
                let dashArray = null;
                let lineColor = '#2196F3';
                
                if (currentRoute.mode !== undefined && currentRoute.mode !== null) {
                    if (currentRoute.mode === 1) {
                        // LOOP: solid line
                        dashArray = null;
                    } else if (currentRoute.mode === 2) {
                        // PING_PONG: dashed line (forward path)
                        dashArray = '10, 10';
                    } else if (currentRoute.mode === 4) {
                        // STOP: red dashed line
                        dashArray = '10, 10';
                        lineColor = '#f44336';
                    } else {
                        // ONCE (0) or other: dashed line
                        dashArray = '10, 10';
                    }
                } else {
                    // Fallback to loop_mode boolean
                    dashArray = isLoopMode ? null : '10, 10';
                }
                
                routeVisualization.path = L.polyline(coords, {
                    color: lineColor,
                    weight: 3,
                    opacity: 0.7,
                    dashArray: dashArray
                }).addTo(map);
                
                map.fitBounds(routeVisualization.path.getBounds(), { padding: [50, 50] });
            }
        }

        // Camera names map
        const cameraNames = {
            'main': 'RGB Kamera',
            'person_detection': 'Person Detection',
            'thermal1': 'Thermal Kamera 1',
            'thermal2': 'Thermal Kamera 2',
            'rgb2': 'RGB Kamera 2',
            'lidar_debug': 'LiDAR Debug',
            'depth_debug': 'Tiefe (RealSense)'
        };

        let isCameraModalOpen = false;

        function openCameraModal() {
            const modal = document.getElementById('camera-modal');
            const modalTitle = document.getElementById('camera-modal-title');
            const feed = document.getElementById('camera-feed');
            document.getElementById('camera-modal-feed-wrapper').appendChild(feed);
            feed.style.maxWidth = '100vw';
            feed.style.maxHeight = 'calc(100vh - 48px)';
            feed.style.width = '100%';
            feed.style.height = '100%';
            feed.style.objectFit = 'contain';
            modalTitle.textContent = cameraNames[currentCameraStream] || 'Kamera';
            modal.classList.add('active');
            isCameraModalOpen = true;
        }

        function closeCameraModal() {
            const modal = document.getElementById('camera-modal');
            const feed = document.getElementById('camera-feed');
            const container = document.getElementById('camera-container');
            container.appendChild(feed);
            feed.style.maxWidth = '';
            feed.style.maxHeight = '';
            feed.style.width = '';
            feed.style.height = '';
            feed.style.objectFit = '';
            modal.classList.remove('active');
            isCameraModalOpen = false;
        }

        function openCameraStream(stream, btn) {
            switchCamera(stream, btn);
            openCameraModal();
        }

        document.addEventListener('keydown', (e) => {
            if (e.key === 'Escape') {
                closeCameraModal();
                if (document.getElementById('map-panel').classList.contains('map-fullscreen')) {
                    toggleMapFullscreen();
                }
            }
        });

        function expandPanel(panelId, event) {
            if (event && (event.target.closest('button') || event.target.closest('select'))) return;
            if (window.innerWidth >= 1024) return;
            const panel = document.getElementById(panelId);
            if (panel.classList.contains('panel-expanded')) return;
            document.querySelectorAll('.panel-expanded').forEach(p => p.classList.remove('panel-expanded'));
            panel.classList.add('panel-expanded');
            if (panelId === 'map-panel' && map) setTimeout(() => map.invalidateSize(), 50);
        }

        function collapsePanel() {
            const expanded = document.querySelector('.panel-expanded');
            if (expanded) {
                expanded.classList.remove('panel-expanded');
                if (map) setTimeout(() => map.invalidateSize(), 50);
            }
        }

        function toggleMapFullscreen() {
            const panel = document.getElementById('map-panel');
            const btn = document.getElementById('map-fullscreen-btn');
            const isFullscreen = panel.classList.toggle('map-fullscreen');
            document.body.classList.toggle('map-fullscreen-active', isFullscreen);
            btn.textContent = isFullscreen ? '✕ Schließen' : '⛶ Vollbild';
            setTimeout(() => { if (map) map.invalidateSize(); }, 50);
        }

        function switchCamera(stream, btn) {
    console.log('Switching camera to:', stream);

    // Set flag to prevent auto-reconnect
    isSwitchingCamera = true;

    currentCameraStream = stream;

    // Alle Buttons als inaktiv markieren
    document.querySelectorAll('.camera-btn').forEach(b => {
        b.classList.remove('active');
    });

    // Geklickten Button aktiv markieren + Header-Label aktualisieren
    if (btn) {
        btn.classList.add('active');
        const labelText = btn.dataset.camLabel || btn.textContent.trim();
        const headerLabel = document.getElementById('camera-current-label');
        if (headerLabel) headerLabel.textContent = labelText;
    }

    // WICHTIG: Altes Bild sofort entfernen und Lade-Overlay zeigen
    const container = document.getElementById('camera-container');
    const img = document.getElementById('camera-feed');
    const loading = document.getElementById('camera-loading');
    container.classList.remove('has-frame');
    container.classList.add('loading');
    img.removeAttribute('src');
    if (loading) loading.style.display = 'flex';
    lastCameraFrameTime = 0;
    lastDisplayedFrameTimestamp = 0; // Reset frame timestamp when switching cameras

    // WebSocket schließen und neu verbinden
    if (wsCamera) {
        console.log('Closing old camera WebSocket');
        // Remove all event handlers to prevent auto-reconnect
        wsCamera.onclose = null;
        wsCamera.onerror = null;
        wsCamera.onmessage = null;
        wsCamera.close();
        wsCamera = null;
    }

    // Small delay to ensure old connection is fully closed
    setTimeout(() => {
        isSwitchingCamera = false;
        connectCameraWebSocket();
    }, 100);
}

// UI polish: hide the loading overlay once a frame actually renders.
// Called from the camera-feed img.onload handler (set up below).
function hideCameraLoading() {
    const loading = document.getElementById('camera-loading');
    const container = document.getElementById('camera-container');
    if (loading) loading.style.display = 'none';
    if (container) container.classList.remove('loading');
}

// Camera tab switcher: 📺 Live vs 🔧 Debug.
// The previous "Technische Ansichten anzeigen" toggle was replaced with a
// proper two-tab UI per operator request — LiDAR + 3D-Tiefe live in a
// completely separate Debug tab now, not mixed in with live cameras.
function switchCameraTab(tab, btn) {
    if (tab !== 'live' && tab !== 'debug') tab = 'live';
    const controls = document.getElementById('camera-controls');
    const liveBtn = document.getElementById('cam-tab-btn-live');
    const debugBtn = document.getElementById('cam-tab-btn-debug');
    if (!controls) return;
    controls.classList.toggle('cam-tab-live', tab === 'live');
    controls.classList.toggle('cam-tab-debug', tab === 'debug');
    if (liveBtn) {
        liveBtn.classList.toggle('active', tab === 'live');
        liveBtn.setAttribute('aria-selected', tab === 'live' ? 'true' : 'false');
    }
    if (debugBtn) {
        debugBtn.classList.toggle('active', tab === 'debug');
        debugBtn.setAttribute('aria-selected', tab === 'debug' ? 'true' : 'false');
    }
    try { localStorage.setItem('camera_active_tab', tab); } catch (e) {}
}
// Apply persisted tab preference on load (defaults to 'live').
document.addEventListener('DOMContentLoaded', () => {
    try {
        const saved = localStorage.getItem('camera_active_tab') || 'live';
        const btn = document.getElementById('cam-tab-btn-' + saved);
        switchCameraTab(saved, btn);
    } catch (e) {}
});
    function connectCameraWebSocket() {
        // Don't create new connection if we're switching cameras
        if (isSwitchingCamera) {
            return;
        }
        
        const wsUrl = `ws://${window.location.host}/ws/camera/${currentCameraStream}`;
        console.log('Connecting to camera WebSocket:', wsUrl);
        
        // Make sure old connection is closed
        if (wsCamera && (wsCamera.readyState === WebSocket.OPEN || wsCamera.readyState === WebSocket.CONNECTING)) {
            console.log('Closing existing camera WebSocket before creating new one');
            wsCamera.onclose = null;
            wsCamera.onerror = null;
            wsCamera.onmessage = null;
            wsCamera.close();
        }
        
        wsCamera = new WebSocket(wsUrl);
        // CRITICAL: Set binary type for raw JPEG reception (thermal cameras)
        wsCamera.binaryType = 'arraybuffer';
        
        // Reusable object URL to prevent memory leaks
        let currentObjectUrl = null;
        
        wsCamera.onopen = () => {
            console.log('Camera WebSocket opened:', currentCameraStream);
        };
        
        wsCamera.onmessage = (event) => {
            const container = document.getElementById('camera-container');
            const img = document.getElementById('camera-feed');
            
            // Handle both binary (new fast mode) and JSON (legacy mode)
            // Check for binary data: ArrayBuffer OR Blob (Firefox uses Blob sometimes)
            if (event.data instanceof ArrayBuffer || event.data instanceof Blob) {
                // BINARY MODE - thermal cameras send raw JPEG bytes
                // This is ~33% smaller and much faster than Base64+JSON
                
                // Revoke old URL to prevent memory leak
                if (currentObjectUrl) {
                    URL.revokeObjectURL(currentObjectUrl);
                }
                
                // Handle both ArrayBuffer and Blob
                let blob;
                if (event.data instanceof Blob) {
                    blob = event.data;
                } else {
                    blob = new Blob([event.data], { type: 'image/jpeg' });
                }
                
                currentObjectUrl = URL.createObjectURL(blob);
                img.src = currentObjectUrl;
                if (isCameraModalOpen) {
                    const modalFeed = document.getElementById('camera-modal-feed');
                    if (modalFeed) modalFeed.src = currentObjectUrl;
                }
                container.classList.add('has-frame');
                hideCameraLoading();
                lastCameraFrameTime = Date.now();
            } else if (typeof event.data === 'string') {
                // JSON MODE - legacy format for main camera
                try {
                    const data = JSON.parse(event.data);
                    if (data.type === 'camera') {
                        // Skip old frames if timestamp is available
                        if (data.timestamp && data.timestamp <= lastDisplayedFrameTimestamp) {
                            return;
                        }
                        
                        img.src = 'data:image/jpeg;base64,' + data.data;
                        container.classList.add('has-frame');
                        hideCameraLoading();
                        lastCameraFrameTime = Date.now();

                        if (data.timestamp) {
                            lastDisplayedFrameTimestamp = data.timestamp;
                        }
                    }
                } catch (e) {
                    console.warn('Failed to parse camera message:', e);
                }
            }
        };
        
        wsCamera.onerror = (error) => {
            console.error('Camera WebSocket error:', error);
            if (!isSwitchingCamera && currentCameraStream) {
                setTimeout(() => {
                    if (!isSwitchingCamera && currentCameraStream) {
                        console.log('Reconnecting camera WebSocket...');
                        connectCameraWebSocket();
                    }
                }, 2000);
            }
        };
        
        wsCamera.onclose = () => {
            console.log('Camera WebSocket closed');
            // Cleanup object URL on close
            if (currentObjectUrl) {
                URL.revokeObjectURL(currentObjectUrl);
                currentObjectUrl = null;
            }
            if (!isSwitchingCamera && currentCameraStream) {
                setTimeout(() => {
                    if (!isSwitchingCamera && currentCameraStream) {
                        console.log('Attempting camera WebSocket reconnect...');
                        connectCameraWebSocket();
                    }
                }, 2000);
            }
        };
    }
        async function startSelectedRouteNow() {
            const sel = document.getElementById('route-select');
            let routeName = sel.value;
            if (!routeName) {
                const opts = Array.from(sel.options).filter(o => o.value);
                if (opts.length === 0) {
                    alert('Keine Route vorhanden. Bitte zuerst eine Route erstellen.');
                    return;
                }
                routeName = opts[0].value;
                sel.value = routeName;
                loadSelectedRoute();
            }
            if (!confirm(`Route "${routeName}" jetzt starten?`)) {
                return;
            }
            try {
                const response = await fetch(`/api/routes/start_now/${encodeURIComponent(routeName)}`, { method: 'POST' });
                const data = await response.json();
                if (data.status === 'success') {
                    alert(`✅ Route "${data.route}" gestartet.`);
                } else {
                    alert('❌ ' + (data.message || 'Unbekannter Fehler'));
                }
            } catch (e) {
                alert('❌ Verbindungsfehler: ' + e.message);
            }
        }

        async function deleteCurrentRoute() {
            const routeName = document.getElementById('route-select').value;
            
            if (!routeName) {
                alert('Bitte erst eine Route auswählen!');
                return;
            }
            
            if (!confirm(`Route "${routeName}" wirklich löschen?`)) {
                return;
            }
            
            const response = await fetch(`/api/routes/delete/${routeName}`, { method: 'DELETE' });
            const data = await response.json();
            
            if (data.status === 'success') {
                // Liste neu laden
                await loadRoutes();

                // Karte aufräumen
                if (routePath) map.removeLayer(routePath);
                routeMarkers.forEach(m => map.removeLayer(m));
                routeMarkers = [];
                currentRoute = null;
                document.getElementById('route-select').value = '';

                const cleaned = Array.isArray(data.cleaned_schedules) ? data.cleaned_schedules : [];
                const deact   = Array.isArray(data.deactivated_schedules) ? data.deactivated_schedules : [];
                let msg = '✅ Route gelöscht!';
                if (cleaned.length) {
                    msg += `\n\n• Aus ${cleaned.length} Zeitplan/Zeitplänen entfernt: ${cleaned.join(', ')}`;
                }
                if (deact.length) {
                    msg += `\n• ${deact.length} Zeitplan/Zeitpläne deaktiviert (keine Routen mehr): ${deact.join(', ')}`;
                }
                alert(msg);
            } else {
                // Fehlermeldung vom Backend anzeigen
                alert('❌ ' + data.message);
            }
        }
        
        // ── Route recording (PS5 Circle button or screen tap) ──────────
        let isRecording = false;
        let recordedWaypoints = [];
        let recordingMarkers = [];

        function startRecording() {
            isRecording = true;
            recordedWaypoints = [];
            recordingMarkers.forEach(m => map.removeLayer(m));
            recordingMarkers = [];
            document.getElementById('recording-panel').style.display = 'block';
            document.getElementById('recording-count').textContent = '0 Punkte';
        }

        function onWaypointSaved(wp) {
            if (!isRecording) return;
            recordedWaypoints.push(wp);
            document.getElementById('recording-count').textContent = recordedWaypoints.length + (recordedWaypoints.length === 1 ? ' Punkt' : ' Punkte');
            const marker = L.circleMarker([wp.latitude, wp.longitude], {
                radius: 8, color: '#ef4444', fillColor: '#ef4444', fillOpacity: 0.7
            }).addTo(map);
            marker.bindTooltip(String(recordedWaypoints.length), {permanent: true, direction: 'center', className: 'wp-number-label'});
            recordingMarkers.push(marker);
            map.panTo([wp.latitude, wp.longitude]);
        }

        async function saveWaypointFromScreen() {
            try {
                const res = await fetch('/api/control/save_waypoint', { method: 'POST' });
                const data = await res.json();
                if (data.status === 'success') {
                    onWaypointSaved(data.waypoint);
                } else {
                    alert(data.message || 'Fehler beim Speichern');
                }
            } catch (e) {
                alert('Verbindungsfehler: ' + e.message);
            }
        }

        async function stopRecording() {
            if (recordedWaypoints.length < 2) {
                alert('Mindestens 2 Punkte nötig!');
                return;
            }
            const name = prompt('Name für die neue Route:');
            if (!name || !name.trim()) return;
            try {
                const res = await fetch('/api/routes/save', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ name: name.trim(), loop_mode: false, waypoints: recordedWaypoints })
                });
                const data = await res.json();
                if (data.status === 'success') {
                    alert('✅ Route "' + name.trim() + '" gespeichert (' + recordedWaypoints.length + ' Punkte)');
                    cancelRecording();
                    loadRoutes();
                } else {
                    alert('❌ ' + (data.message || 'Fehler'));
                }
            } catch (e) {
                alert('Fehler: ' + e.message);
            }
        }

        function cancelRecording() {
            isRecording = false;
            recordedWaypoints = [];
            recordingMarkers.forEach(m => map.removeLayer(m));
            recordingMarkers = [];
            document.getElementById('recording-panel').style.display = 'none';
        }

        async function toggleLight() {
            // Wenn ein Kommando noch läuft → ignorieren
            if (lightPending) return;

            const btn = document.getElementById('light-btn');
            const text = document.getElementById('light-text');

            lightPending = true;
            btn.disabled = true;                // UI blockieren
            const oldText = text.textContent;   // alten Text merken
            text.textContent = 'Bitte warten…';

            try {
                const response = await fetch('/api/control/light', { 
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ state: !lightStatus })
                });
                
                if (!response.ok) {
                    throw new Error(`HTTP error! status: ${response.status}`);
                }
                
                const data = await response.json();

                if (data.status === 'success') {
                    // Backend gibt nur success zurück, wenn ACK vom Roboter da ist
                    updateLightButton(data.light_status);
                    // Backend will send log message via WebSocket
                } else {
                    // Server returned error response - backend already logged the error
                    alert('Fehler: ' + data.message);
                    // UI auf tatsächlichen Status zurücksetzen (kommt eh per WS regelmäßig)
                    fetch('/api/control/light')
                        .then(r => r.json())
                        .then(d => updateLightButton(d.light_status));
                }
            } catch (e) {
                // Communication/network error
                addLogEntry({
                    timestamp: new Date().toLocaleTimeString(),
                    message: `❌ Kommunikationsfehler: ${e.message}`,
                    isError: true
                });
                alert('Fehler: ' + e.message);
            } finally {
                lightPending = false;
                btn.disabled = false;
                text.textContent = oldText;
            }
        }


        function updateLightButton(status) {
            lightStatus = status;
            const btn = document.getElementById('light-btn');
            const text = document.getElementById('light-text');
            if (status) {
                btn.className = 'btn-light-on btn-large';
                text.textContent = 'Licht Ein';
            } else {
                btn.className = 'btn-light-off btn-large';
                text.textContent = 'Licht Aus';
            }
        }
        
        let chargingPending = false;
        let chargingStatus = false;
        
        function updateChargingButton(status) {
            chargingStatus = status;
            const btn = document.getElementById('charging-btn');
            const text = document.getElementById('charging-text');
            if (status) {
                btn.className = 'btn-charging-on btn-large';
                text.textContent = 'Laden Ein';
            } else {
                btn.className = 'btn-charging-off btn-large';
                text.textContent = 'Laden Aus';
            }
        }
        
        async function toggleCharging() {
            if (chargingPending) return;

            const newState = !chargingStatus;

            if (!newState && chargingStatus) {
                if (!confirm('Ladevorgang wirklich beenden?')) return;
            }

            const btn = document.getElementById('charging-btn');
            const text = document.getElementById('charging-text');

            chargingPending = true;
            btn.disabled = true;
            const oldText = text.textContent;
            text.textContent = 'Bitte warten…';

            try {
                const response = await fetch('/api/control/charge/manual', { 
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ state: newState })
                });
                
                if (!response.ok) {
                    throw new Error(`HTTP error! status: ${response.status}`);
                }
                
                const data = await response.json();

                if (data.status === 'success') {
                    updateChargingButton(data.charging_enabled);
                } else {
                    alert('Fehler: ' + data.message);
                    text.textContent = oldText;
                }
            } catch (e) {
                addLogEntry({
                    timestamp: new Date().toLocaleTimeString(),
                    message: `❌ Kommunikationsfehler: ${e.message}`,
                    isError: true
                });
                alert('Fehler: ' + e.message);
                text.textContent = oldText;
            } finally {
                chargingPending = false;
                btn.disabled = false;
            }
        }

        // Route Editor - MIT HOME-ICON VERSTECKEN (FIX 3)
        function openRouteEditor(mode) {
            // Home Position prüfen
            if (!homePosition) {
                alert('⚠️ Bitte erst die Home Position in den Einstellungen festlegen!');
                window.location.href = '/settings';
                return;
            }
            
            editorVisualization.mode = mode;
            editorVisualization.waypoints = [];
            
            // BEIDE Marker entfernen wenn Editor öffnet
            if (homeMarker) {
                map.removeLayer(homeMarker);
                homeMarker = null;
            }
            if (chargeMarker) {
                map.removeLayer(chargeMarker);
                chargeMarker = null;
            }
            
            const sidebar = document.getElementById('route-editor');
            const title = document.getElementById('editor-title');
            
            if (mode === 'new') {
                title.textContent = 'Neue Route';
                document.getElementById('editor-route-name').value = '';
                document.getElementById('editor-loop-mode').checked = false;
                
                // Home Position als ersten Punkt hinzufügen
                editorVisualization.waypoints.push({
                    latitude: homePosition.latitude,
                    longitude: homePosition.longitude,
                    altitude: 0.0
                });
                
                // Karte auf Home Position zentrieren
                map.setView([homePosition.latitude, homePosition.longitude], 18);
            } else {
                if (!currentRoute) {
                    alert('Bitte erst eine Route auswählen!');
                    return;
                }
                title.textContent = 'Route Bearbeiten: ' + currentRoute.name;
                document.getElementById('editor-route-name').value = currentRoute.name;
                document.getElementById('editor-loop-mode').checked = currentRoute.loop_mode;
                editorVisualization.waypoints = [...currentRoute.waypoints];
                
                // Bei vorhandenen Waypoints zum ersten zentrieren
                if (editorVisualization.waypoints.length > 0) {
                    map.setView([editorVisualization.waypoints[0].latitude, editorVisualization.waypoints[0].longitude], 18);
                }
            }
            
            sidebar.classList.add('open');
            document.body.classList.add('editor-active');
            
            // Map nach DOM-Update neu berechnen
            setTimeout(() => {
                map.invalidateSize();
            }, 350);
            
            visualizeEditorWaypoints();
        }

        function closeRouteEditor() {
            editorVisualization.mode = null;
            const sidebar = document.getElementById('route-editor');
            
            sidebar.classList.remove('open');
            document.body.classList.remove('editor-active');
            
            // Clear editor visualization only
            clearEditorVisualization();
            editorVisualization.waypoints = [];
            
            // Home-Icon wieder anzeigen nach Editor-Schließen
            loadHomePosition();

            
            if (currentRoute) updateRouteVisualization();

            setTimeout(() => map.invalidateSize(), 350);
        }

        function addEditorWaypoint(lat, lon) {
            editorVisualization.waypoints.push({ latitude: lat, longitude: lon, altitude: 0.0 });
            visualizeEditorWaypoints();
        }

        function handleSelectButtonPress(positionData) {
            // Only add waypoint if we're in route editor mode
            if (editorVisualization.mode === 'new' || editorVisualization.mode === 'edit') {
                const lat = positionData.latitude;
                const lon = positionData.longitude;
                const alt = positionData.altitude || 0.0;
                
                // Add waypoint at current robot position
                addEditorWaypoint(lat, lon);
                
                // Optional: Show visual feedback
                console.log(`Select button pressed: Added waypoint at ${lat.toFixed(6)}, ${lon.toFixed(6)}`);
                
                // Optional: Flash the map to show waypoint was added
                if (map) {
                    map.setView([lat, lon], map.getZoom(), { animate: true, duration: 0.3 });
                }
            } else {
                // Not in editor mode - just log the button press
                console.log('Select button pressed, but not in route editor mode');
            }
        }

        function visualizeEditorWaypoints() {
            // Clear editor visualization only (not route)
            clearEditorVisualization();
            
            const listDiv = document.getElementById('waypoint-list');
            if (editorVisualization.waypoints.length === 0) {
                listDiv.innerHTML = '<p style="color: #888; text-align: center;">Noch keine Waypoints</p>';
            } else {
                listDiv.innerHTML = '';
                editorVisualization.waypoints.forEach((wp, idx) => {
                    const item = document.createElement('div');
                    item.className = 'waypoint-item';
                    
                    const isHome = idx === 0 && homePosition && 
                                   Math.abs(wp.latitude - homePosition.latitude) < 0.000001 &&
                                   Math.abs(wp.longitude - homePosition.longitude) < 0.000001;
                    
                    item.innerHTML = `
                        <span class="drag-handle">☰</span>
                        <div class="waypoint-info">
                            <span class="wp-number">${isHome ? '🏠' : '#' + (idx + 1)}</span>
                            <span class="wp-coords">${wp.latitude.toFixed(6)}, ${wp.longitude.toFixed(6)}</span>
                        </div>
                        ${isHome ? '' : `<button class="btn-delete-wp" onclick="deleteEditorWaypoint(${idx})">🗑️</button>`}
                    `;
                    
                    // Make item draggable (except home position)
                    if (!isHome) {
                        item.draggable = true;
                        item.dataset.index = idx;
                        
                        item.addEventListener('dragstart', handleWaypointDragStart);
                        item.addEventListener('dragend', handleWaypointDragEnd);
                        item.addEventListener('dragover', handleWaypointDragOver);
                        item.addEventListener('drop', handleWaypointDrop);
                        item.addEventListener('dragleave', handleWaypointDragLeave);
                    } else {
                        item.style.cursor = 'default';
                        item.querySelector('.drag-handle').style.opacity = '0.3';
                        item.querySelector('.drag-handle').style.cursor = 'not-allowed';
                    }
                    
                    listDiv.appendChild(item);
                });
                
                // Add a drop zone at the end for moving items to the last position
                const dropZone = document.createElement('div');
                dropZone.className = 'waypoint-drop-zone';
                // dropZone.innerHTML = '<span style="color: #888; font-size: 0.85em;">↓ Hierher ziehen um an das Ende zu verschieben</span>';
                dropZone.addEventListener('dragover', handleDropZoneDragOver);
                dropZone.addEventListener('drop', handleDropZoneDrop);
                dropZone.addEventListener('dragleave', handleDropZoneDragLeave);
                listDiv.appendChild(dropZone);
            }
            
            editorVisualization.waypoints.forEach((wp, idx) => {
                const isHome = idx === 0 && homePosition && 
                               Math.abs(wp.latitude - homePosition.latitude) < 0.000001 &&
                               Math.abs(wp.longitude - homePosition.longitude) < 0.000001;
                
                // Create draggable marker (except for home position)
                const markerOptions = {
                    radius: 8,
                    fillColor: '#FF9800',
                    color: '#fff',
                    weight: 2,
                    fillOpacity: 0.8,
                    draggable: !isHome  // Only allow dragging non-home waypoints
                };
                
                const marker = L.circleMarker([wp.latitude, wp.longitude], markerOptions).addTo(map);
                
                // Set cursor style
                const markerElement = marker.getElement();
                if (markerElement) {
                    markerElement.style.cursor = isHome ? 'not-allowed' : 'move';
                }
                
                marker.bindPopup(`<b>Punkt ${idx + 1}</b><br>${wp.latitude.toFixed(6)}, ${wp.longitude.toFixed(6)}`);
                marker.waypointIndex = idx;  // Store index for drag handler
                
                // Add drag handlers for non-home waypoints
                if (!isHome) {
                    marker.on('drag', function(e) {
                        handleWaypointMarkerDrag(e, idx);
                    });
                    marker.on('dragend', function(e) {
                        handleWaypointMarkerDragEnd(e, idx);
                    });
                }
                
                editorVisualization.markers.push(marker);
                
                // Add number label
                const icon = L.divIcon({
                    className: 'waypoint-label',
                    html: `<div style="background: #FF9800; color: white; border-radius: 50%; width: 20px; height: 20px; display: flex; align-items: center; justify-content: center; font-weight: bold; font-size: 12px; cursor: ${isHome ? 'not-allowed' : 'move'};">${idx + 1}</div>`,
                    iconSize: [20, 20]
                });
                const numMarker = L.marker([wp.latitude, wp.longitude], { 
                    icon,
                    draggable: !isHome
                }).addTo(map);
                
                // Sync number marker drag with main marker
                if (!isHome) {
                    numMarker.on('drag', function(e) {
                        const newLatLng = e.target.getLatLng();
                        marker.setLatLng(newLatLng);
                        handleWaypointMarkerDrag(e, idx);
                    });
                    numMarker.on('dragend', function(e) {
                        const newLatLng = e.target.getLatLng();
                        marker.setLatLng(newLatLng);
                        handleWaypointMarkerDragEnd(e, idx);
                    });
                }
                
                editorVisualization.markers.push(numMarker);
            });
            
            if (editorVisualization.waypoints.length > 1) {
                const coords = editorVisualization.waypoints.map(wp => [wp.latitude, wp.longitude]);
                
                // Check if loop mode is enabled
                const loopMode = document.getElementById('editor-loop-mode').checked;
                if (loopMode && coords.length > 0) {
                    // Connect last waypoint to first waypoint for loop mode
                    coords.push(coords[0]);
                }
                
                editorVisualization.path = L.polyline(coords, {
                    color: '#FF9800',
                    weight: 3,
                    opacity: 0.7
                }).addTo(map);
            }
            
            document.getElementById('editor-info-text').textContent = 
                `📍 ${editorVisualization.waypoints.length} Punkte | Klicke auf die Karte um weitere hinzuzufügen`;
        }

        // Drag and Drop handlers for waypoint reordering
        let draggedWaypointIndex = null;

        function handleWaypointDragStart(e) {
            draggedWaypointIndex = parseInt(e.target.dataset.index);
            e.target.classList.add('dragging');
            e.dataTransfer.effectAllowed = 'move';
            e.dataTransfer.setData('text/html', e.target.innerHTML);
        }

        function handleWaypointDragEnd(e) {
            e.target.classList.remove('dragging');
            // Remove all drag-over classes
            document.querySelectorAll('.waypoint-item').forEach(item => {
                item.classList.remove('drag-over');
            });
        }

        function handleWaypointDragOver(e) {
            if (e.preventDefault) {
                e.preventDefault(); // Allows drop
            }
            e.dataTransfer.dropEffect = 'move';
            
            const targetIndex = parseInt(e.currentTarget.dataset.index);
            // Don't allow dropping on home position (index 0)
            if (targetIndex === 0) {
                return false;
            }
            
            e.currentTarget.classList.add('drag-over');
            return false;
        }

        function handleWaypointDragLeave(e) {
            e.currentTarget.classList.remove('drag-over');
        }

        function handleWaypointDrop(e) {
            if (e.stopPropagation) {
                e.stopPropagation(); // Stops some browsers from redirecting
            }
            
            const dropIndex = parseInt(e.currentTarget.dataset.index);
            
            // Don't allow dropping on home position (index 0)
            if (dropIndex === 0 || draggedWaypointIndex === 0) {
                return false;
            }
            
            // Don't drop on itself
            if (draggedWaypointIndex !== dropIndex) {
                // Remove the dragged item
                const draggedItem = editorVisualization.waypoints[draggedWaypointIndex];
                editorVisualization.waypoints.splice(draggedWaypointIndex, 1);
                
                // Insert at new position (adjust index if dragged from before drop position)
                const newIndex = draggedWaypointIndex < dropIndex ? dropIndex - 1 : dropIndex;
                editorVisualization.waypoints.splice(newIndex, 0, draggedItem);
                
                // Re-render waypoints
                visualizeEditorWaypoints();
            }
            
            return false;
        }

        function handleDropZoneDragOver(e) {
            if (e.preventDefault) {
                e.preventDefault();
            }
            e.dataTransfer.dropEffect = 'move';
            e.currentTarget.classList.add('drag-over');
            return false;
        }

        function handleDropZoneDragLeave(e) {
            e.currentTarget.classList.remove('drag-over');
        }

        function handleDropZoneDrop(e) {
            if (e.stopPropagation) {
                e.stopPropagation();
            }
            
            e.currentTarget.classList.remove('drag-over');
            
            // Don't allow moving home position
            if (draggedWaypointIndex === 0) {
                return false;
            }
            
            // Move item to end
            const draggedItem = editorVisualization.waypoints[draggedWaypointIndex];
            editorVisualization.waypoints.splice(draggedWaypointIndex, 1);
            editorVisualization.waypoints.push(draggedItem);
            
            // Re-render waypoints
            visualizeEditorWaypoints();
            
            return false;
        }

        // Handler for dragging waypoint markers on the map
        function handleWaypointMarkerDrag(e, idx) {
            const newLatLng = e.target.getLatLng();
            
            // Update the waypoint coordinates in real-time
            editorVisualization.waypoints[idx].latitude = newLatLng.lat;
            editorVisualization.waypoints[idx].longitude = newLatLng.lng;
            
            // Update the path line in real-time
            if (editorVisualization.path) {
                const coords = editorVisualization.waypoints.map(wp => [wp.latitude, wp.longitude]);
                const loopMode = document.getElementById('editor-loop-mode').checked;
                if (loopMode && coords.length > 0) {
                    coords.push(coords[0]);
                }
                editorVisualization.path.setLatLngs(coords);
            }
        }

        function handleWaypointMarkerDragEnd(e, idx) {
            const newLatLng = e.target.getLatLng();
            
            // Update final coordinates
            editorVisualization.waypoints[idx].latitude = newLatLng.lat;
            editorVisualization.waypoints[idx].longitude = newLatLng.lng;
            
            // Update the waypoint list display with new coordinates
            const listDiv = document.getElementById('waypoint-list');
            const items = listDiv.querySelectorAll('.waypoint-item');
            if (items[idx]) {
                const coordsSpan = items[idx].querySelector('.wp-coords');
                if (coordsSpan) {
                    coordsSpan.textContent = `${newLatLng.lat.toFixed(6)}, ${newLatLng.lng.toFixed(6)}`;
                }
            }
            
            // Update popup content
            e.target.setPopupContent(`<b>Punkt ${idx + 1}</b><br>${newLatLng.lat.toFixed(6)}, ${newLatLng.lng.toFixed(6)}`);
        }

        function toggleLoopMode() {
            const loopMode = document.getElementById('editor-loop-mode').checked;
            const reverseBtn = document.getElementById('reverse-route-btn');
            
            // Enable/disable reverse button based on loop mode
            reverseBtn.disabled = !loopMode;
            
            // Update visualization
            visualizeEditorWaypoints();
        }

        function reverseRouteOrder() {
            // Don't reverse if there are less than 2 waypoints (excluding home)
            if (editorVisualization.waypoints.length < 3) {
                alert('Mindestens 3 Waypoints erforderlich um die Route umzukehren!');
                return;
            }
            
            // Keep the first waypoint (home) in place, reverse the rest
            const home = editorVisualization.waypoints[0];
            const restWaypoints = editorVisualization.waypoints.slice(1);
            restWaypoints.reverse();
            
            editorVisualization.waypoints = [home, ...restWaypoints];
            
            // Re-render waypoints
            visualizeEditorWaypoints();
        }

        function deleteEditorWaypoint(idx) {
            // Ersten Punkt (Home) nicht löschen
            if (idx === 0) {
                alert('Der erste Punkt (Home Position) kann nicht gelöscht werden!');
                return;
            }
            editorVisualization.waypoints.splice(idx, 1);
            visualizeEditorWaypoints();
        }

        async function saveRoute() {
            const name = document.getElementById('editor-route-name').value.trim();
            const loopMode = document.getElementById('editor-loop-mode').checked;
            
            if (!name) {
                alert('Bitte Route-Namen eingeben!');
                return;
            }
            
            if (editorVisualization.waypoints.length < 2) {
                alert('Mindestens 2 Waypoints erforderlich (Home + 1 weiterer Punkt)!');
                return;
            }
            
            const response = await fetch('/api/routes/save', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    name: name,
                    loop_mode: loopMode,
                    waypoints: editorVisualization.waypoints
                })
            });
            
            const data = await response.json();
            if (data.status === 'success') {
                alert('Route gespeichert!');
                closeRouteEditor();
                await loadRoutes();
                document.getElementById('route-select').value = data.route;
                await loadSelectedRoute();
            } else {
                alert('Fehler: ' + data.message);
            }
        }

        async function startAutonomous() {
            if (!currentRoute) {
                alert('Bitte erst eine Route auswählen!');
                return;
            }
            
            try {
                const response = await fetch('/api/control/start', { 
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({
                        route: currentRoute.name,
                        loop_mode: currentRoute.loop_mode || false
                    })
                });
                
                if (!response.ok) {
                    throw new Error(`HTTP error! status: ${response.status}`);
                }
                
                const data = await response.json();
                
                if (data.status === 'success') {
                    // Update autonomous operation state (should be enabled when route starts)
                    autonomousOperationEnabled = true;
                    updateStopButton();
                    // Backend will send log message via WebSocket
                } else {
                    // Server returned error response - backend already logged the error
                    alert('Fehler: ' + data.message);
                }
            } catch (e) {
                // Communication/network error
                addLogEntry({
                    timestamp: new Date().toLocaleTimeString(),
                    message: `❌ Kommunikationsfehler: ${e.message}`,
                    isError: true
                });
                alert('Fehler: ' + e.message);
            }
        }

        async function stopAutonomous() {
            try {
                const response = await fetch('/api/control/stop', { method: 'POST' });
                
                if (!response.ok) {
                    throw new Error(`HTTP error! status: ${response.status}`);
                }
                
                const data = await response.json();
                
                if (data.status === 'success') {
                    autonomousOperationEnabled = data.autonomous_enabled;
                    updateStopButton();
                    // Backend will send log message via WebSocket
                } else {
                    // Server returned error response - backend already logged the error
                    alert('Fehler: ' + data.message);
                }
            } catch (e) {
                // Communication/network error
                addLogEntry({
                    timestamp: new Date().toLocaleTimeString(),
                    message: `❌ Kommunikationsfehler: ${e.message}`,
                    isError: true
                });
                alert('Fehler: ' + e.message);
            }
        }

        async function toggleAutonomousMode() {
            if (autonomousOperationEnabled) {
                if (!confirm('Automatik ausschalten? Der Roboter hält an.')) return;
            }
            await stopAutonomous();
        }

        function updateStopButton() {
            const btn = document.getElementById('autonomous-btn');
            if (!btn) return;

            const btnText = btn.querySelector('.btn-text');

            if (autonomousOperationEnabled) {
                btn.className = 'btn-start btn-large';
                btnText.textContent = 'Automatik Ein';
            } else {
                btn.className = 'btn-stop btn-large';
                btnText.textContent = 'Automatik Aus';
            }
        }

        async function loadAutonomousStatus() {
            try {
                const response = await fetch('/api/control/autonomous');
                const data = await response.json();
                autonomousOperationEnabled = data.autonomous_enabled || false;
                updateStopButton();
            } catch (e) {
                console.error('Failed to load autonomous status:', e);
            }
        }

        let connectionStatusEntry = null; // Track connection status log entry
        let lastLogEntry = null; // Track last log entry for deduplication
        const MAX_STORED_LOGS = 100; // Maximum number of logs to store in localStorage
        let loadedLogsMap = new Map(); // Track logs loaded from localStorage to prevent duplicates from backend buffer
        
        // Load logs from backend API
        async function loadLogs() {
            try {
                const logDiv = document.getElementById('event-log');
                if (!logDiv) {
                    console.warn('event-log div not found, retrying...');
                    setTimeout(loadLogs, 100);
                    return;
                }
                
                // Load logs from backend
                const response = await fetch('/api/logs');
                const data = await response.json();
                const logs = data.logs || [];
                
                console.log('Loading', logs.length, 'logs from backend');
                
                // Clear existing logs and reset tracking
                logDiv.innerHTML = '';
                connectionStatusEntry = null;
                lastLogEntry = null;
                loadedLogsMap.clear();
                
                // Restore logs (newest first - logs are stored with newest at index 0)
                // Filter out connection status messages and invalid logs - they should not be persisted
                const regularLogs = logs.filter(log => {
                    try {
                        const logData = typeof log === 'string' ? JSON.parse(log) : log;
                        // Filter out connection status messages and invalid entries
                        return logData.connection_status === undefined && 
                               logData.timestamp !== undefined && 
                               logData.message !== undefined;
                    } catch (e) {
                        console.warn('Invalid log entry, skipping:', log);
                        return false;
                    }
                });
                
                regularLogs.forEach((log, index) => {
                    // Handle both old format (with count) and new format (without count)
                    const logData = typeof log === 'string' ? JSON.parse(log) : log;
                    
                    // Skip if required fields are missing (double check after filter)
                    if (logData.timestamp === undefined || logData.message === undefined) {
                        console.warn('Log entry missing required fields, skipping:', logData);
                        return;
                    }
                    
                    const isError = logData.isError || logData.level === 'error' || logData.level === 'warn';
                    const count = logData.count || 1;
                    
                    const entryDiv = document.createElement('div');
                    entryDiv.className = isError ? 'log-entry error' : 'log-entry';
                    entryDiv.innerHTML = `<span class="log-timestamp">[${logData.timestamp}]</span>${logData.message}${count > 1 ? ` <span class="log-counter">(${count})</span>` : ''}`;
                    // Insert at the beginning (newest first)
                    logDiv.insertBefore(entryDiv, logDiv.firstChild);
                    
                    // Track loaded logs to prevent duplicates from backend buffer
                    // Use timestamp + message as key to identify same log entries
                    const logKey = `${logData.timestamp}|${logData.message}|${isError}`;
                    loadedLogsMap.set(logKey, {
                        element: entryDiv,
                        count: count
                    });
                    
                    // Track the first (newest) regular log entry for deduplication
                    if (index === 0) {
                        const messageKey = logData.message + '|' + (isError ? 'error' : 'info');
                        lastLogEntry = {
                            messageKey: messageKey,
                            element: entryDiv,
                            count: count
                        };
                    }
                });
            } catch (e) {
                console.error('Failed to load logs from backend:', e);
                loadedLogsMap.clear();
            }
        }
        
        // Save logs to localStorage (for backup, but primary storage is backend)
        function saveLogsToStorage() {
            // No longer needed - logs are stored in backend
            // Keep function for compatibility but don't save to localStorage
            return;
        }
        
        // Clear logs function - clears logs in backend for all clients
        async function clearLogs() {
            if (confirm('Möchten Sie wirklich alle Logs löschen? Dies betrifft alle Benutzer.')) {
                try {
                    const response = await fetch('/api/logs', { method: 'DELETE' });
                    const data = await response.json();
                    
                    if (data.status === 'success') {
                        // Clear local display
                        const logDiv = document.getElementById('event-log');
                        if (logDiv) {
                            logDiv.innerHTML = '';
                            connectionStatusEntry = null;
                            lastLogEntry = null;
                            loadedLogsMap.clear();
                        }
                        console.log('Logs cleared in backend');
                    } else {
                        alert('Fehler beim Löschen der Logs: ' + (data.message || 'Unbekannter Fehler'));
                    }
                } catch (e) {
                    console.error('Failed to clear logs:', e);
                    alert('Fehler beim Löschen der Logs: ' + e.message);
                }
            }
        }
        
        function addLogEntry(entry) {
            const logDiv = document.getElementById('event-log');
            if (!logDiv) return;
            
            // Validate entry has required fields (all entries need timestamp and message)
            if (!entry || entry.timestamp === undefined || entry.message === undefined) {
                console.warn('Invalid log entry, skipping (missing timestamp or message):', entry);
                return;
            }
            
            // Handle connection status messages specially
            if (entry.connection_status !== undefined) {
                
                // Remove ALL existing connection status messages from DOM
                const existingStatusEntries = logDiv.querySelectorAll('[data-connection-status="true"]');
                existingStatusEntries.forEach(statusEntry => {
                    if (statusEntry.parentNode) {
                        statusEntry.parentNode.removeChild(statusEntry);
                    }
                });
                connectionStatusEntry = null;
                
                if (entry.connection_status === false) {
                    // Verbindung weg -> Badge-Funktion mit 'null' aufrufen (Badges werden rot)
                    if (typeof updateFusionBadges === 'function') {
                        updateFusionBadges(null);
                    }
                    
                    // Add disconnected message
                    const entryDiv = document.createElement('div');
                    entryDiv.className = 'log-entry error';
                    entryDiv.innerHTML = `<span class="log-timestamp">[${entry.timestamp}]</span>${entry.message}`;
                    entryDiv.setAttribute('data-connection-status', 'true');
                    logDiv.insertBefore(entryDiv, logDiv.firstChild);
                    connectionStatusEntry = entryDiv;
                } else {
                    // Connection restored - all connection status messages already removed above
                    // The "Verbindung hergestellt" message is not shown, only the disconnect message is shown
                }
                return;
            }
            
            // Determine if this is an error log (check both isError and level fields)
            const isError = entry.isError || entry.level === 'error' || entry.level === 'warn';
            
            // Regular log entry - first check if this log was already loaded from backend
            const logKey = `${entry.timestamp}|${entry.message}|${isError}`;
            if (loadedLogsMap.has(logKey)) {
                // This log was already loaded from backend, ignore it from backend buffer
                console.log('Ignoring duplicate log from backend buffer:', entry.message);
                return;
            }
            
            // Check if this is a duplicate of the NEWEST (first) log entry only
            const messageKey = entry.message + '|' + (isError ? 'error' : 'info');
            
            // Only check the newest (first) log entry for duplicates
            const firstEntry = logDiv.querySelector('.log-entry:not([data-connection-status="true"])');
            
            if (firstEntry && lastLogEntry && lastLogEntry.element === firstEntry) {
                const existingMessage = firstEntry.textContent.replace(/\[.*?\]\s*/, '').replace(/\s*\(\d+\)$/, '');
                const existingIsError = firstEntry.classList.contains('error');
                
                // Only merge if it's the same message AND same error status as the newest entry
                if (existingMessage === entry.message && existingIsError === isError) {
                    // Same message as newest - increment counter
                    const counterSpan = firstEntry.querySelector('.log-counter');
                    const countMatch = firstEntry.textContent.match(/\((\d+)\)$/);
                    const currentCount = countMatch ? parseInt(countMatch[1]) : 1;
                    const newCount = currentCount + 1;
                    
                    if (counterSpan) {
                        counterSpan.textContent = ` (${newCount})`;
                    } else {
                        const newCounter = document.createElement('span');
                        newCounter.className = 'log-counter';
                        newCounter.textContent = ` (${newCount})`;
                        firstEntry.appendChild(newCounter);
                    }
                    
                    // Update timestamp to latest occurrence
                    const timestampSpan = firstEntry.querySelector('.log-timestamp');
                    if (timestampSpan) {
                        timestampSpan.textContent = `[${entry.timestamp}]`;
                    }
                    
                    // Update lastLogEntry tracking
                    lastLogEntry.count = newCount;
                    
                    return; // Don't create new entry
                }
            }
            
            // Different message or no existing entry - create new entry
            const entryDiv = document.createElement('div');
            entryDiv.className = isError ? 'log-entry error' : 'log-entry';
            entryDiv.innerHTML = `<span class="log-timestamp">[${entry.timestamp}]</span>${entry.message}`;
            
            logDiv.insertBefore(entryDiv, logDiv.firstChild);
            
            // Track this entry for deduplication
            lastLogEntry = {
                messageKey: messageKey,
                element: entryDiv,
                count: 1
            };
            
            while (logDiv.children.length > 50) {
                logDiv.removeChild(logDiv.lastChild);
            }
        }
        
        // Ergänzung Leon:
        function focusOnHome() {
        
            // Wir prüfen auf 'chargeMarker' und 'map'
            if (typeof chargeMarker !== 'undefined' && chargeMarker && map) {
                
                // 1. Koordinaten der Ladestation holen
                var latLng = chargeMarker.getLatLng();
                
                // 2. Intelligenten Zoom berechnen:
                // Wenn du aktuell näher dran bist als 18 (z.B. 19 oder 20), behalte deinen Zoom.
                // Wenn du weit weg bist, zoome auf 18 rein.
                var currentZoom = map.getZoom();
                var targetZoom = currentZoom > 18 ? currentZoom : 18;

                // 3. "flyTo" statt "setView"
                // Das sorgt für eine flüssige Flug-Animation zur Mitte
                map.flyTo(latLng, targetZoom, {
                    animate: true,
                    duration: 0.3 // Dauer des Fluges in Sekunden
                });

                // 4. Popup öffnen
                chargeMarker.openPopup();

            } else {
                console.warn("Ladestation-Marker (chargeMarker) wurde noch nicht geladen.");
                alert("Ladestation-Position ist noch nicht verfügbar.");
            }
        }


        // Init
        (async () => {
            // Check debug mode from backend
            try {
                const debugResponse = await fetch('/api/debug/enabled');
                const debugData = await debugResponse.json();
                if (debugData.debug_enabled) {
                    document.body.classList.add('debug-mode');
                    console.log('Debug mode enabled');
                }
            } catch (e) {
                console.warn('Could not check debug mode:', e);
            }

            await initMap();
            loadRoutes();
            loadAutonomousStatus();
            
            fetch('/api/events').then(r => r.json()).then(data => {
                if (data.events && Array.isArray(data.events)) {
                    data.events.forEach(entry => {
                        // Only add entries with required fields
                        if (entry && entry.timestamp !== undefined && entry.message !== undefined) {
                            addLogEntry(entry);
                        } else {
                            console.warn('Skipping invalid event entry:', entry);
                        }
                    });
                }
            }).catch(e => {
                console.error('Failed to load events:', e);
            });

            fetch('/api/control/light').then(r => r.json()).then(data => {
                updateLightButton(data.light_status);
            });
            
            fetch('/api/control/charge/status').then(r => r.json()).then(data => {
                updateChargingButton(data.charging_enabled);
            }).catch(e => {
                console.log('Could not load charging status:', e);
            });

            // Close modal when clicking outside of it
            const confirmModal = document.getElementById('confirm-modal');
            if (confirmModal) {
                confirmModal.addEventListener('click', function(e) {
                    if (e.target === this) {
                        closeConfirmModal();
                    }
                });
            }
        })();
        // Load logs from backend when DOM is ready
        if (document.readyState === 'loading') {
            document.addEventListener('DOMContentLoaded', loadLogs);
        } else {
            // DOM is already ready
            loadLogs();
        }

        // Cleanup intervals when leaving the page
        window.addEventListener('beforeunload', () => {
            intervals.forEach(id => clearInterval(id));
        });

        // Fallback for mobile browsers
        window.addEventListener('pagehide', () => {
            intervals.forEach(id => clearInterval(id));
        });
