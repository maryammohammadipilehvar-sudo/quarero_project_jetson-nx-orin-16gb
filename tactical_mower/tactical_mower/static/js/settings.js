        // Globale Variablen
        let currentPosition = null;
        let currentBattery = null;
        let currentFusionStatus = null;
        
        // Temporary storage für Charge Point (vor dem Speichern)
        let tempChargePoint = null;

        // Log management functions - only save to localStorage, don't display on this page
        const MAX_STORED_LOGS = 100;

        function addLogEntry(entry) {
            // On non-index pages, only save to localStorage, don't display
            // Get current logs from storage
            let logs = [];
            try {
                const storedLogs = localStorage.getItem('robot_logs');
                if (storedLogs) {
                    logs = JSON.parse(storedLogs);
                }
            } catch (e) {
                console.error('Failed to load logs from storage:', e);
            }

            // Handle connection status messages
            if (entry.connection_status !== undefined) {
                if (entry.connection_status === false) {
                    if (typeof updateFusionBadges === 'function') {
                        updateFusionBadges(null);
                    }
                    // Add disconnected message
                    logs.unshift({
                        timestamp: entry.timestamp,
                        message: entry.message,
                        isError: true,
                        connection_status: false,
                        count: 1
                    });
                } else {
                    // Remove disconnected messages when connected
                    logs = logs.filter(log => log.connection_status !== false);
                }
            } else {
                // Regular log entry - check for duplicates
                const existingIndex = logs.findIndex(log => 
                    log.message === entry.message && 
                    log.isError === (entry.isError || false) &&
                    log.connection_status === undefined
                );

                if (existingIndex !== -1) {
                    // Increment count and move to front (newest first)
                    const existingLog = logs[existingIndex];
                    existingLog.count++;
                    existingLog.timestamp = entry.timestamp;
                    // Remove from current position and add to front
                    logs.splice(existingIndex, 1);
                    logs.unshift(existingLog);
                } else {
                    // Add new entry at the front
                    logs.unshift({
                        timestamp: entry.timestamp,
                        message: entry.message,
                        isError: entry.isError || false,
                        count: 1
                    });
                }
            }

            // Keep only the most recent logs
            const logsToStore = logs.slice(0, MAX_STORED_LOGS);
            localStorage.setItem('robot_logs', JSON.stringify(logsToStore));
        }

        // WebSocket für Position
        const wsPosition = new WebSocket(`ws://${window.location.host}/ws/position`);
        wsPosition.onmessage = (event) => {
            const data = JSON.parse(event.data);
            if (data.type === 'position') {
                currentPosition = data.data;
                updateCurrentPositionDisplay();
            }
            if (data.type === 'fusion_status') {
                currentFusionStatus = data.data;
                updateFusionBadges(currentFusionStatus);
            } else if (data.type === 'event') {
                addLogEntry(data.data);
            }
        };

        // WebSocket für Robot State (Battery)
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
                currentBattery = stateData.battery;
                updateCurrentBatteryDisplay();
            }
        };

        // Track intervals for cleanup
        const intervals = [];

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

        function updateCurrentPositionDisplay() {
            if (!currentPosition || currentPosition.latitude === 0) {
                return;
            }

            const display = document.getElementById('current-position');
            display.textContent = `${currentPosition.latitude.toFixed(6)}, ${currentPosition.longitude.toFixed(6)}`;
        }

        function updateCurrentBatteryDisplay() {
            if (currentBattery === null || currentBattery === undefined) {
                return;
            }

            const display = document.getElementById('current-battery');
            display.textContent = `${currentBattery}%`;

            const card = document.getElementById('current-battery-card');
            if (currentBattery < 20) {
                card.style.borderLeftColor = '#f44336';
                display.style.color = '#f44336';
            } else if (currentBattery < 50) {
                card.style.borderLeftColor = '#FF9800';
                display.style.color = '#FF9800';
            } else {
                card.style.borderLeftColor = '#4CAF50';
                display.style.color = '#4CAF50';
            }
        }

        function updateBatteryDisplay() {
            const value = document.getElementById('battery-threshold').value;
            document.getElementById('battery-threshold-display').textContent = value;
        }

        function updateSpeedDisplay() {
            const value = document.getElementById('speed-factor').value;
            const kmh = parseFloat(value) * 2.6;
            document.getElementById('speed-factor-display').textContent = kmh.toFixed(1);
        }

        async function loadSettings() {
            try {
                // Charge/Home Points laden
                const homeResponse = await fetch('/api/settings/home-position');
                const homeData = await homeResponse.json();
                
                console.log('Loaded settings:', homeData);
                
                // Charge Point anzeigen
                const chargeDisplay = document.getElementById('charge-point-display');
                if (homeData.charge_point && homeData.charge_point.latitude) {
                    chargeDisplay.textContent = 
                        `${homeData.charge_point.latitude.toFixed(6)}, ${homeData.charge_point.longitude.toFixed(6)}`;
                    chargeDisplay.classList.remove('empty');
                    
                    // In temp speichern
                    tempChargePoint = {
                        latitude: homeData.charge_point.latitude,
                        longitude: homeData.charge_point.longitude
                    };
                } else {
                    chargeDisplay.textContent = 'Noch nicht gesetzt';
                    chargeDisplay.classList.add('empty');
                    tempChargePoint = null;
                }
                
                // Home Point anzeigen
                const homeDisplay = document.getElementById('home-point-display');
                if (homeData.home_point && homeData.home_point.latitude) {
                    homeDisplay.textContent = 
                        `${homeData.home_point.latitude.toFixed(6)}, ${homeData.home_point.longitude.toFixed(6)}`;
                    homeDisplay.classList.remove('empty');
                } else {
                    homeDisplay.textContent = 'Noch nicht gesetzt';
                    homeDisplay.classList.add('empty');
                }

                // Battery Settings laden
                const batteryResponse = await fetch('/api/settings/battery');
                const batteryData = await batteryResponse.json();
                
                if (batteryData.threshold) {
                    document.getElementById('battery-threshold').value = batteryData.threshold;
                    updateBatteryDisplay();
                }

                // Advanced Settings laden
                const advancedResponse = await fetch('/api/settings/advanced');
                const advancedData = await advancedResponse.json();
                
                if (advancedData.speed_factor !== undefined && advancedData.speed_factor !== null) {
                    const slider = document.getElementById('speed-factor');
                    slider.value = advancedData.speed_factor;
                    updateSpeedDisplay();
                }
                if (advancedData.auto_charge_return !== undefined) {
                    document.getElementById('auto-charge-return').checked = advancedData.auto_charge_return;
                }
                if (advancedData.max_route_distance !== undefined && advancedData.max_route_distance !== null) {
                    document.getElementById('max-route-distance').value = parseFloat(advancedData.max_route_distance);
                }
                if (advancedData.enable_obstacle_avoidance !== undefined) {
                    document.getElementById('enable-obstacle-avoidance').checked = advancedData.enable_obstacle_avoidance;
                }
                if (advancedData.waypoint_tolerance !== undefined && advancedData.waypoint_tolerance !== null) {
                    document.getElementById('waypoint-tolerance').value = parseFloat(advancedData.waypoint_tolerance);
                }

                // Security settings laden
                const securityResponse = await fetch('/api/events/settings/security');
                const securityData = await securityResponse.json();

                const emailCfg = securityData.email || {};
                document.getElementById('security-email-enabled').checked = !!emailCfg.enabled;
                document.getElementById('security-smtp-host').value = emailCfg.smtp_host || '';
                document.getElementById('security-smtp-port').value = emailCfg.smtp_port || 587;
                document.getElementById('security-use-tls').checked = emailCfg.use_tls !== false;
                document.getElementById('security-username').value = emailCfg.username || '';
                document.getElementById('security-sender').value = emailCfg.sender || '';
                if (Array.isArray(emailCfg.recipients)) {
                    document.getElementById('security-recipients').value = emailCfg.recipients.join(', ');
                }

                const eventDefaults = securityData.event_defaults || {};
                if (eventDefaults.pre_event_seconds !== undefined) {
                    document.getElementById('security-pre-seconds').value = eventDefaults.pre_event_seconds;
                }
                if (eventDefaults.post_event_seconds !== undefined) {
                    document.getElementById('security-post-seconds').value = eventDefaults.post_event_seconds;
                }
            } catch (error) {
                console.error('Error loading settings:', error);
            }
        }

        function setCurrentAsCharge() {
            if (!currentPosition || currentPosition.latitude === 0) {
                alert('❌ Keine gültige GPS-Position verfügbar! Bitte warten bis GPS-Signal empfangen wurde.');
                return;
            }
            
            // In temp speichern
            tempChargePoint = {
                latitude: currentPosition.latitude,
                longitude: currentPosition.longitude
            };
            
            // Display aktualisieren
            const chargeDisplay = document.getElementById('charge-point-display');
            chargeDisplay.textContent = 
                `${tempChargePoint.latitude.toFixed(6)}, ${tempChargePoint.longitude.toFixed(6)}`;
            chargeDisplay.classList.remove('empty');
            
            // Kurzes Feedback
            const card = document.getElementById('current-position-card');
            card.style.borderLeftColor = '#4CAF50';
            setTimeout(() => {
                card.style.borderLeftColor = 'var(--accent-green)';
            }, 1000);
        }

        async function saveHomePosition() {
            try {
                // Prüfe ob Charge Point gesetzt wurde
                if (!tempChargePoint) {
                    alert('❌ Bitte erst "Aktuelle Position als Ladestation" drücken!');
                    return;
                }
                
                const lat = tempChargePoint.latitude;
                const lon = tempChargePoint.longitude;
                
                // Fusion-Status prüfen (2,3,4 = grün)
                const fusionCode = currentFusionStatus && currentFusionStatus.fusion_status;
                if (![2, 3, 4].includes(fusionCode)) {
                    alert(`❌ Position kann nur bei grünem Fusion-Status gespeichert werden.\nAktueller Status: ${fusionCode || 'unbekannt'}`);
                    return;
                }
                
                // Modal mit Warnung anzeigen
                document.getElementById('new-charge-point-display').textContent = 
                    `${lat.toFixed(6)}, ${lon.toFixed(6)}`;
                document.getElementById('home-position-modal').classList.add('show');
            } catch (error) {
                console.error('Error preparing home position save:', error);
                alert('❌ Fehler: ' + error.message);
            }
        }

        function closeHomePositionModal() {
            document.getElementById('home-position-modal').classList.remove('show');
        }

        function closeHomePositionModalOnOverlay(event) {
            if (event.target.id === 'home-position-modal') {
                closeHomePositionModal();
            }
        }

        async function confirmSaveHomePosition() {
            try {
                const lat = tempChargePoint.latitude;
                const lon = tempChargePoint.longitude;
                
                console.log('Confirming home position change - deleting schedules and routes first');
                
                // Zuerst alle Schedules und Routen löschen
                const deleteResponse = await fetch('/api/settings/home-position', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ 
                        latitude: lat, 
                        longitude: lon,
                        delete_schedules_and_routes: true
                    })
                });
                
                const data = await deleteResponse.json();
                console.log('Server response:', data);
                
                if (data.status === 'success') {
                    // Modal schließen
                    closeHomePositionModal();
                    
                    // Home Point Display aktualisieren
                    const homeDisplay = document.getElementById('home-point-display');
                    homeDisplay.textContent = 
                        `${data.home_point.latitude.toFixed(6)}, ${data.home_point.longitude.toFixed(6)}`;
                    homeDisplay.classList.remove('empty');
                    
                    showSuccessMessage('home-success');
                    
                    // Info über gelöschte Elemente anzeigen
                    if (data.deleted_schedules > 0 || data.deleted_routes > 0) {
                        alert(`✅ Ladestation erfolgreich gespeichert!\n\nGelöscht:\n• ${data.deleted_schedules} Zeitplan/Zeitpläne\n• ${data.deleted_routes} Route(n)`);
                    }
                } else {
                    alert('❌ Fehler: ' + data.message);
                }
            } catch (error) {
                console.error('Error saving position:', error);
                alert('❌ Verbindungsfehler: ' + error.message);
            }
        }

        async function saveBatterySettings() {
            const threshold = parseInt(document.getElementById('battery-threshold').value);
            
            const response = await fetch('/api/settings/battery', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ threshold: threshold })
            });
            
            const data = await response.json();
            if (data.status === 'success') {
                showSuccessMessage('battery-success');
            } else {
                alert('❌ Fehler: ' + data.message);
            }
        }

        async function saveAdvancedSettings() {
            const speedFactor = parseFloat(document.getElementById('speed-factor').value);
            const autoChargeReturn = document.getElementById('auto-charge-return').checked;
            const maxRouteDistanceInput = document.getElementById('max-route-distance').value;
            const maxRouteDistance = parseFloat(maxRouteDistanceInput);
            
            // Validate max_route_distance is a valid number
            if (isNaN(maxRouteDistance) || maxRouteDistance < 1.0 || maxRouteDistance > 100.0) {
                alert('❌ Fehler: Maximale Distanz zur Route muss eine Zahl zwischen 1.0 und 100.0 sein.');
                return;
            }
            
            const waypointToleranceInput = document.getElementById('waypoint-tolerance').value;
            const waypointTolerance = parseFloat(waypointToleranceInput);
            
            // Validate waypoint_tolerance is a valid number
            if (isNaN(waypointTolerance) || waypointTolerance < 0.1 || waypointTolerance > 5.0) {
                alert('❌ Fehler: Waypoint erreicht Schwellenwert muss eine Zahl zwischen 0.1 und 5.0 sein.');
                return;
            }
            
            const enableObstacleAvoidance = document.getElementById('enable-obstacle-avoidance').checked;
            
            const response = await fetch('/api/settings/advanced', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    speed_factor: speedFactor,
                    auto_charge_return: autoChargeReturn,
                    max_route_distance: maxRouteDistance,
                    waypoint_tolerance: waypointTolerance,
                    enable_obstacle_avoidance: enableObstacleAvoidance
                })
            });
            
            const data = await response.json();
            if (data.status === 'success') {
                showSuccessMessage('advanced-success');
            } else {
                alert('❌ Fehler: ' + data.message);
            }
        }

        async function saveSecuritySettings() {
            try {
                const enabled = document.getElementById('security-email-enabled').checked;
                const smtpHost = document.getElementById('security-smtp-host').value;
                const smtpPort = parseInt(document.getElementById('security-smtp-port').value) || 587;
                const useTls = document.getElementById('security-use-tls').checked;
                const username = document.getElementById('security-username').value;
                const password = document.getElementById('security-password').value;
                const sender = document.getElementById('security-sender').value;
                const recipientsStr = document.getElementById('security-recipients').value;
                const recipients = recipientsStr.split(',').map(r => r.trim()).filter(r => r.length > 0);

                const preSeconds = parseFloat(document.getElementById('security-pre-seconds').value) || 10;
                const postSeconds = parseFloat(document.getElementById('security-post-seconds').value) || 10;

                const payload = {
                    email: {
                        enabled,
                        smtp_host: smtpHost,
                        smtp_port: smtpPort,
                        use_tls: useTls,
                        username,
                        password,
                        sender,
                        recipients
                    },
                    event_defaults: {
                        pre_event_seconds: preSeconds,
                        post_event_seconds: postSeconds
                    }
                };

                const response = await fetch('/api/events/settings/security', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(payload)
                });

                if (!response.ok) {
                    const errorData = await response.json().catch(() => ({ detail: 'Unknown error' }));
                    throw new Error(errorData.detail || `HTTP ${response.status}`);
                }

                const data = await response.json();
                if (data.status === 'success') {
                    showSuccessMessage('security-success');
                    // Clear password field so it is not kept in DOM
                    document.getElementById('security-password').value = '';
                    // Reload settings to show updated values
                    await loadSettings();
                } else {
                    throw new Error(data.message || 'Unknown error');
                }
            } catch (error) {
                console.error('Error saving security settings:', error);
                alert('❌ Fehler beim Speichern der Sicherheits-Einstellungen: ' + error.message);
            }
        }

        function showSuccessMessage(elementId) {
            const el = document.getElementById(elementId);
            el.classList.add('show');
            setTimeout(() => {
                el.classList.remove('show');
            }, 3000);
        }

        function updateFusionBadges(state) {
            const imu = Number(state.imu_status);
            const gnss1 = Number(state.gnss1_status);
            const gnss2 = Number(state.gnss2_status);
            const fusion = Number(state.fusion_status);

            // IMU-Farbe: Grün ab warmstarted (1), rot nur bei echten Fehlern (0, null, undefined)
            let imuColor = 'red';
            if (imu >= 1) {  // IMU_STATUS_WARMSTARTED (1), CONVERGING (2), CONVERGED (3)
                imuColor = 'green';
            }

            let gnss1Color = 'red';
            if (gnss1 === 5) {
                gnss1Color = 'orange';
            } else if (gnss1 === 8) {
                gnss1Color = 'green';
            } else if (gnss1 === 1) {
                gnss1Color = 'orange';
            }

            let gnss2Color = 'red';
            if (gnss2 === 5) {
                gnss2Color = 'orange';
            } else if (gnss2 === 8) {
                gnss2Color = 'green';
            } else if (gnss2 === 1) {
                gnss2Color = 'orange';
            }

            let fusionColor = 'red';
            if (fusion === 2 || fusion === 3 || fusion === 4) {
                fusionColor = 'green';
            }

            setBadgeColor('badge-imu', imuColor);
            setBadgeColor('badge-gnss1', gnss1Color); 
            setBadgeColor('badge-gnss2', gnss2Color);  
            setBadgeColor('badge-fusion', fusionColor);
        }

        function setBadgeColor(id, color) {
            const el = document.getElementById(id);
            if (!el) return;

            el.classList.remove('red', 'orange', 'green');

            if (color !== 'orange' && color !== 'green') {
                color = 'red';
            }
            el.classList.add(color);
        }

        // Restore Modal Functions
        function openRestoreModal() {
            document.getElementById('restore-modal').classList.add('show');
            resetRestoreModal();
        }

        function closeRestoreModal() {
            document.getElementById('restore-modal').classList.remove('show');
            resetRestoreModal();
        }

        function closeRestoreModalOnOverlay(event) {
            if (event.target.id === 'restore-modal') {
                closeRestoreModal();
            }
        }

        function resetRestoreModal() {
            // Reset checkboxes
            document.getElementById('restore-home-position').checked = false;
            document.getElementById('restore-schedules').checked = false;
            document.getElementById('restore-routes').checked = false;
            document.getElementById('restore-advanced').checked = false;
            document.getElementById('restore-email').checked = false;
            document.getElementById('restore-alarm').checked = false;
            
            // Reset disabled states
            document.getElementById('restore-schedules').disabled = false;
            document.getElementById('restore-routes').disabled = false;
            
            // Hide dependency notes
            document.getElementById('restore-home-note').style.display = 'none';
            document.getElementById('restore-schedules-note').style.display = 'none';
            document.getElementById('restore-routes-note').style.display = 'none';
            
            // Reset steps
            document.getElementById('restore-step-1').classList.add('active');
            document.getElementById('restore-step-2').classList.remove('active');
            document.getElementById('restore-confirmation-input').value = '';
            document.getElementById('restore-proceed-btn').disabled = true;
            document.getElementById('restore-confirm-btn').disabled = true;
        }

        function updateRestoreDependencies() {
            const homeChecked = document.getElementById('restore-home-position').checked;
            const routesChecked = document.getElementById('restore-routes').checked;
            const schedulesCheckbox = document.getElementById('restore-schedules');
            const routesCheckbox = document.getElementById('restore-routes');
            const schedulesNote = document.getElementById('restore-schedules-note');
            const homeNote = document.getElementById('restore-home-note');
            const routesNote = document.getElementById('restore-routes-note');
            
            // Priority: Home position has highest priority
            if (homeChecked) {
                // Home position checked: force schedules and routes to be checked and disabled
                schedulesCheckbox.checked = true;
                schedulesCheckbox.disabled = true;
                routesCheckbox.checked = true;
                routesCheckbox.disabled = true;
                homeNote.style.display = 'block';
                schedulesNote.style.display = 'block';
                routesNote.style.display = 'none';
            } else if (routesChecked) {
                // Routes checked (but not home): force schedules to be checked and disabled
                schedulesCheckbox.checked = true;
                schedulesCheckbox.disabled = true;
                routesCheckbox.disabled = false;
                schedulesNote.style.display = 'block';
                routesNote.style.display = 'block';
                homeNote.style.display = 'none';
            } else {
                // Neither home nor routes checked: allow independent control
                schedulesCheckbox.disabled = false;
                routesCheckbox.disabled = false;
                if (!schedulesCheckbox.checked) {
                    schedulesNote.style.display = 'none';
                }
                routesNote.style.display = 'none';
                homeNote.style.display = 'none';
            }
            
            // Enable/disable proceed button based on selections
            const hasSelection = 
                document.getElementById('restore-home-position').checked ||
                document.getElementById('restore-schedules').checked ||
                document.getElementById('restore-routes').checked ||
                document.getElementById('restore-advanced').checked ||
                document.getElementById('restore-email').checked ||
                document.getElementById('restore-alarm').checked;
            
            document.getElementById('restore-proceed-btn').disabled = !hasSelection;
        }

        function proceedToConfirmation() {
            document.getElementById('restore-step-1').classList.remove('active');
            document.getElementById('restore-step-2').classList.add('active');
            document.getElementById('restore-confirmation-input').focus();
        }

        function backToSelection() {
            document.getElementById('restore-step-2').classList.remove('active');
            document.getElementById('restore-step-1').classList.add('active');
            document.getElementById('restore-confirmation-input').value = '';
            document.getElementById('restore-confirm-btn').disabled = true;
        }

        function checkConfirmationInput() {
            const input = document.getElementById('restore-confirmation-input').value.trim();
            const confirmBtn = document.getElementById('restore-confirm-btn');
            confirmBtn.disabled = input !== 'LÖSCHEN';
        }

        async function confirmRestore() {
            const restoreData = {
                home_position: document.getElementById('restore-home-position').checked,
                schedules: document.getElementById('restore-schedules').checked,
                routes: document.getElementById('restore-routes').checked,
                advanced: document.getElementById('restore-advanced').checked,
                email: document.getElementById('restore-email').checked,
                alarm: document.getElementById('restore-alarm').checked
            };
            
            try {
                const response = await fetch('/api/settings/restore', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(restoreData)
                });
                
                const data = await response.json();
                
                if (data.status === 'success' || data.status === 'partial') {
                    alert('✅ ' + data.message);
                    if (data.errors && data.errors.length > 0) {
                        alert('⚠️ Fehler:\n' + data.errors.join('\n'));
                    }
                    closeRestoreModal();
                    // Reload settings to reflect changes
                    await loadSettings();
                    // Reload page to show updated state
                    window.location.reload();
                } else {
                    alert('❌ Fehler: ' + data.message);
                }
            } catch (error) {
                console.error('Error restoring settings:', error);
                alert('❌ Verbindungsfehler: ' + error.message);
            }
        }

        // Init
        loadSettings();

        // Cleanup intervals when leaving the page
        window.addEventListener('beforeunload', () => {
            intervals.forEach(id => clearInterval(id));
        });

        // Fallback for mobile browsers
        window.addEventListener('pagehide', () => {
            intervals.forEach(id => clearInterval(id));
        });
