        let selectedWeekdays = [];
        let selectedRoutes = [];
        let currentRouteMode = 'sequential';
        let editingScheduleIndex = null;
        let draggedElement = null;
        let schedulerBusy = false;   // NEU: blockiert Buttons während Request
        let fusionRawState = {
            fusion_status: null,
            imu_status: null,
            gnss_status: null,
            rtk_status: null
        };
        // Routen laden
        async function loadRoutes() {
            const response = await fetch('/api/routes');
            const data = await response.json();
            
            const select = document.getElementById('route-selector');
            select.innerHTML = '<option value="">-- Route auswählen --</option>';
            
            data.routes.forEach(route => {
                const option = document.createElement('option');
                option.value = route;
                option.textContent = route;
                select.appendChild(option);
            });
        }

        // Route zum Zeitplan hinzufügen
        function addRouteToSchedule() {
            const routeName = document.getElementById('route-selector').value;
            const repeatCount = parseInt(document.getElementById('repeat-count').value);
            
            if (!routeName) {
                alert('Bitte Route auswählen!');
                return;
            }
            
            if (repeatCount < 1) {
                alert('Wiederholungen müssen mindestens 1 sein!');
                return;
            }
            
            // Prüfen ob Route bereits hinzugefügt
            if (selectedRoutes.some(r => r.route_name === routeName)) {
                alert('Route bereits hinzugefügt!');
                return;
            }
            
            selectedRoutes.push({
                route_name: routeName,
                repeat_count: repeatCount
            });
            
            updateSelectedRoutesList();
            
            // Reset
            document.getElementById('route-selector').value = '';
            document.getElementById('repeat-count').value = 1;
        }

        function setSchedulerUiBusy(isBusy) {
            schedulerBusy = isBusy;

            // Alle Buttons im Scheduler sperren
            document.querySelectorAll('.btn-save, .btn-cancel, .btn-edit, .btn-delete, .btn-add-route')
                .forEach(btn => {
                    btn.disabled = isBusy;
                });

            // Status-Badges (AKTIV/INAKTIV) sperren
            document.querySelectorAll('.status-badge').forEach(badge => {
                if (isBusy) {
                    badge.classList.add('disabled');
                } else {
                    badge.classList.remove('disabled');
                }
            });
        }

        // Selected Routes Liste aktualisieren
        function updateSelectedRoutesList() {
            const container = document.getElementById('selected-routes');
            
            if (selectedRoutes.length === 0) {
                container.className = 'selected-routes-list empty';
                container.innerHTML = 'Keine Routen ausgewählt';
                return;
            }
            
            container.className = 'selected-routes-list';
            container.innerHTML = '';
            
            selectedRoutes.forEach((route, idx) => {
                const item = document.createElement('div');
                item.className = 'route-item';
                item.draggable = true;
                item.dataset.index = idx;
                
                item.innerHTML = `
                    <span class="drag-handle">☰</span>
                    <div class="route-item-info">
                        <div class="route-item-name">${route.route_name}</div>
                        <div class="route-item-repeat">${route.repeat_count}x wiederholen</div>
                    </div>
                    <button class="btn-remove-route" onclick="removeRouteFromSchedule(${idx})">🗑️</button>
                `;
                
                // Drag & Drop Event Listeners
                item.addEventListener('dragstart', handleDragStart);
                item.addEventListener('dragover', handleDragOver);
                item.addEventListener('drop', handleDrop);
                item.addEventListener('dragend', handleDragEnd);
                
                container.appendChild(item);
            });
        }

        // Drag & Drop Handlers
        function handleDragStart(e) {
            draggedElement = this;
            this.classList.add('dragging');
            e.dataTransfer.effectAllowed = 'move';
            e.dataTransfer.setData('text/html', this.innerHTML);
        }

        function handleDragOver(e) {
            if (e.preventDefault) {
                e.preventDefault();
            }
            e.dataTransfer.dropEffect = 'move';
            
            const afterElement = getDragAfterElement(e.currentTarget.parentElement, e.clientY);
            const dragging = document.querySelector('.dragging');
            
            if (afterElement == null) {
                e.currentTarget.parentElement.appendChild(dragging);
            } else {
                e.currentTarget.parentElement.insertBefore(dragging, afterElement);
            }
            
            return false;
        }

        function handleDrop(e) {
            if (e.stopPropagation) {
                e.stopPropagation();
            }
            
            // Reihenfolge im Array aktualisieren
            const items = Array.from(document.querySelectorAll('.route-item'));
            const newOrder = items.map(item => parseInt(item.dataset.index));
            
            const reorderedRoutes = newOrder.map(idx => selectedRoutes[idx]);
            selectedRoutes = reorderedRoutes;
            
            updateSelectedRoutesList();
            
            return false;
        }

        function handleDragEnd(e) {
            this.classList.remove('dragging');
        }

        function getDragAfterElement(container, y) {
            const draggableElements = [...container.querySelectorAll('.route-item:not(.dragging)')];
            
            return draggableElements.reduce((closest, child) => {
                const box = child.getBoundingClientRect();
                const offset = y - box.top - box.height / 2;
                
                if (offset < 0 && offset > closest.offset) {
                    return { offset: offset, element: child };
                } else {
                    return closest;
                }
            }, { offset: Number.NEGATIVE_INFINITY }).element;
        }

        // Route aus Zeitplan entfernen
        function removeRouteFromSchedule(idx) {
            selectedRoutes.splice(idx, 1);
            updateSelectedRoutesList();
        }

        // Route Mode wählen
        function selectRouteMode(mode) {
            currentRouteMode = mode;
            
            document.querySelectorAll('.mode-btn').forEach(btn => {
                btn.classList.remove('active');
            });
            
            document.querySelector(`[data-mode="${mode}"]`).classList.add('active');
        }

        // Wochentag Toggle
        function toggleWeekday(day) {
            const btn = document.querySelector(`[data-day="${day}"]`);
            btn.classList.toggle('active');
            
            const idx = selectedWeekdays.indexOf(day);
            if (idx > -1) {
                selectedWeekdays.splice(idx, 1);
            } else {
                selectedWeekdays.push(day);
            }
        }

        // Zeitplan speichern
       async function saveSchedule() {
            if (schedulerBusy) return;  // Mehrfachklick verhindern

            const startTime = document.getElementById('start-time').value;
            const endTime = document.getElementById('end-time').value;
            const loop = document.getElementById('schedule-loop').checked;
            const active = document.getElementById('schedule-active').checked;
                    
            if (selectedRoutes.length === 0) {
                alert('Bitte mindestens eine Route hinzufügen!');
                return;
            }
                    
            if (selectedWeekdays.length === 0) {
                alert('Bitte mindestens einen Wochentag auswählen!');
                return;
            }

            const scheduleData = {
                routes: selectedRoutes,
                route_mode: currentRouteMode,
                weekdays: selectedWeekdays,
                start_time: startTime,
                end_time: endTime,
                loop: loop,
                require_home_return: true,  // Always true
                active: active
            };

            setSchedulerUiBusy(true);

            try {
                let response;
                if (editingScheduleIndex !== null) {
                    response = await fetch(`/api/schedule/edit/${editingScheduleIndex}`, {
                        method: 'PUT',
                        headers: { 'Content-Type': 'application/json' },
                        body: JSON.stringify(scheduleData)
                    });
                } else {
                    response = await fetch('/api/schedule/save', {
                        method: 'POST',
                        headers: { 'Content-Type': 'application/json' },
                        body: JSON.stringify(scheduleData)
                    });
                }
                    
                const data = await response.json();

                // Backend antwortet nur "success", wenn ACK vom Roboter kam
                if (data.status === 'success') {
                    alert('✅ Zeitplan gespeichert!');
                    resetForm();
                    loadSchedules();
                } else {
                    alert('❌ Fehler: ' + data.message);
                }
            } catch (e) {
                alert('❌ Fehler: ' + e.message);
            } finally {
                setSchedulerUiBusy(false);
            }
        }


        // Zeitpläne laden
        async function loadSchedules() {
            // Add cache-busting parameter to ensure fresh data
            const response = await fetch(`/api/schedule/schedules?t=${Date.now()}`);
            const data = await response.json();
            
            const listDiv = document.getElementById('schedule-list');
            
            if (data.schedules.length === 0) {
                listDiv.innerHTML = '<p style="color: #888; text-align: center; padding: 20px;">Keine Zeitpläne vorhanden</p>';
                return;
            }
            
            const weekdayNames = ['Mo', 'Di', 'Mi', 'Do', 'Fr', 'Sa', 'So'];
            
            listDiv.innerHTML = '';
            data.schedules.forEach((schedule, idx) => {
                const days = schedule.weekdays.map(d => weekdayNames[d]).join(', ');
                const modeIcon = schedule.route_mode === 'random' ? '🎲' : '🔢';
                const routesList = schedule.routes.map(r => 
                    `${r.route_name} (${r.repeat_count}x)`
                ).join(', ');
                
                // Ensure loop field is properly checked (handle both 'loop' and 'loop_mode' for backward compatibility)
                // Check both field names since YAML might have either
                const loopField = schedule.loop !== undefined ? schedule.loop : schedule.loop_mode;
                const loopValue = loopField === true || loopField === 'true' || loopField === 1 || loopField === '1';
                
                const item = document.createElement('div');
                item.className = 'schedule-item';
                item.innerHTML = `
                    <div class="schedule-info">
                        <h4>
                            ${modeIcon} ${schedule.routes.length} Route(n)
                            <span class="status-badge ${schedule.active ? 'status-active' : 'status-inactive'}" 
                                  onclick="toggleSchedule(${idx})">
                                ${schedule.active ? 'AKTIV' : 'INAKTIV'}
                            </span>
                        </h4>
                        <div class="route-details">
                            <div class="route-details-item">📍 Routen: ${routesList}</div>
                            <div class="route-details-item">📅 ${days}</div>
                            <div class="route-details-item">🕒 ${schedule.start_time} - ${schedule.end_time}</div>
                            <div class="route-details-item">🔄 Wiederholen: ${loopValue ? 'Ja' : 'Nein'}</div>
                        </div>
                    </div>
                    <div class="schedule-actions">
                        <button class="btn-small btn-edit" onclick="editSchedule(${idx})">✏️ Bearbeiten</button>
                        <button class="btn-small btn-delete" onclick="deleteSchedule(${idx})">🗑️ Löschen</button>
                    </div>
                `;
                listDiv.appendChild(item);
            });
        }

        // Zeitplan bearbeiten
        async function editSchedule(idx) {
            const response = await fetch('/api/schedule/schedules');
            const data = await response.json();
            const schedule = data.schedules[idx];
            
            editingScheduleIndex = idx;
            
            // Form füllen
            selectedRoutes = [...schedule.routes];
            updateSelectedRoutesList();
            
            selectRouteMode(schedule.route_mode);
            
            selectedWeekdays = [...schedule.weekdays];
            document.querySelectorAll('.weekday-btn').forEach(btn => {
                const day = parseInt(btn.dataset.day);
                if (selectedWeekdays.includes(day)) {
                    btn.classList.add('active');
                } else {
                    btn.classList.remove('active');
                }
            });
            
            document.getElementById('start-time').value = schedule.start_time;
            document.getElementById('end-time').value = schedule.end_time;
            // Handle both 'loop' and 'loop_mode' field names for backward compatibility
            const loopField = schedule.loop !== undefined ? schedule.loop : schedule.loop_mode;
            document.getElementById('schedule-loop').checked = loopField || false;
            document.getElementById('schedule-active').checked = schedule.active;
            
            // Scroll to top
            window.scrollTo({ top: 0, behavior: 'smooth' });
            
            alert('📝 Zeitplan wird bearbeitet. Änderungen speichern um zu aktualisieren.');
        }

        // Zeitplan Status togglen
        async function toggleSchedule(idx) {
            if (schedulerBusy) return;

            setSchedulerUiBusy(true);

            try {
                const response = await fetch(`/api/schedule/toggle/${idx}`, { method: 'POST' });
                const data = await response.json();
                    
                if (data.status === 'success') {
                    loadSchedules();
                } else {
                    alert('❌ Fehler: ' + data.message);
                }
            } catch (e) {
                alert('❌ Fehler: ' + e.message);
            } finally {
                setSchedulerUiBusy(false);
            }
        }


        // Zeitplan löschen
        async function deleteSchedule(idx) {
            if (schedulerBusy) return;
            if (!confirm('Zeitplan wirklich löschen?')) return;

            setSchedulerUiBusy(true);

            try {
                const response = await fetch(`/api/schedule/delete/${idx}`, { method: 'DELETE' });
                const data = await response.json();
                    
                if (data.status === 'success') {
                    loadSchedules();
                } else {
                    alert('❌ Fehler: ' + data.message);
                }
            } catch (e) {
                alert('❌ Fehler: ' + e.message);
            } finally {
                setSchedulerUiBusy(false);
            }
        }


        // Formular zurücksetzen
        function resetForm() {
            selectedRoutes = [];
            updateSelectedRoutesList();
            
            currentRouteMode = 'sequential';
            selectRouteMode('sequential');
            
            selectedWeekdays = [];
            document.querySelectorAll('.weekday-btn').forEach(btn => {
                btn.classList.remove('active');
            });
            
            document.getElementById('route-selector').value = '';
            document.getElementById('repeat-count').value = 1;
            document.getElementById('start-time').value = '08:00';
            document.getElementById('end-time').value = '17:00';
            document.getElementById('schedule-loop').checked = false;
            document.getElementById('schedule-active').checked = true;
            
            editingScheduleIndex = null;
        }

        // Log management - save to localStorage (logs are only displayed on index.html)
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
                const messageKey = entry.message + '|' + (entry.isError ? 'error' : 'info');
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

        // WebSocket für Robot State (Connection Status)
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

        // Init
        loadRoutes();
        loadSchedules();

        // Cleanup intervals when leaving the page
        window.addEventListener('beforeunload', () => {
            intervals.forEach(id => clearInterval(id));
        });

        // Fallback for mobile browsers
        window.addEventListener('pagehide', () => {
            intervals.forEach(id => clearInterval(id));
        });
