// =====================================================
// Version Check für automatisches Cache-Busting
// =====================================================
class VersionChecker {
    constructor() {
        this.localVersionKey = 'robot_app_version';
        this.checkInterval = 60000; // Prüfe alle 60 Sekunden
        this.intervalId = null;
    }

    async checkVersion() {
        try {
            const response = await fetch('/api/version', { 
                cache: 'no-store',  // Niemals cachen
                headers: { 'Cache-Control': 'no-cache' }
            });
            
            if (!response.ok) {
                console.warn('Version check failed:', response.status);
                return;
            }
            
            const data = await response.json();
            const serverVersion = data.version;
            const localVersion = localStorage.getItem(this.localVersionKey);
            
            if (localVersion && localVersion !== serverVersion) {
                console.log(`Neue App-Version erkannt: ${localVersion} → ${serverVersion}`);
                // Speichere neue Version vor dem Reload
                localStorage.setItem(this.localVersionKey, serverVersion);
                // Hard Reload: Cache ignorieren und alles neu laden
                this.forceReload();
            } else if (!localVersion) {
                // Erste Nutzung - Version speichern
                localStorage.setItem(this.localVersionKey, serverVersion);
                console.log('App-Version initialisiert:', serverVersion);
            }
        } catch (error) {
            console.warn('Version check error:', error);
        }
    }

    forceReload() {
        // Versuche verschiedene Methoden für maximale Kompatibilität
        if ('caches' in window) {
            // Service Worker Cache API verfügbar - lösche alle Caches
            caches.keys().then(names => {
                for (const name of names) {
                    caches.delete(name);
                }
            }).then(() => {
                this.performReload();
            });
        } else {
            this.performReload();
        }
    }

    performReload() {
        // location.reload(true) ist deprecated, aber wir nutzen alternative Methode
        // Füge timestamp Query-Parameter hinzu um Cache zu umgehen
        const url = new URL(window.location.href);
        url.searchParams.set('_v', Date.now());
        window.location.replace(url.toString());
    }

    start() {
        // Sofort beim Start prüfen
        this.checkVersion();
        
        // Regelmäßig prüfen
        this.intervalId = setInterval(() => {
            this.checkVersion();
        }, this.checkInterval);
    }

    stop() {
        if (this.intervalId) {
            clearInterval(this.intervalId);
            this.intervalId = null;
        }
    }
}

// =====================================================
// WebSocket Manager für Status-Updates
// =====================================================
class StatusManager {
    constructor() {
        this.wsPosition = null;
        this.reconnectTimeout = null;
        this.reconnectDelay = 1000; // Start with 1 second
        this.maxReconnectDelay = 30000; // Max 30 seconds
        this.initWebSocket();
    }

    cleanupWebSocket() {
        if (this.wsPosition) {
            this.wsPosition.onopen = null;
            this.wsPosition.onmessage = null;
            this.wsPosition.onerror = null;
            this.wsPosition.onclose = null;
            if (this.wsPosition.readyState === WebSocket.OPEN || 
                this.wsPosition.readyState === WebSocket.CONNECTING) {
                this.wsPosition.close();
            }
            this.wsPosition = null;
        }
        if (this.reconnectTimeout) {
            clearTimeout(this.reconnectTimeout);
            this.reconnectTimeout = null;
        }
    }

    initWebSocket() {
        // Cleanup existing connection before creating new one
        this.cleanupWebSocket();
        
        this.wsPosition = new WebSocket(`ws://${window.location.host}/ws/position`);
        
        this.wsPosition.onopen = () => {
            // Reset reconnect delay on successful connection
            this.reconnectDelay = 1000;
        };
        
        this.wsPosition.onmessage = (event) => {
            const data = JSON.parse(event.data);
            if (data.type === 'fusion_status') {
                this.updateFusionBadges(data.data);
            } else if (data.type === 'event') {
                // Save logs to localStorage on all pages
                this.addLogEntry(data.data);
            }
        };

        this.wsPosition.onerror = () => {
            console.error('WebSocket error');
            this.scheduleReconnect();
        };

        this.wsPosition.onclose = () => {
            this.scheduleReconnect();
        };
    }

    scheduleReconnect() {
        // Exponential backoff: double the delay each time, up to max
        const delay = this.reconnectDelay;
        this.reconnectDelay = Math.min(this.reconnectDelay * 2, this.maxReconnectDelay);
        
        this.reconnectTimeout = setTimeout(() => {
            this.initWebSocket();
        }, delay);
    }

    destroy() {
        this.cleanupWebSocket();
    }

    updateFusionBadges(state) {
        const imu = Number(state.imu_status);
        const gnss1 = Number(state.gnss1_status);
        const gnss2 = Number(state.gnss2_status);
        const fusion = Number(state.fusion_status);

        // IMU-Farbe: Grün ab warmstarted (1), rot nur bei echten Fehlern (0, null, undefined)
        let imuColor = 'red';
        if (imu >= 1) imuColor = 'green';  // IMU_STATUS_WARMSTARTED (1), CONVERGING (2), CONVERGED (3)

        // GNSS1-Farbe
        let gnss1Color = 'red';
        if (gnss1 === 5) gnss1Color = 'orange';
        else if (gnss1 === 8) gnss1Color = 'green';
        else if (gnss1 === 1) gnss1Color = 'orange';

        // GNSS2-Farbe
        let gnss2Color = 'red';
        if (gnss2 === 5) gnss2Color = 'orange';
        else if (gnss2 === 8) gnss2Color = 'green';
        else if (gnss2 === 1) gnss2Color = 'orange';

        // Fusion-Farbe
        let fusionColor = 'red';
        if (fusion === 2) fusionColor = 'green';
        else if (fusion === 1) fusionColor = 'orange';

        this.setBadgeColor('badge-imu', imuColor);
        this.setBadgeColor('badge-gnss1', gnss1Color);
        this.setBadgeColor('badge-gnss2', gnss2Color);
        this.setBadgeColor('badge-fusion', fusionColor);

        // UI sprint #2 — also drive the single summary dot.
        // The 4 old badges are now hidden by default (CSS) and only shown
        // when the operator taps the summary dot to see the detail popover.
        this.updateSummaryStatus({ imu, gnss1, gnss2, fusion });
    }

    /* ─── UI sprint #2: single summary status dot ─────────────────────
     * Replaces the 4 cryptic nav badges (GNSS1/GNSS2/IMU/Fusion) with
     * one big colored indicator + plain German label. Tap → popover
     * shows the old 4 badges (for technicians / debug).
     * See PLAN_UI_CUSTOMER_HANDOFF.md §7 item #2.
     */
    installSummaryDot() {
        const group = document.querySelector('.nav-status-group');
        if (!group || document.getElementById('summary-status-dot')) return;

        const dot = document.createElement('button');
        dot.id = 'summary-status-dot';
        dot.type = 'button';
        dot.className = 'nav-summary-dot red';
        dot.setAttribute('aria-label', 'Robotersystem-Status anzeigen');
        dot.setAttribute('aria-expanded', 'false');
        dot.innerHTML = `
            <span class="summary-dot-light" aria-hidden="true"></span>
            <span class="summary-dot-label">Status wird geladen…</span>
        `;
        // Insert the dot as the FIRST child so it leads the nav-status-group.
        // Old badges live behind it and are revealed only via .show-detail.
        group.insertBefore(dot, group.firstChild);

        dot.addEventListener('click', () => {
            const expanded = group.classList.toggle('show-detail');
            dot.setAttribute('aria-expanded', expanded ? 'true' : 'false');
        });

        // Click outside the group closes the popover
        document.addEventListener('click', (e) => {
            if (!group.contains(e.target) && group.classList.contains('show-detail')) {
                group.classList.remove('show-detail');
                dot.setAttribute('aria-expanded', 'false');
            }
        });
    }

    updateSummaryStatus({ imu, gnss1, gnss2, fusion }) {
        const dot = document.getElementById('summary-status-dot');
        if (!dot) return;

        // Compute single state — see DESIGN PLAN §7 item #2 for the rules.
        // 🟢 Alles bereit: both GNSS at RTK_FIXED (8) AND fusion globally initialised (2)
        // 🟡 GPS schwach: both GNSS at least RTK_FLOAT (5) (or 1 = SPP — accept as caveat)
        //                 OR fusion only locally initialised (1)
        // 🔴 GPS verloren: anything worse
        const gnssOK = (g) => g === 8;
        const gnssWeak = (g) => g === 8 || g === 5 || g === 1;

        let color = 'red';
        let label = 'GPS verloren — Roboter steht still';

        if (gnssOK(gnss1) && gnssOK(gnss2) && fusion === 2) {
            color = 'green';
            label = 'Alles bereit';
        } else if (gnssWeak(gnss1) && gnssWeak(gnss2) && (fusion === 1 || fusion === 2)) {
            color = 'orange';
            label = 'GPS gerade schwach';
        }

        // IMU red is a hardware/sensor problem — override to red regardless
        if (imu !== undefined && imu < 1 && !Number.isNaN(imu)) {
            color = 'red';
            label = 'Sensor-Problem (IMU)';
        }

        dot.classList.remove('red', 'orange', 'green');
        dot.classList.add(color);
        const labelEl = dot.querySelector('.summary-dot-label');
        if (labelEl) labelEl.textContent = label;
    }

    setBadgeColor(id, color) {
        const el = document.getElementById(id);
        if (!el) return;
        el.classList.remove('red', 'orange', 'green');
        if (color !== 'orange' && color !== 'green') {
            color = 'red';
        }
        el.classList.add(color);
    }

    addLogEntry(entry) {
        // On non-index pages, only save to localStorage, don't display
        const MAX_STORED_LOGS = 100;
        
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
}

// Automatisch initialisieren wenn DOM geladen
document.addEventListener('DOMContentLoaded', () => {
    // Version Checker starten (prüft auf neue App-Versionen)
    window.versionChecker = new VersionChecker();
    window.versionChecker.start();

    // Status Manager starten
    window.statusManager = new StatusManager();

    // UI sprint #2 — inject the summary status dot into every page's nav.
    // Safe to call on pages without .nav-status-group (no-op).
    window.statusManager.installSummaryDot();
});

// Cleanup beim Verlassen der Seite
window.addEventListener('beforeunload', () => {
    if (window.versionChecker) {
        window.versionChecker.stop();
    }
    if (window.statusManager) {
        window.statusManager.destroy();
    }
});

// Fallback für mobile Browser
window.addEventListener('pagehide', () => {
    if (window.versionChecker) {
        window.versionChecker.stop();
    }
    if (window.statusManager) {
        window.statusManager.destroy();
    }
});