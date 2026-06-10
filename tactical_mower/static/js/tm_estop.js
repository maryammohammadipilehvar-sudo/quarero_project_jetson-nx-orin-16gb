
// ============================================================================
// EMERGENCY STOP — injected into every page via common.js.
// Calls /api/control/emergency-stop + /api/control/stop + /api/control/move
// in parallel for redundancy. Shows a confirmation banner.
// Keyboard shortcut: Ctrl+. or Cmd+. (Mac).
// ============================================================================
(function () {
    let bannerTimer = null;

    function injectEstop() {
        if (document.getElementById('estop-btn')) return;
        const navbar = document.querySelector('.navbar');
        if (!navbar) return;

        // Button (lives inside the navbar, ordered to the far right via CSS)
        const btn = document.createElement('button');
        btn.id = 'estop-btn';
        btn.className = 'estop-btn';
        btn.type = 'button';
        btn.setAttribute('aria-label', 'Notstop — Motoren sofort stoppen (Strg+Punkt)');
        btn.title = 'Notstop — Motoren sofort stoppen · Strg + .';
        btn.innerHTML =
            '<span class="estop-icon" aria-hidden="true">⛔</span>' +
            '<span class="estop-label">NOTSTOP AUS</span>';
        navbar.appendChild(btn);

        // Banner (lives at the top of the body, hidden until first stop)
        const banner = document.createElement('div');
        banner.id = 'estop-banner';
        banner.className = 'estop-banner';
        banner.setAttribute('role', 'alert');
        banner.setAttribute('aria-live', 'assertive');
        banner.innerHTML =
            '<div class="estop-banner-icon" aria-hidden="true">🛑</div>' +
            '<div class="estop-banner-text">' +
                '<div class="estop-banner-title">Roboter angehalten</div>' +
                '<div class="estop-banner-sub" id="estop-banner-sub">Befehl wird gesendet …</div>' +
            '</div>' +
            '<button class="estop-banner-reset" type="button" aria-label="Notstop aufheben" title="Notstop aufheben">Aufheben</button>' +
            '<button class="estop-banner-close" type="button" aria-label="Schließen" title="Schließen">✕</button>';
        document.body.appendChild(banner);

        btn.addEventListener('click', triggerEstop);
        banner.querySelector('.estop-banner-close').addEventListener('click', hideBanner);
        banner.querySelector('.estop-banner-reset').addEventListener('click', clearEstop);

        // Hotkey: Ctrl+. (period) — universal "abort" shortcut on most platforms
        document.addEventListener('keydown', function (e) {
            if ((e.ctrlKey || e.metaKey) && e.key === '.') {
                e.preventDefault();
                triggerEstop();
            }
        });
    }

    async function triggerEstop() {
        const btn = document.getElementById('estop-btn');
        if (btn) btn.classList.add('triggered');
        showBanner('Befehl wird gesendet …');

        const post = function (path, body) {
            return fetch(path, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(body || {})
            });
        };

        // Send in parallel — first success wins. Redundant on purpose.
        // emergency-stop API expects {state: true} to TRIGGER, {state: false} to clear.
        const calls = [
            post('/api/control/emergency-stop', { state: true }),
            post('/api/control/stop', {}),
            post('/api/control/move', { x: 0, y: 0 })
        ];
        const results = await Promise.allSettled(calls);
        const ok = results.some(function (r) {
            return r.status === 'fulfilled' && r.value && r.value.ok;
        });

        const time = new Date().toLocaleTimeString('de-DE', {
            hour: '2-digit', minute: '2-digit', second: '2-digit'
        });

        if (ok) {
            showBanner('Motoren gestoppt um ' + time);
            if (btn) {
                const lbl = btn.querySelector('.estop-label');
                if (lbl) lbl.textContent = 'NOTSTOP AN';
                btn.classList.add('is-active');
            }
        } else {
            showBanner('Befehl fehlgeschlagen um ' + time + ' — wiederhole …');
            // Auto-retry once after 800ms
            setTimeout(function () {
                Promise.allSettled([
                    post('/api/control/emergency-stop', {}),
                    post('/api/control/stop', {}),
                    post('/api/control/move', { x: 0, y: 0 })
                ]).then(function (r2) {
                    const ok2 = r2.some(function (x) { return x.status === 'fulfilled' && x.value && x.value.ok; });
                    showBanner(ok2
                        ? 'Motoren gestoppt um ' + (new Date().toLocaleTimeString('de-DE'))
                        : 'Befehl wiederholt fehlgeschlagen — Hardware-Notstop prüfen!');
                });
            }, 800);
        }

        // Reset button visual after flash, breathing resumes
        setTimeout(function () {
            if (btn) btn.classList.remove('triggered');
        }, 900);
    }

    async function clearEstop() {
        showBanner('Notstop wird aufgehoben …');
        try {
            const r = await fetch('/api/control/emergency-stop', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ state: false })
            });
            const time = new Date().toLocaleTimeString('de-DE', { hour: '2-digit', minute: '2-digit', second: '2-digit' });
            if (r.ok) {
                showBanner('Notstop aufgehoben um ' + time + ' — Roboter wieder fahrbereit');
                setTimeout(hideBanner, 3000);
                const eb = document.getElementById('estop-btn');
                if (eb) {
                    const elb = eb.querySelector('.estop-label');
                    if (elb) elb.textContent = 'NOTSTOP AUS';
                    eb.classList.remove('is-active');
                }
            } else {
                showBanner('Aufheben fehlgeschlagen — bitte am Roboter prüfen');
            }
        } catch (e) {
            showBanner('Verbindungsfehler beim Aufheben — bitte am Roboter prüfen');
        }
    }

    function showBanner(subText) {
        const banner = document.getElementById('estop-banner');
        if (!banner) return;
        banner.classList.add('open');
        const sub = document.getElementById('estop-banner-sub');
        if (sub) sub.textContent = subText;

        if (bannerTimer) clearTimeout(bannerTimer);
        bannerTimer = setTimeout(hideBanner, 8000);
    }

    function hideBanner() {
        const banner = document.getElementById('estop-banner');
        if (banner) banner.classList.remove('open');
        if (bannerTimer) { clearTimeout(bannerTimer); bannerTimer = null; }
    }

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', injectEstop);
    } else {
        injectEstop();
    }
})();
