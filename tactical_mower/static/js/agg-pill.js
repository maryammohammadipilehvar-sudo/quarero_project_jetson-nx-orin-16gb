
// =====================================================
// Aggregierte Status-Pille (v2 Design)
// Konsolidiert die 4 Subsystem-Badges (GNSS1/GNSS2/IMU/Fusion)
// in eine einzelne Pille mit Drill-Down-Popover.
//
// Bestehende Logik in updateFusionBadges() bleibt unverändert —
// wir beobachten die 4 versteckten Badges via MutationObserver
// und leiten den aggregierten Zustand ab.
// =====================================================
(function() {
    const BADGE_IDS = ['badge-gnss1', 'badge-gnss2', 'badge-imu', 'badge-fusion'];
    const SEMANTIC_KEY = {
        'badge-gnss1': 'gnss1',
        'badge-gnss2': 'gnss2',
        'badge-imu':   'imu',
        'badge-fusion':'fusion'
    };
    const STATUS_DETAIL = {
        'badge-gnss1':  { green: 'RTK Fix', orange: 'Float',   red: 'Inaktiv' },
        'badge-gnss2':  { green: 'RTK Fix', orange: 'Float',   red: 'Inaktiv' },
        'badge-imu':    { green: 'Stabil',  orange: 'Initial', red: 'Fehler'   },
        'badge-fusion': { green: 'Aktiv',   orange: 'Anlauf',  red: 'Inaktiv'  }
    };

    function getBadgeState(el) {
        if (!el) return 'unknown';
        if (el.classList.contains('red')) return 'red';
        if (el.classList.contains('orange')) return 'orange';
        if (el.classList.contains('green')) return 'green';
        return 'unknown';
    }

    function rank(s) {
        return ({ red: 3, orange: 2, unknown: 1, green: 0 })[s] || 0;
    }

    function injectPill() {
        if (document.getElementById('agg-pill')) return;
        const oldGroup = document.querySelector('.nav-status-group');
        if (!oldGroup || !oldGroup.parentNode) return;

        const wrap = document.createElement('div');
        wrap.className = 'agg-pill-wrap';
        wrap.innerHTML =
            '<button id="agg-pill" class="agg-pill" type="button" aria-haspopup="true" aria-expanded="false" aria-label="Systemstatus">' +
                '<span class="agg-pill-dot" aria-hidden="true"></span>' +
                '<span class="agg-pill-text"><span id="agg-pill-label">Initialisiere</span><span class="agg-pill-sub" id="agg-pill-count">—</span></span>' +
            '</button>' +
            '<div id="agg-popover" class="agg-popover" role="dialog" aria-label="Subsystem-Status">' +
                '<div class="agg-row" data-key="gnss1"><span class="agg-row-lbl">GNSS Primär</span><span class="agg-row-val" id="agg-val-gnss1">—</span></div>' +
                '<div class="agg-row" data-key="gnss2"><span class="agg-row-lbl">GNSS Sekundär</span><span class="agg-row-val" id="agg-val-gnss2">—</span></div>' +
                '<div class="agg-row" data-key="imu"><span class="agg-row-lbl">IMU</span><span class="agg-row-val" id="agg-val-imu">—</span></div>' +
                '<div class="agg-row" data-key="fusion"><span class="agg-row-lbl">Sensor-Fusion</span><span class="agg-row-val" id="agg-val-fusion">—</span></div>' +
            '</div>';
        oldGroup.parentNode.insertBefore(wrap, oldGroup.nextSibling);

        const pill = document.getElementById('agg-pill');
        const popover = document.getElementById('agg-popover');
        pill.addEventListener('click', e => {
            e.stopPropagation();
            const isOpen = popover.classList.toggle('open');
            pill.setAttribute('aria-expanded', isOpen ? 'true' : 'false');
        });
        document.addEventListener('click', e => {
            if (popover.classList.contains('open') && !popover.contains(e.target) && e.target !== pill && !pill.contains(e.target)) {
                popover.classList.remove('open');
                pill.setAttribute('aria-expanded', 'false');
            }
        });
        document.addEventListener('keydown', e => {
            if (e.key === 'Escape' && popover.classList.contains('open')) {
                popover.classList.remove('open');
                pill.setAttribute('aria-expanded', 'false');
            }
        });
    }

    function updatePill() {
        const pill = document.getElementById('agg-pill');
        if (!pill) return;

        const states = {};
        let worst = 'unknown';
        let okCount = 0;
        BADGE_IDS.forEach(id => {
            const s = getBadgeState(document.getElementById(id));
            states[id] = s;
            if (s === 'green') okCount++;
            if (rank(s) > rank(worst)) worst = s;
        });

        pill.classList.remove('ok', 'warn', 'error');
        let label, sub;
        if (worst === 'red') {
            pill.classList.add('error');
            const failing = BADGE_IDS.filter(id => states[id] === 'red').length;
            label = failing > 1 ? failing + ' Systeme prüfen' : 'System prüfen';
            sub = okCount + '/4';
        } else if (worst === 'orange') {
            pill.classList.add('warn');
            label = 'Eingeschränkt';
            sub = okCount + '/4';
        } else if (okCount === BADGE_IDS.length) {
            // All 4 badges green — fully OK
            pill.classList.add('ok');
            label = 'Systeme OK';
            sub = '4/4';
        } else if (okCount > 0) {
            // Some green, some unknown — partially OK, still coming online
            pill.classList.add('ok');
            label = 'Systeme OK';
            sub = okCount + '/4';
        } else {
            // No data yet
            label = 'Warte auf Daten';
            sub = '—';
        }
        const labelEl = document.getElementById('agg-pill-label');
        if (labelEl) labelEl.textContent = label;
        const subEl = document.getElementById('agg-pill-count');
        if (subEl) subEl.textContent = sub;

        BADGE_IDS.forEach(id => {
            const key = SEMANTIC_KEY[id];
            const detail = STATUS_DETAIL[id];
            const row = document.querySelector('.agg-row[data-key="' + key + '"]');
            const val = document.getElementById('agg-val-' + key);
            if (!row || !val) return;
            row.classList.remove('ok', 'warn', 'error');
            const s = states[id];
            if (s === 'green')       { row.classList.add('ok');    val.textContent = detail.green; }
            else if (s === 'orange') { row.classList.add('warn');  val.textContent = detail.orange; }
            else if (s === 'red')    { row.classList.add('error'); val.textContent = detail.red; }
            else                     { val.textContent = '—'; }
        });
    }

    function setupObservers() {
        BADGE_IDS.forEach(id => {
            const el = document.getElementById(id);
            if (!el) return;
            new MutationObserver(updatePill).observe(el, { attributes: true, attributeFilter: ['class'] });
        });
        updatePill();
    }

    function init() {
        injectPill();
        setupObservers();
    }

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', init);
    } else {
        init();
    }
})();
