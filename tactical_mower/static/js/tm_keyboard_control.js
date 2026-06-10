
// ============================================================================
// KEYBOARD CONTROL — Apple TouchBar-style overlay on the /control page.
// Floating bottom-center, frosted glass, embossed key caps, direction radar.
// Multi-target capture so it fires no matter what.
// ============================================================================
(function () {
    if (window.__tmKbV3) return;
    window.__tmKbV3 = true;

    function isControlPage() {
        return !!document.getElementById('joystick-stick')
            || (location && /\/control(\/|$)/.test(location.pathname));
    }

    function ensureStyles() {
        if (document.getElementById('kb-pad-styles')) return;
        const s = document.createElement('style');
        s.id = 'kb-pad-styles';
        s.textContent = [
            '.kb-pad {',
            '  position: fixed; left: 50%; bottom: 24px; transform: translateX(-50%) translateY(0);',
            '  z-index: 9000; opacity: 0; pointer-events: none;',
            '  display: grid; grid-template-columns: auto 1px auto 1px auto; gap: 18px; align-items: center;',
            '  padding: 16px 22px;',
            '  background: rgba(255, 255, 255, 0.78);',
            '  -webkit-backdrop-filter: blur(28px) saturate(180%);',
            '  backdrop-filter: blur(28px) saturate(180%);',
            '  border-radius: 22px;',
            '  box-shadow:',
            '    0 24px 60px -20px rgba(15, 23, 42, 0.30),',
            '    0 8px 24px -8px rgba(15, 23, 42, 0.20),',
            '    inset 0 1px 0 rgba(255, 255, 255, 0.75);',
            '  font-family: var(--font-ui, "Manrope", "SF Pro Text", system-ui, sans-serif);',
            '  color: #0B1220;',
            '  transition: opacity 320ms cubic-bezier(0.16, 1, 0.3, 1), transform 320ms cubic-bezier(0.16, 1, 0.3, 1);',
            '}',
            '.kb-pad.ready { opacity: 1; pointer-events: auto; transform: translateX(-50%) translateY(0); }',
            '.kb-pad.intro { transform: translateX(-50%) translateY(40px); }',
            '.kb-pad-sep { width: 1px; height: 56px; background: linear-gradient(to bottom, transparent, rgba(15,23,42,0.10), transparent); }',
            '/* ---- Key cap grid ---- */',
            '.kb-pad-keys { display: grid; grid-template-columns: repeat(3, 44px); grid-template-rows: 44px 44px; gap: 6px; }',
            '.kb-key {',
            '  background: linear-gradient(180deg, #FFFFFF 0%, #EEF2F7 100%);',
            '  border-radius: 11px;',
            '  display: flex; align-items: center; justify-content: center;',
            '  font-weight: 700; font-size: 0.875rem;',
            '  color: #1F2937; user-select: none;',
            '  box-shadow:',
            '    0 1px 1px rgba(15, 23, 42, 0.05),',
            '    0 1px 0 rgba(15, 23, 42, 0.08),',
            '    inset 0 0 0 1px rgba(255, 255, 255, 0.6),',
            '    inset 0 -1px 0 rgba(15, 23, 42, 0.08);',
            '  transition: transform 80ms cubic-bezier(0.4, 0, 0.2, 1), box-shadow 80ms cubic-bezier(0.4, 0, 0.2, 1), background 80ms cubic-bezier(0.4, 0, 0.2, 1), color 80ms;',
            '  letter-spacing: -0.005em;',
            '}',
            '.kb-key-blank { background: transparent !important; box-shadow: none !important; }',
            '.kb-key.down {',
            '  background: linear-gradient(180deg, #2F74C7 0%, #235AA0 100%);',
            '  color: #fff;',
            '  transform: translateY(2px);',
            '  box-shadow:',
            '    0 0 0 1px rgba(35, 90, 160, 0.45),',
            '    0 4px 12px -4px rgba(47, 116, 199, 0.55),',
            '    inset 0 1px 0 rgba(255, 255, 255, 0.22);',
            '}',
            '/* ---- Direction radar ---- */',
            '.kb-pad-radar {',
            '  position: relative;',
            '  width: 100px; height: 100px;',
            '  background: radial-gradient(circle at center, rgba(15,23,42,0.04) 0%, rgba(15,23,42,0.08) 100%);',
            '  border-radius: 50%;',
            '  box-shadow: inset 0 0 0 1px rgba(15,23,42,0.10), inset 0 4px 12px rgba(15,23,42,0.08);',
            '  flex-shrink: 0;',
            '}',
            '.kb-radar-ring {',
            '  position: absolute; inset: 18px;',
            '  border-radius: 50%;',
            '  border: 1px dashed rgba(15,23,42,0.10);',
            '}',
            '.kb-radar-dot {',
            '  position: absolute; left: 50%; top: 50%;',
            '  width: 16px; height: 16px;',
            '  margin: -8px 0 0 -8px;',
            '  background: radial-gradient(circle at 35% 35%, #fff 0%, #2F74C7 60%, #235AA0 100%);',
            '  border-radius: 50%;',
            '  box-shadow:',
            '    0 4px 12px -2px rgba(47, 116, 199, 0.50),',
            '    0 0 0 2px rgba(255, 255, 255, 0.6),',
            '    inset 0 -2px 4px rgba(35, 90, 160, 0.40);',
            '  transition: transform 60ms cubic-bezier(0.4, 0, 0.2, 1);',
            '}',
            '/* ---- Telemetry pane ---- */',
            '.kb-pad-tele { display: flex; flex-direction: column; gap: 6px; min-width: 150px; }',
            '.kb-tele-row { display: flex; justify-content: space-between; align-items: center; gap: 14px; }',
            '.kb-tele-lbl {',
            '  font-size: 0.625rem; font-weight: 800;',
            '  letter-spacing: 0.10em; text-transform: uppercase;',
            '  color: #64748B;',
            '}',
            '.kb-tele-val {',
            '  font-family: "JetBrains Mono", ui-monospace, "SF Mono", monospace;',
            '  font-size: 0.875rem; font-weight: 700;',
            '  color: #0B1220;',
            '  font-variant-numeric: tabular-nums;',
            '}',
            '.kb-tele-bar {',
            '  height: 4px; background: rgba(15,23,42,0.08);',
            '  border-radius: 2px; overflow: hidden;',
            '}',
            '.kb-tele-bar-fill {',
            '  height: 100%; width: 0;',
            '  background: linear-gradient(90deg, #2F74C7, #235AA0);',
            '  border-radius: 2px;',
            '  transition: width 60ms cubic-bezier(0.4, 0, 0.2, 1);',
            '}',
            '.kb-tele-bar-fill.neg { background: linear-gradient(90deg, #235AA0, #2F74C7); margin-left: auto; }',
            '/* ---- Mode chip ---- */',
            '.kb-mode {',
            '  display: inline-flex; align-items: center; gap: 6px;',
            '  padding: 4px 10px; border-radius: 999px;',
            '  background: rgba(15,23,42,0.06); color: #475569;',
            '  font-size: 0.6875rem; font-weight: 800;',
            '  letter-spacing: 0.08em; text-transform: uppercase;',
            '  transition: background 140ms, color 140ms;',
            '  margin-top: 4px; align-self: flex-start;',
            '}',
            '.kb-mode::before { content: ""; width: 6px; height: 6px; border-radius: 50%; background: currentColor; opacity: 0.7; }',
            '.kb-mode.precise { background: rgba(47, 116, 199, 0.16); color: #235AA0; }',
            '.kb-mode.ultra   { background: rgba(127, 84, 232, 0.18); color: #5B21B6; }',
            '/* ---- Hint footer (inside the pad) ---- */',
            '.kb-pad-hint {',
            '  position: absolute; left: 50%; bottom: -22px;',
            '  transform: translateX(-50%);',
            '  font-size: 0.6875rem; font-weight: 500;',
            '  color: rgba(15,23,42,0.45); white-space: nowrap;',
            '  letter-spacing: 0.005em;',
            '  pointer-events: none;',
            '}',
            '.kb-pad-hint strong { color: rgba(15,23,42,0.65); font-weight: 700; }',
            '@media (max-width: 720px) {',
            '  .kb-pad { grid-template-columns: auto auto; gap: 14px; padding: 14px 16px; bottom: 16px; }',
            '  .kb-pad-sep, .kb-pad-radar { display: none; }',
            '  .kb-pad-keys { grid-template-columns: repeat(3, 38px); grid-template-rows: 38px 38px; }',
            '  .kb-key { font-size: 0.75rem; border-radius: 9px; }',
            '  .kb-pad-tele { min-width: 100px; }',
            '  .kb-pad-hint { display: none; }',
            '}',
            '@media (prefers-reduced-motion: reduce) { .kb-pad, .kb-key, .kb-radar-dot { transition: none !important; } }'
        ].join('\n');
        document.head.appendChild(s);
    }

    function buildPad() {
        if (document.getElementById('kb-pad')) return document.getElementById('kb-pad');
        const pad = document.createElement('div');
        pad.id = 'kb-pad';
        pad.className = 'kb-pad intro';
        pad.setAttribute('aria-label', 'Tastatur-Steuerung');
        pad.innerHTML = [
            '<div class="kb-pad-keys" aria-hidden="true">',
            '  <div class="kb-key kb-key-blank"></div>',
            '  <div class="kb-key" data-k="fwd">W</div>',
            '  <div class="kb-key kb-key-blank"></div>',
            '  <div class="kb-key" data-k="left">A</div>',
            '  <div class="kb-key" data-k="back">S</div>',
            '  <div class="kb-key" data-k="right">D</div>',
            '</div>',
            '<div class="kb-pad-sep"></div>',
            '<div class="kb-pad-radar" aria-hidden="true">',
            '  <div class="kb-radar-ring"></div>',
            '  <div class="kb-radar-dot" id="kb-radar-dot"></div>',
            '</div>',
            '<div class="kb-pad-sep"></div>',
            '<div class="kb-pad-tele">',
            '  <div class="kb-tele-row"><span class="kb-tele-lbl">Vor</span><span class="kb-tele-val" id="kb-val-y">0.00</span></div>',
            '  <div class="kb-tele-bar"><div class="kb-tele-bar-fill" id="kb-bar-y"></div></div>',
            '  <div class="kb-tele-row"><span class="kb-tele-lbl">Drehen</span><span class="kb-tele-val" id="kb-val-x">0.00</span></div>',
            '  <div class="kb-tele-bar"><div class="kb-tele-bar-fill" id="kb-bar-x"></div></div>',
            '  <span class="kb-mode" id="kb-mode">Normal</span>',
            '</div>',
            '<div class="kb-pad-hint"><strong>W A S D</strong> oder Pfeiltasten · <strong>Shift</strong> präzise · <strong>Ctrl</strong> ultra · <strong>Space</strong> Brake</div>'
        ].join('');
        document.body.appendChild(pad);
        requestAnimationFrame(function () { pad.classList.remove('intro'); pad.classList.add('ready'); });
        return pad;
    }

    function init() {
        if (!isControlPage()) return;
        ensureStyles();
        const pad = buildPad();

        const els = {
            keys: {
                fwd:   pad.querySelector('[data-k="fwd"]'),
                back:  pad.querySelector('[data-k="back"]'),
                left:  pad.querySelector('[data-k="left"]'),
                right: pad.querySelector('[data-k="right"]'),
            },
            radar: document.getElementById('kb-radar-dot'),
            valX:  document.getElementById('kb-val-x'),
            valY:  document.getElementById('kb-val-y'),
            barX:  document.getElementById('kb-bar-x'),
            barY:  document.getElementById('kb-bar-y'),
            mode:  document.getElementById('kb-mode'),
        };

        const KEY = {
            fwd:   ['w','W','ArrowUp'],
            back:  ['s','S','ArrowDown'],
            left:  ['a','A','ArrowLeft'],
            right: ['d','D','ArrowRight'],
        };
        const in_ = (arr, k) => arr.indexOf(k) >= 0;

        const st = { fwd:false, back:false, left:false, right:false, shift:false, ctrl:false, brake:false };
        let lastX = 0, lastY = 0;

        const mag = () => st.ctrl ? 0.12 : (st.shift ? 0.30 : 0.85);

        function compute() {
            if (st.brake) return { x: 0, y: 0 };
            const m = mag();
            let x = 0, y = 0;
            if (st.fwd)   y += m;
            if (st.back)  y -= m;
            if (st.right) x += m;
            if (st.left)  x -= m;
            const n = Math.hypot(x, y);
            if (n > 1) { x /= n; y /= n; }
            return { x: +x.toFixed(3), y: +y.toFixed(3) };
        }

        function fmt(v) {
            const s = v.toFixed(2);
            return (v >= 0 ? '+' + s : s);
        }

        function updateMode() {
            if (!els.mode) return;
            els.mode.classList.remove('precise', 'ultra');
            if (st.ctrl)       { els.mode.textContent = 'Ultra'; els.mode.classList.add('ultra'); }
            else if (st.shift) { els.mode.textContent = 'Präzise'; els.mode.classList.add('precise'); }
            else               { els.mode.textContent = 'Normal'; }
        }

        function paint(v) {
            // Key caps press state
            for (const k of ['fwd','back','left','right']) {
                if (els.keys[k]) els.keys[k].classList.toggle('down', !!st[k]);
            }
            // Values
            if (els.valX) els.valX.textContent = fmt(v.x);
            if (els.valY) els.valY.textContent = fmt(v.y);
            // Bars (split bipolar)
            if (els.barY) els.barY.style.width = Math.min(100, Math.abs(v.y) * 100) + '%';
            if (els.barX) els.barX.style.width = Math.min(100, Math.abs(v.x) * 100) + '%';
            // Radar dot — relative to center, max ~38px
            if (els.radar) {
                const r = 38;
                const dx = (v.x) * r;
                const dy = -(v.y) * r;  // screen Y is inverted; forward should go up
                els.radar.style.transform = 'translate(' + dx + 'px, ' + dy + 'px)';
            }
        }

        // ---------- API send ----------
        let inflight = false, pending = null;
        function send(v) {
            const body = JSON.stringify({ x: v.y, y: v.x });
            if (inflight) { pending = body; return; }
            inflight = true;
            /* DISABLED 2026-06-10 (double-publishing) */ if(false) fetch('/api/control/move', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: body,
                keepalive: true
            }).catch(function(){})
              .finally(function(){
                  inflight = false;
                  if (pending) {
                      const n = pending; pending = null;
                      /* DISABLED 2026-06-10 (double-publishing) */ if(false) fetch('/api/control/move', {
                          method: 'POST',
                          headers: { 'Content-Type': 'application/json' },
                          body: n, keepalive: true
                      }).catch(function(){});
                  }
              });
        }

        function push() {
            const v = compute();
            paint(v);
            if (v.x !== lastX || v.y !== lastY) {
                lastX = v.x; lastY = v.y;
                send(v);
            }
        }

        function isText(t) {
            if (!t) return false;
            const tag = (t.tagName || '').toUpperCase();
            return tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT' || t.isContentEditable;
        }

        function onDown(e) {
            if (isText(e.target)) return;
            if (e.repeat) return;
            const k = e.key;
            let h = false;
            if (in_(KEY.fwd,   k)) { st.fwd   = true; h = true; }
            if (in_(KEY.back,  k)) { st.back  = true; h = true; }
            if (in_(KEY.left,  k)) { st.left  = true; h = true; }
            if (in_(KEY.right, k)) { st.right = true; h = true; }
            if (k === ' ')        { st.brake = true; h = true; }
            if (k === 'Shift')    { st.shift = true; updateMode(); }
            if (k === 'Control')  { st.ctrl  = true; updateMode(); }
            if (h) { e.preventDefault(); push(); }
        }

        function onUp(e) {
            if (isText(e.target)) return;
            const k = e.key;
            if (in_(KEY.fwd,   k)) st.fwd   = false;
            if (in_(KEY.back,  k)) st.back  = false;
            if (in_(KEY.left,  k)) st.left  = false;
            if (in_(KEY.right, k)) st.right = false;
            if (k === ' ')        st.brake = false;
            if (k === 'Shift')    { st.shift = false; updateMode(); }
            if (k === 'Control')  { st.ctrl  = false; updateMode(); }
            push();
        }

        function release() {
            st.fwd = st.back = st.left = st.right = false;
            st.shift = st.ctrl = st.brake = false;
            updateMode(); push();
        }

        // Multi-target listeners so SOMETHING fires no matter what's focused
        for (const target of [window, document, document.body]) {
            if (!target) continue;
            target.addEventListener('keydown', onDown, true);
            target.addEventListener('keyup', onUp, true);
        }
        window.addEventListener('blur', release);
        document.addEventListener('visibilitychange', function () { if (document.hidden) release(); });

        updateMode();
        push();

        try { console.log('[kbpad] active on /control'); } catch (e) {}
    }

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', init);
    } else {
        init();
    }
})();
