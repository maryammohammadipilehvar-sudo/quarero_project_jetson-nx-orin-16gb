/* ============================================================================
 * tm_map_controls_apple.js
 * 1) Move Leaflet's .leaflet-control-layers into our .hero-v2-map-controls pill
 * 2) Override Leaflet's auto-collapse on mouseleave (the popout sits in a
 *    relocated position so Leaflet thinks the mouse "left" instantly,
 *    causing the panel to flicker open/closed).
 * 3) Bind our own click handler that toggles the expanded class reliably,
 *    plus an outside-click handler to close.
 * ========================================================================== */
(function () {
    'use strict';

    function rebind(ctrl) {
        if (ctrl.dataset.appleBound === '1') return;
        ctrl.dataset.appleBound = '1';

        var toggle = ctrl.querySelector('.leaflet-control-layers-toggle');
        if (!toggle) return;

        // Stop Leaflet's mouseover/mouseout from firing — they're the culprit
        // for the flicker (they hide the panel when the cursor isn't *exactly*
        // over the small toggle/popout area).
        ['mouseover', 'mouseout', 'mouseenter', 'mouseleave'].forEach(function (ev) {
            ctrl.addEventListener(ev, function (e) {
                e.stopImmediatePropagation();
            }, true);
        });

        // Our own click toggle — capture phase so we run before Leaflet
        toggle.addEventListener('click', function (e) {
            e.preventDefault();
            e.stopImmediatePropagation();
            ctrl.classList.toggle('leaflet-control-layers-expanded');
        }, true);

        // Clicks on radio/checkbox inputs INSIDE the panel should propagate
        // so Leaflet still switches layers. They naturally do — we only
        // stopped propagation on hover events.

        // Close panel when clicking outside it
        document.addEventListener('click', function (e) {
            if (!ctrl.contains(e.target) && !toggle.contains(e.target)) {
                ctrl.classList.remove('leaflet-control-layers-expanded');
            }
        });
    }

    function relocate() {
        var pill = document.querySelector('.hero-v2-map-controls');
        var layers = document.querySelector('.leaflet-control-layers');
        if (!pill || !layers) return false;
        if (layers.parentElement !== pill) {
            pill.appendChild(layers);
            layers.classList.add('in-pill');
        }
        rebind(layers);
        return true;
    }

    function start() {
        if (relocate()) return;
        var attempts = 0;
        var iv = setInterval(function () {
            attempts++;
            if (relocate() || attempts > 40) clearInterval(iv);
        }, 200);
    }

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', start);
    } else {
        start();
    }
})();
