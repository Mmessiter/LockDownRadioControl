// LockDownRadioControl — RXV2 web UI shared helpers
//
// All pages link this script. It provides:
//   - fetchState()        — hits /api/state.json
//   - fetchEvents()       — hits /api/events.json (Black-box only)
//   - setText(id, value)  — populates an element by id, no-op if absent
//   - setHtml(id, value)  — same but innerHTML (use for trusted markup only)
//   - mountFooter()       — drops the FW version line into <div class=footer>
//   - startPolling(fn,ms) — calls fn() every ms, after the first fetch resolves
//
// Pages call mountFooter() once and (optionally) startPolling() for live data.

(function(){
    'use strict';

    window.LDRC = {
        state: null,
        events: null,

        async fetchState() {
            try {
                const r = await fetch('/api/state.json', { cache: 'no-store' });
                if (!r.ok) return null;
                this.state = await r.json();
                return this.state;
            } catch (e) { return null; }
        },

        async fetchEvents() {
            try {
                const r = await fetch('/api/events.json', { cache: 'no-store' });
                if (!r.ok) return null;
                this.events = await r.json();
                return this.events;
            } catch (e) { return null; }
        },

        setText(id, value) {
            const el = document.getElementById(id);
            if (!el) return;
            el.textContent = (value === undefined || value === null) ? '' : String(value);
        },

        setHtml(id, value) {
            const el = document.getElementById(id);
            if (!el) return;
            el.innerHTML = (value === undefined || value === null) ? '' : String(value);
        },

        setClass(id, cls, on) {
            const el = document.getElementById(id);
            if (!el) return;
            el.classList.toggle(cls, !!on);
        },

        // Format helpers
        msAgo(ms) {
            if (ms === undefined || ms === null || ms < 0) return 'never';
            return ms + ' ms ago';
        },

        sAgo(s) {
            if (s === undefined || s === null || s < 0) return 'never';
            return s + ' s ago';
        },

        mountFooter() {
            const f = document.querySelector('.footer');
            if (!f) return;
            // Populated by first state fetch; show a placeholder for now.
            if (!f.textContent.trim()) f.textContent = 'loading...';
        },

        async startPolling(fn, ms) {
            const tick = async () => {
                await this.fetchState();
                try { fn(this.state); } catch (e) { console.error(e); }
                setTimeout(tick, ms || 500);
            };
            tick();
        }
    };

    // Footer FW version on every page once state arrives.
    document.addEventListener('DOMContentLoaded', async () => {
        LDRC.mountFooter();
        await LDRC.fetchState();
        if (LDRC.state && LDRC.state.info) {
            const f = document.querySelector('.footer');
            if (f) f.textContent = LDRC.state.info.fw_version;
        }
    });
})();
