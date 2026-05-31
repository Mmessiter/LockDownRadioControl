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

        // Drop-in async replacement for window.confirm — colourful modal.
        // Usage: if (await LDRC.confirm('Write to EEPROM?')) ...
        // Options: { title, message, icon, yes, no, kind: 'go'|'warn'|'danger' }
        confirm(message, opts) {
            opts = opts || {};
            const kind = opts.kind || 'go';    // 'go' (green) | 'warn' (amber) | 'danger' (red)
            return new Promise(resolve => {
                let ov = document.getElementById('_ldrcModal');
                if (!ov) {
                    ov = document.createElement('div');
                    ov.id = '_ldrcModal';
                    ov.className = 'modalOverlay';
                    ov.innerHTML = '<div class=modalBox>'
                        + '<div class=modalAccent id=_mAcc>'
                        +   '<div class=modalIcon id=_mIcon>⚡</div>'
                        +   '<div class=modalTitle id=_mTitle></div>'
                        + '</div>'
                        + '<div class=modalMsg id=_mMsg></div>'
                        + '<div class=modalBtns>'
                        +   '<button class="modalBtn no" id=_mNo>Cancel</button>'
                        +   '<button class="modalBtn yes" id=_mYes>OK</button>'
                        + '</div></div>';
                    document.body.appendChild(ov);
                }
                const acc   = document.getElementById('_mAcc');
                const yesBt = document.getElementById('_mYes');
                const noBt  = document.getElementById('_mNo');
                acc.className = 'modalAccent' + (kind === 'warn' ? ' warn' : kind === 'danger' ? ' danger' : '');
                yesBt.className = 'modalBtn yes' + (kind === 'warn' ? ' warn' : kind === 'danger' ? ' danger' : '');
                document.getElementById('_mIcon').textContent  = opts.icon  || (kind === 'danger' ? '⚠️' : kind === 'warn' ? '✏️' : '💾');
                document.getElementById('_mTitle').textContent = opts.title || (kind === 'danger' ? 'Are you sure?' : 'Confirm');
                document.getElementById('_mMsg').textContent   = message;
                yesBt.textContent = opts.yes || 'Yes, do it';
                noBt.textContent  = opts.no  || 'Cancel';
                const close = (ok) => { ov.classList.remove('show'); yesBt.onclick = null; noBt.onclick = null; resolve(ok); };
                yesBt.onclick = () => close(true);
                noBt.onclick  = () => close(false);
                ov.classList.add('show');
                setTimeout(() => yesBt.focus(), 100);
            });
        },

        // Help modal. Each page provides its own help text in a hidden
        // `<template id=helpContent>` element near the bottom of the
        // body, and a floating `?` button anywhere with onclick
        // "LDRC.showHelp()". The function builds an overlay with the
        // template's content and a "Got it" button. Tap outside the
        // panel or the close button to dismiss.
        showHelp() {
            const tpl = document.getElementById('helpContent');
            const html = tpl ? tpl.innerHTML
                             : '<p>No help text on this page yet.</p>';
            const overlay = document.createElement('div');
            overlay.className = 'helpModal';
            overlay.innerHTML =
                '<div class=helpPanel>' + html +
                '<button class=helpClose type=button>Got it</button>' +
                '</div>';
            const close = () => overlay.remove();
            overlay.addEventListener('click', (e) => {
                if (e.target === overlay) close();
            });
            overlay.querySelector('.helpClose').addEventListener('click', close);
            document.body.appendChild(overlay);
        },

        // Retry-on-fail wrapper for the /api/msp endpoint. MSP responses
        // occasionally drop on the CRSF wire — pause 200 ms and try again,
        // up to `retries` total attempts. Returns response text or throws.
        async msp(fn, dataHex, retries) {
            retries = retries || 3;
            const url = '/api/msp?fn=' + fn + (dataHex ? '&data=' + dataHex : '');
            let lastErr = null;
            for (let attempt = 0; attempt < retries; attempt++) {
                try {
                    const r = await fetch(url, { cache: 'no-store' });
                    if (r.ok) return (await r.text()).trim();
                    lastErr = new Error('HTTP ' + r.status + ': ' + (await r.text()));
                } catch (e) { lastErr = e; }
                if (attempt < retries - 1) await new Promise(rs => setTimeout(rs, 200));
            }
            throw lastErr;
        },

        // Rotorflight API version we've actually verified the byte layouts
        // against. Pages that talk MSP for tuning compare the FC's API to
        // this and warn if the FC is newer — byte fields can shift across
        // Rotorflight releases and the page may silently write the wrong
        // bytes. Bump this in sync with the firmware source we re-verified.
        // 1209 == Rotorflight 2.3 (firmware RF-4.6.x).
        RF_API_VERIFIED: 1209,
        rfVersionAlert(api) {
            // Returns a warning string if FC is newer than verified, else null.
            if (!api || api < 100) return null;
            if (api <= this.RF_API_VERIFIED) return null;
            const maj = Math.floor(api / 100), min = api % 100;
            const vMaj = Math.floor(this.RF_API_VERIFIED / 100), vMin = this.RF_API_VERIFIED % 100;
            return 'Rotorflight API ' + maj + '.' + min +
                   ' is newer than the version this LDRC firmware was tested against (' +
                   vMaj + '.' + vMin + '). Field positions may have shifted — please ' +
                   'update LDRC firmware. Edits are still allowed, but Save-and-verify ' +
                   'will show what the flight controller actually accepted.';
        },

        // Dirty-tracking: tuning pages call markDirty() on any user input
        // (via a delegated 'input' listener) and clearDirty() after a
        // successful load() or save(). confirmLoseChanges() shows a
        // dramatic warning before profile-switches that would discard
        // un-saved edits — most often the user has just nudged a value
        // and the new bank's load() would silently overwrite it.
        dirty: false,
        markDirty()  { this.dirty = true;  },
        clearDirty() { this.dirty = false; },
        async confirmLoseChanges(action) {
            if (!this.dirty) return true;
            const msg = action
                ? 'You have un-saved changes. They will be lost when ' + action + ' — they CANNOT be recovered. Save first, or continue and discard them.'
                : 'You have un-saved changes. They will be lost. Save first, or continue and discard them.';
            const ok = await this.confirm(msg,
                {title:'Un-saved changes!', icon:'⚠️', yes:'Discard & continue', no:'Stay & save', kind:'danger'});
            if (ok) this.dirty = false;
            return ok;
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

    // Footer FW version. Defer the fetch by 200 ms so it can't block the
    // first paint, and never await it on DOMContentLoaded (a stalled
    // /api/state.json was queuing nav requests on the chip's single-client
    // WebServer for the iOS keep-alive socket).
    document.addEventListener('DOMContentLoaded', () => {
        LDRC.mountFooter();
        setTimeout(() => {
            LDRC.fetchState().then(() => {
                if (LDRC.state && LDRC.state.info) {
                    const f = document.querySelector('.footer');
                    if (f) f.textContent = LDRC.state.info.fw_version;
                }
            });
        }, 200);
    });

    // Click interceptor — ported from the Reed remaking-machine project,
    // where this pattern gives near-instant nav on iOS Safari:
    //   - e.preventDefault() then JS-initiated location.href = a.href
    //     bypasses iOS's 300 ms double-tap-zoom delay
    //   - the `navigating` flag swallows re-taps so they can't queue
    //     additional requests on the single-client WebServer
    //   - 50 ms setTimeout gives Safari one paint tick to show the overlay
    let _navigating = false;
    document.addEventListener('click', async e => {
        if (_navigating) { e.preventDefault(); return; }
        const a = e.target.closest('a[href]');
        if (!a || a.target) return;
        const href = a.getAttribute('href');
        if (!href || href === '#' || href[0] === '#') return;
        if (href.startsWith('blob:') || href.startsWith('data:')) return;
        if (/^https?:|^mailto:|^tel:/i.test(href) && a.origin && a.origin !== location.origin) return;
        if (a.hasAttribute('download')) return;
        e.preventDefault();
        // If the current page has un-saved edits, ask before navigating
        // away — the new page would silently discard them.
        if (LDRC.dirty) {
            if (!(await LDRC.confirmLoseChanges('leaving this page'))) return;
        }
        _navigating = true;
        LDRC.showLoading();
        setTimeout(() => { location.href = a.href; }, 50);
    }, true);
    document.addEventListener('submit', () => LDRC.showLoading(), true);
    // Browser back/refresh/close: the native "Leave site?" dialog is the
    // only way to intercept these. Returning any string triggers it.
    window.addEventListener('beforeunload', e => {
        if (LDRC.dirty && !_navigating) {
            e.preventDefault();
            e.returnValue = 'Un-saved changes will be lost.';
            return e.returnValue;
        }
    });
    // bfcache: iOS may restore the page with the overlay still up.
    window.addEventListener('pageshow', () => {
        _navigating = false;
        LDRC.hideLoading();
    });

    LDRC.showLoading = function(msg) {
        let ov = document.getElementById('_ldrcLoad');
        if (!ov) {
            ov = document.createElement('div');
            ov.id = '_ldrcLoad';
            ov.innerHTML = '<div class=loadingBox><div class=loadingSpin></div><div id=_ldrcLoadMsg>Please wait&hellip;</div></div>';
            document.body.appendChild(ov);
        }
        const m = document.getElementById('_ldrcLoadMsg');
        if (m) m.innerHTML = msg || 'Please wait&hellip;';
        ov.classList.add('show');
    };
    LDRC.hideLoading = function() {
        const ov = document.getElementById('_ldrcLoad');
        if (ov) ov.classList.remove('show');
    };

    // Stale-values overlay — used by Rotorflight value-display pages
    // while a fresh fetch is in flight. Toggles a <body> class that
    // CSS (style.css) turns into "fade values to grey". Call markStale
    // at the start of a load, clearStale after the last .value = ...
    // assignment. Failed loads should leave the page stale (values
    // remain suspect until something actually arrives).
    LDRC.markStale  = function() { document.body.classList.add('stale-values'); };
    LDRC.clearStale = function() { document.body.classList.remove('stale-values'); };
})();
