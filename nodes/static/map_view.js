/**
 * Overhead map view for Argo web dashboard.
 * Static geometry from /api/map/geometry; live boat state from /api/status polling.
 */
(function () {
    'use strict';

    const DEG = Math.PI / 180;
    const VIS_SCALE = 3;
    const BOAT_LEN = 8 * VIS_SCALE;
    const BOAT_WID = 5 * VIS_SCALE;
    const ARROW_MIN = 12;
    const ARROW_MAX = 40;
    const WIND_ARROW_MIN = ARROW_MIN * VIS_SCALE;
    const WIND_ARROW_MAX = ARROW_MAX * VIS_SCALE;
    const ZOOM_MIN = 0.02;
    const ZOOM_MAX = 9;
    const GRID_SPACING_M = 10;
    const GRID_MIN_PX = 6;

    let geometry = null;
    let geometryMapName = null;
    let viewTransform = { scale: 1, offsetX: 0, offsetY: 0, width: 0, height: 0 };
    let userZoom = 1;
    let userPanX = 0;
    let userPanY = 0;

    let targetState = null;
    let displayState = null;
    let animFrameId = null;
    let mapEnabled = true;
    let hasAutoFitBoat = false;
    let lastDistanceM = null;
    let showGrid = false;

    function $(id) {
        return document.getElementById(id);
    }

    function setMapStatus(msg) {
        const el = $('map-view-status');
        if (el) el.textContent = msg || '—';
    }

    function lonLatToXY(lon, lat) {
        if (!geometry) return { x: 0, y: 0 };
        const R = 6378137.0;
        const olon = geometry.origin_lon;
        const olat = geometry.origin_lat;
        const x = (lon * DEG - olon * DEG) * R * Math.cos(olat * DEG);
        const y = (lat * DEG - olat * DEG) * R;
        return { x, y };
    }

    function compassToCanvasRad(compassDeg) {
        return (90 - compassDeg) * DEG;
    }

    function windFlowCompassDeg(compassHeading, windAngleRel) {
        if (compassHeading == null || windAngleRel == null) return null;
        const fromAbs = (compassHeading + windAngleRel) % 360;
        return (fromAbs + 180) % 360;
    }

    function extractLiveState(data) {
        const lat = data.gps_latitude != null ? data.gps_latitude : data.gps_last_valid_latitude;
        const lon = data.gps_longitude != null ? data.gps_longitude : data.gps_last_valid_longitude;
        let x = null;
        let y = null;
        if (lat != null && lon != null && geometry) {
            const pt = lonLatToXY(lon, lat);
            x = pt.x;
            y = pt.y;
        }
        if (data.distance_to_home != null) {
            lastDistanceM = Math.round(data.distance_to_home * 1852);
        }
        return {
            x, y,
            lat, lon,
            heading: data.compass_heading,
            wind_speed: data.wind_speed,
            wind_angle: data.wind_angle,
            cog: data.gps_cog,
            sog: data.gps_sog,
            stale: !!data.gps_data_stale,
            has_fix: lat != null && lon != null,
        };
    }

    function lerp(a, b, t) {
        if (a == null || b == null) return b;
        return a + (b - a) * t;
    }

    function lerpAngle(a, b, t) {
        if (a == null || b == null) return b;
        let diff = ((b - a + 540) % 360) - 180;
        return (a + diff * t + 360) % 360;
    }

    function boundsWithBoat(geom, boatX, boatY) {
        const bounds = geom && geom.bounds ? { ...geom.bounds } : null;
        if (!bounds) return null;
        if (boatX == null || boatY == null) return bounds;
        bounds.min_x = Math.min(bounds.min_x, boatX);
        bounds.max_x = Math.max(bounds.max_x, boatX);
        bounds.min_y = Math.min(bounds.min_y, boatY);
        bounds.max_y = Math.max(bounds.max_y, boatY);
        const span = Math.max(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y, 1);
        const margin = Math.max(25, span * 0.1);
        bounds.min_x -= margin;
        bounds.max_x += margin;
        bounds.min_y -= margin;
        bounds.max_y += margin;
        return bounds;
    }

    function computeFitTransform(canvas, bounds, padding) {
        if (!bounds) {
            const w = canvas.clientWidth || canvas.width;
            const h = canvas.clientHeight || canvas.height;
            return { scale: 1, offsetX: w / 2, offsetY: h / 2 };
        }
        const w = canvas.clientWidth || canvas.width;
        const h = canvas.clientHeight || canvas.height;
        const mapW = bounds.max_x - bounds.min_x || 1;
        const mapH = bounds.max_y - bounds.min_y || 1;
        const cx = (bounds.min_x + bounds.max_x) / 2;
        const cy = (bounds.min_y + bounds.max_y) / 2;
        const pad = padding || 24;
        const scale = Math.min((w - pad * 2) / mapW, (h - pad * 2) / mapH);
        return {
            scale,
            offsetX: w / 2 - cx * scale,
            offsetY: h / 2 + cy * scale,
            width: w,
            height: h,
            cx, cy,
        };
    }

    function fitBoundsIncludingBoat(resetUserZoom) {
        const canvas = $('argo-map-canvas');
        if (!canvas || !geometry) return;
        const st = targetState || displayState;
        const boatX = st && st.has_fix ? st.x : null;
        const boatY = st && st.has_fix ? st.y : null;
        const bounds = boundsWithBoat(geometry, boatX, boatY);
        viewTransform = computeFitTransform(canvas, bounds);
        if (resetUserZoom) {
            userZoom = 1;
            userPanX = 0;
            userPanY = 0;
        }
        updateMapStatusLine();
    }

    function updateMapStatusLine() {
        if (!geometry) return;
        const parts = [`${geometryMapName || geometry.map_name} — ${(geometry.boundaries || []).length} boundaries`];
        if (lastDistanceM != null) {
            parts.push(`${lastDistanceM} m from home`);
        }
        setMapStatus(parts.join(' · '));
    }

    function mapToScreen(x, y) {
        const t = viewTransform;
        const z = userZoom;
        const sx = (x * t.scale * z) + t.offsetX + userPanX;
        const sy = (-y * t.scale * z) + t.offsetY + userPanY;
        return { x: sx, y: sy };
    }

    function screenToMap(sx, sy) {
        const t = viewTransform;
        const k = t.scale * userZoom;
        if (k === 0) return { x: 0, y: 0 };
        return {
            x: (sx - t.offsetX - userPanX) / k,
            y: -(sy - t.offsetY - userPanY) / k,
        };
    }

    function zoomAtScreenPoint(sx, sy, newZoom) {
        const mapPt = screenToMap(sx, sy);
        userZoom = newZoom;
        const k = viewTransform.scale * userZoom;
        userPanX = sx - viewTransform.offsetX - mapPt.x * k;
        userPanY = sy - viewTransform.offsetY + mapPt.y * k;
    }

    function getVisibleMapBounds(w, h) {
        const corners = [
            screenToMap(0, 0),
            screenToMap(w, 0),
            screenToMap(0, h),
            screenToMap(w, h),
        ];
        return {
            min_x: Math.min(...corners.map(c => c.x)),
            max_x: Math.max(...corners.map(c => c.x)),
            min_y: Math.min(...corners.map(c => c.y)),
            max_y: Math.max(...corners.map(c => c.y)),
        };
    }

    function drawGrid(ctx, w, h) {
        const k = viewTransform.scale * userZoom;
        if (k <= 0 || GRID_SPACING_M * k < GRID_MIN_PX) return;

        const vis = getVisibleMapBounds(w, h);
        const pad = GRID_SPACING_M * 2;
        const minX = vis.min_x - pad;
        const maxX = vis.max_x + pad;
        const minY = vis.min_y - pad;
        const maxY = vis.max_y + pad;

        const x0 = Math.floor(minX / GRID_SPACING_M) * GRID_SPACING_M;
        const y0 = Math.floor(minY / GRID_SPACING_M) * GRID_SPACING_M;

        ctx.save();
        ctx.lineWidth = 1;

        for (let x = x0; x <= maxX; x += GRID_SPACING_M) {
            const major = Math.round(x) % 50 === 0;
            const p1 = mapToScreen(x, minY);
            const p2 = mapToScreen(x, maxY);
            ctx.beginPath();
            ctx.moveTo(p1.x, p1.y);
            ctx.lineTo(p2.x, p2.y);
            ctx.strokeStyle = major
                ? 'rgba(100, 130, 170, 0.45)'
                : 'rgba(140, 160, 190, 0.28)';
            ctx.lineWidth = major ? 1.25 : 1;
            ctx.stroke();
        }

        for (let y = y0; y <= maxY; y += GRID_SPACING_M) {
            const major = Math.round(y) % 50 === 0;
            const p1 = mapToScreen(minX, y);
            const p2 = mapToScreen(maxX, y);
            ctx.beginPath();
            ctx.moveTo(p1.x, p1.y);
            ctx.lineTo(p2.x, p2.y);
            ctx.strokeStyle = major
                ? 'rgba(100, 130, 170, 0.45)'
                : 'rgba(140, 160, 190, 0.28)';
            ctx.lineWidth = major ? 1.25 : 1;
            ctx.stroke();
        }

        ctx.restore();
    }

    function drawPolyline(ctx, points, stroke, lineWidth, closed) {
        if (!points || points.length < 2) return;
        ctx.beginPath();
        const p0 = mapToScreen(points[0][0], points[0][1]);
        ctx.moveTo(p0.x, p0.y);
        for (let i = 1; i < points.length; i++) {
            const p = mapToScreen(points[i][0], points[i][1]);
            ctx.lineTo(p.x, p.y);
        }
        if (closed) ctx.closePath();
        ctx.strokeStyle = stroke;
        ctx.lineWidth = lineWidth;
        ctx.stroke();
    }

    function drawArrow(ctx, x, y, compassDeg, lengthPx, color, lineWidth) {
        if (compassDeg == null) return;
        const rad = compassToCanvasRad(compassDeg);
        const ex = x + Math.cos(rad) * lengthPx;
        const ey = y + Math.sin(rad) * lengthPx;
        ctx.beginPath();
        ctx.moveTo(x, y);
        ctx.lineTo(ex, ey);
        ctx.strokeStyle = color;
        ctx.lineWidth = lineWidth || 2;
        ctx.stroke();
        const head = Math.max(6, lengthPx * 0.25);
        const a1 = rad + Math.PI * 0.82;
        const a2 = rad - Math.PI * 0.82;
        ctx.beginPath();
        ctx.moveTo(ex, ey);
        ctx.lineTo(ex + Math.cos(a1) * head, ey + Math.sin(a1) * head);
        ctx.moveTo(ex, ey);
        ctx.lineTo(ex + Math.cos(a2) * head, ey + Math.sin(a2) * head);
        ctx.stroke();
    }

    function drawBoat(ctx, x, y, headingDeg, stale) {
        const rad = compassToCanvasRad(headingDeg || 0);
        const cos = Math.cos(rad);
        const sin = Math.sin(rad);
        const hw = BOAT_WID / 2;
        const hl = BOAT_LEN / 2;
        const local = [
            [hl, 0],
            [-hl, hw],
            [-hl * 0.6, 0],
            [-hl, -hw],
        ];
        ctx.beginPath();
        local.forEach((pt, i) => {
            const rx = pt[0] * cos - pt[1] * sin;
            const ry = pt[0] * sin + pt[1] * cos;
            const sx = x + rx;
            const sy = y + ry;
            if (i === 0) ctx.moveTo(sx, sy);
            else ctx.lineTo(sx, sy);
        });
        ctx.closePath();
        ctx.fillStyle = stale ? 'rgba(255, 107, 107, 0.85)' : 'rgba(30, 60, 114, 0.95)';
        ctx.fill();
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 2;
        ctx.stroke();
    }

    function drawHomeToBoatLine(ctx, homeX, homeY, boatX, boatY) {
        const hp = mapToScreen(homeX, homeY);
        const bp = mapToScreen(boatX, boatY);
        ctx.beginPath();
        ctx.setLineDash([6, 6]);
        ctx.moveTo(hp.x, hp.y);
        ctx.lineTo(bp.x, bp.y);
        ctx.strokeStyle = 'rgba(30, 60, 114, 0.45)';
        ctx.lineWidth = 1.5;
        ctx.stroke();
        ctx.setLineDash([]);
    }

    function renderFrame() {
        const canvas = $('argo-map-canvas');
        if (!canvas || !geometry || !mapEnabled) return;

        const dpr = window.devicePixelRatio || 1;
        const w = canvas.clientWidth;
        const h = canvas.clientHeight;
        if (w <= 0 || h <= 0) return;

        if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
            canvas.width = Math.round(w * dpr);
            canvas.height = Math.round(h * dpr);
            fitBoundsIncludingBoat(false);
        }

        const ctx = canvas.getContext('2d');
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        ctx.clearRect(0, 0, w, h);

        ctx.fillStyle = '#e8f0f8';
        ctx.fillRect(0, 0, w, h);

        if (showGrid) {
            drawGrid(ctx, w, h);
        }

        (geometry.boundaries || []).forEach(b => {
            drawPolyline(ctx, b.points, '#28a745', 2, b.type === 'sailing_area');
        });
        (geometry.hazards || []).forEach(hz => {
            drawPolyline(ctx, hz.points, '#dc3545', 2.5, true);
        });

        const home = (geometry.waypoints || []).find(w => w.is_home);
        const homeX = home ? home.x : 0;
        const homeY = home ? home.y : 0;

        (geometry.waypoints || []).forEach(wp => {
            const p = mapToScreen(wp.x, wp.y);
            const r = wp.is_home ? 7 : 4;
            ctx.beginPath();
            ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
            ctx.fillStyle = wp.is_home ? 'rgba(40, 167, 69, 0.35)' : '#28a745';
            ctx.fill();
            if (wp.name) {
                ctx.fillStyle = '#333';
                ctx.font = '11px sans-serif';
                ctx.fillText(wp.name, p.x + 8, p.y - 4);
            }
        });

        const st = displayState;
        if (st && st.has_fix && st.x != null && st.y != null) {
            const distMap = Math.hypot(st.x - homeX, st.y - homeY);
            if (distMap > 15) {
                drawHomeToBoatLine(ctx, homeX, homeY, st.x, st.y);
            }

            const bp = mapToScreen(st.x, st.y);

            if (st.wind_speed != null && st.wind_angle != null && st.heading != null) {
                const flow = windFlowCompassDeg(st.heading, st.wind_angle);
                const wLen = Math.min(
                    WIND_ARROW_MAX,
                    Math.max(WIND_ARROW_MIN, st.wind_speed * 8 * VIS_SCALE)
                );
                drawArrow(ctx, bp.x, bp.y, flow, wLen, 'rgba(40, 167, 69, 0.85)', 2.5);
            }

            if (st.sog != null && st.cog != null && st.sog > 0.05) {
                const vLen = Math.min(ARROW_MAX, Math.max(ARROW_MIN, st.sog * 6));
                drawArrow(ctx, bp.x, bp.y, st.cog, vLen, 'rgba(255, 193, 7, 0.9)', 1.5);
            }

            drawBoat(ctx, bp.x, bp.y, st.heading, st.stale);
        }
    }

    function tickAnimation() {
        if (!mapEnabled || document.hidden) {
            animFrameId = null;
            return;
        }
        if (targetState && displayState) {
            const alpha = 0.25;
            displayState = {
                x: lerp(displayState.x, targetState.x, alpha),
                y: lerp(displayState.y, targetState.y, alpha),
                lat: targetState.lat,
                lon: targetState.lon,
                heading: lerpAngle(displayState.heading, targetState.heading, alpha),
                wind_speed: targetState.wind_speed,
                wind_angle: targetState.wind_angle,
                cog: targetState.cog,
                sog: targetState.sog,
                stale: targetState.stale,
                has_fix: targetState.has_fix,
            };
        } else if (targetState) {
            displayState = { ...targetState };
        }
        renderFrame();
        animFrameId = requestAnimationFrame(tickAnimation);
    }

    function ensureAnimation() {
        if (animFrameId == null && mapEnabled && !document.hidden) {
            animFrameId = requestAnimationFrame(tickAnimation);
        }
    }

    function loadMapGeometry(mapName) {
        const query = mapName ? `?map=${encodeURIComponent(mapName)}` : '';
        setMapStatus('Loading map…');
        return fetch(`/api/map/geometry${query}`)
            .then(r => r.json())
            .then(data => {
                if (!data.success) throw new Error(data.message || 'Failed to load map');
                const sameMap = geometry && geometryMapName === data.map_name;
                geometry = data;
                geometryMapName = data.map_name;
                if (!sameMap) {
                    hasAutoFitBoat = false;
                    userZoom = 1;
                    userPanX = 0;
                    userPanY = 0;
                }
                fitBoundsIncludingBoat(!sameMap || !hasAutoFitBoat);
                if (targetState && targetState.has_fix && !hasAutoFitBoat) {
                    hasAutoFitBoat = true;
                    fitBoundsIncludingBoat(true);
                }
                renderFrame();
                ensureAnimation();
                return data;
            })
            .catch(err => {
                setMapStatus(`❌ ${err.message || err}`);
                geometry = null;
            });
    }

    function updateMapFromStatus(data) {
        if (!mapEnabled || !geometry) return;
        targetState = extractLiveState(data);
        if (targetState.has_fix && !hasAutoFitBoat) {
            hasAutoFitBoat = true;
            fitBoundsIncludingBoat(true);
        } else {
            updateMapStatusLine();
        }
        ensureAnimation();
    }

    function recenterMap() {
        fitBoundsIncludingBoat(true);
        renderFrame();
    }

    function setupMapInteractions() {
        const canvas = $('argo-map-canvas');
        const recenterBtn = $('map-recenter-btn');
        if (recenterBtn) recenterBtn.addEventListener('click', recenterMap);

        const gridToggle = $('map-grid-toggle');
        if (gridToggle) {
            try {
                showGrid = localStorage.getItem('argoMapShowGrid') === '1';
            } catch (e) { /* ignore */ }
            gridToggle.checked = showGrid;
            gridToggle.addEventListener('change', () => {
                showGrid = gridToggle.checked;
                try {
                    localStorage.setItem('argoMapShowGrid', showGrid ? '1' : '0');
                } catch (e) { /* ignore */ }
                renderFrame();
            });
        }

        if (!canvas) return;

        let dragging = false;
        let lastX = 0;
        let lastY = 0;

        canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            const rect = canvas.getBoundingClientRect();
            const mx = e.clientX - rect.left;
            const my = e.clientY - rect.top;
            const factor = e.deltaY < 0 ? 1.12 : 0.88;
            const newZoom = Math.min(ZOOM_MAX, Math.max(ZOOM_MIN, userZoom * factor));
            zoomAtScreenPoint(mx, my, newZoom);
            renderFrame();
        }, { passive: false });

        canvas.addEventListener('mousedown', (e) => {
            dragging = true;
            lastX = e.clientX;
            lastY = e.clientY;
        });
        window.addEventListener('mouseup', () => { dragging = false; });
        window.addEventListener('mousemove', (e) => {
            if (!dragging) return;
            userPanX += e.clientX - lastX;
            userPanY += e.clientY - lastY;
            lastX = e.clientX;
            lastY = e.clientY;
            renderFrame();
        });

        canvas.addEventListener('touchstart', (e) => {
            if (e.touches.length === 1) {
                dragging = true;
                lastX = e.touches[0].clientX;
                lastY = e.touches[0].clientY;
            }
        }, { passive: true });
        canvas.addEventListener('touchend', () => { dragging = false; });
        canvas.addEventListener('touchmove', (e) => {
            if (!dragging || e.touches.length !== 1) return;
            e.preventDefault();
            userPanX += e.touches[0].clientX - lastX;
            userPanY += e.touches[0].clientY - lastY;
            lastX = e.touches[0].clientX;
            lastY = e.touches[0].clientY;
            renderFrame();
        }, { passive: false });

        window.addEventListener('resize', () => {
            if (geometry) {
                fitBoundsIncludingBoat(false);
                renderFrame();
            }
        });

        document.addEventListener('visibilitychange', () => {
            if (document.hidden) {
                if (animFrameId != null) {
                    cancelAnimationFrame(animFrameId);
                    animFrameId = null;
                }
            } else {
                ensureAnimation();
            }
        });
    }

    window.ArgoMapView = {
        init: function () {
            setupMapInteractions();
            loadMapGeometry();
        },
        reload: loadMapGeometry,
        updateFromStatus: updateMapFromStatus,
        setEnabled: function (enabled) {
            mapEnabled = enabled;
            if (enabled) ensureAnimation();
            else if (animFrameId != null) {
                cancelAnimationFrame(animFrameId);
                animFrameId = null;
            }
        },
    };
})();
