class BladeDetailPanel {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        this.turbineId = 0;
        this.bladeId = 0;
        this.data = null;
        this.history = [];
        this.visible = false;
        this.render();
    }

    render() {
        if (!this.container) return;
        this.container.innerHTML = `
            <div class="blade-panel-header">
                <div class="blade-title">
                    <span id="panel-badge" class="badge-stage badge-stage-0">正常</span>
                    <h3 id="panel-title">叶片详情</h3>
                </div>
                <button id="panel-close" class="btn-close" title="关闭">×</button>
            </div>
            <div class="panel-body">
                <div class="info-grid">
                    <div class="info-item">
                        <div class="info-label">水轮机编号</div>
                        <div class="info-value" id="panel-turbine">-</div>
                    </div>
                    <div class="info-item">
                        <div class="info-label">叶片编号</div>
                        <div class="info-value" id="panel-blade">-</div>
                    </div>
                    <div class="info-item">
                        <div class="info-label">空化强度</div>
                        <div class="info-value">
                            <div class="progress-bar">
                                <div id="panel-intensity-bar" class="progress-fill progress-low"></div>
                            </div>
                            <span id="panel-intensity">-</span>
                        </div>
                    </div>
                    <div class="info-item">
                        <div class="info-label">累计损伤</div>
                        <div class="info-value">
                            <div class="progress-bar">
                                <div id="panel-damage-bar" class="progress-fill progress-low"></div>
                            </div>
                            <span id="panel-damage">-</span>
                        </div>
                    </div>
                    <div class="info-item">
                        <div class="info-label">应力 MPa</div>
                        <div class="info-value" id="panel-stress">-</div>
                    </div>
                    <div class="info-item">
                        <div class="info-label">剩余寿命</div>
                        <div class="info-value" id="panel-life">-</div>
                    </div>
                </div>
                <div class="chart-section">
                    <div class="section-title">
                        <span>空化强度历史趋势</span>
                        <div class="chart-legend">
                            <span class="legend-item"><span class="dot dot-intensity"></span>强度</span>
                            <span class="legend-item"><span class="dot dot-incipient"></span>初生</span>
                            <span class="legend-item"><span class="dot dot-critical"></span>临界</span>
                        </div>
                    </div>
                    <canvas id="history-chart" class="history-canvas" width="500" height="180"></canvas>
                </div>
                <div class="gauges-section">
                    <div class="section-title">寿命与健康评估</div>
                    <div class="gauges-row">
                        <div class="gauge-item">
                            <canvas id="gauge-life" width="180" height="120"></canvas>
                            <div class="gauge-label">剩余寿命 (%)</div>
                        </div>
                        <div class="gauge-item">
                            <canvas id="gauge-health" width="180" height="120"></canvas>
                            <div class="gauge-label">健康指数</div>
                        </div>
                    </div>
                </div>
                <div class="action-section">
                    <button id="btn-detail" class="btn btn-primary">查看详细分析</button>
                    <button id="btn-history" class="btn btn-secondary">历史报告</button>
                    <button id="btn-notify" class="btn btn-warning">关注此叶片</button>
                </div>
            </div>
        `;
        document.getElementById('panel-close').addEventListener('click', () => this.hide());
        document.getElementById('btn-detail').addEventListener('click', () => this.onDetailClick && this.onDetailClick());
        document.getElementById('btn-history').addEventListener('click', () => this.onHistoryClick && this.onHistoryClick());
        document.getElementById('btn-notify').addEventListener('click', () => this.onNotifyClick && this.onNotifyClick());
    }

    show(turbineId, bladeId, data) {
        this.turbineId = turbineId;
        this.bladeId = bladeId;
        this.data = data || {};
        this.visible = true;
        this.container.classList.add('visible');
        this.updateInfo();
        this.addHistoryPoint(data);
        this.drawHistory();
        this.drawGauges();
    }

    hide() {
        this.visible = false;
        this.container.classList.remove('visible');
    }

    updateData(data) {
        this.data = { ...this.data, ...data };
        if (this.visible) {
            this.updateInfo();
            this.drawGauges();
        }
    }

    addHistoryPoint(data) {
        if (!data) return;
        const ts = Date.now();
        this.history.push({
            ts,
            intensity: data.intensity || 0,
            stage: data.stage || 0,
            damage: data.damage || 0,
            stress: data.stress || 0
        });
        if (this.history.length > 200) this.history.shift();
    }

    updateInfo() {
        const t = this.turbineId + 1;
        const b = this.bladeId + 1;
        const d = this.data;
        document.getElementById('panel-title').textContent = `水轮机 ${t}# 叶片 ${b} 详情`;
        document.getElementById('panel-turbine').textContent = t + '# ' + (['正常','检修中','待维护','紧急'][Math.min(t%4,3)]);
        document.getElementById('panel-blade').textContent = b + ' / 15';
        const stageLabels = ['正常', '初生', '临界', '发展'];
        const badge = document.getElementById('panel-badge');
        badge.textContent = stageLabels[Math.min(d.stage || 0, 3)];
        badge.className = 'badge-stage badge-stage-' + Math.min(d.stage || 0, 3);
        const intensity = (d.intensity || 0) * 100;
        const barI = document.getElementById('panel-intensity-bar');
        barI.style.width = intensity.toFixed(1) + '%';
        barI.className = 'progress-fill ' + (intensity < 30 ? 'progress-low' : intensity < 60 ? 'progress-mid' : intensity < 80 ? 'progress-high' : 'progress-critical');
        document.getElementById('panel-intensity').textContent = intensity.toFixed(1) + '%';
        const damage = (d.damage || 0) * 100;
        const barD = document.getElementById('panel-damage-bar');
        barD.style.width = damage.toFixed(2) + '%';
        barD.className = 'progress-fill ' + (damage < 10 ? 'progress-low' : damage < 30 ? 'progress-mid' : damage < 60 ? 'progress-high' : 'progress-critical');
        document.getElementById('panel-damage').textContent = damage.toFixed(2) + '%';
        document.getElementById('panel-stress').textContent = (d.stress || 0).toFixed(1) + ' MPa';
        const remainingHours = Math.max(0, 200000 - (d.damage || 0) * 200000);
        const days = Math.floor(remainingHours / 24);
        document.getElementById('panel-life').textContent = days > 365 ? (days/365).toFixed(1) + ' 年' : days + ' 天';
    }

    drawHistory() {
        const canvas = document.getElementById('history-chart');
        if (!canvas || !this.history.length) return;
        const ctx = canvas.getContext('2d');
        const w = canvas.width, h = canvas.height;
        ctx.clearRect(0, 0, w, h);
        ctx.fillStyle = 'rgba(15, 23, 42, 0.6)';
        ctx.fillRect(0, 0, w, h);
        const pL = 50, pT = 20, pR = 20, pB = 30;
        const cW = w - pL - pR, cH = h - pT - pB;
        ctx.strokeStyle = 'rgba(148, 163, 184, 0.2)';
        ctx.lineWidth = 1;
        for (let i = 0; i <= 4; i++) {
            const y = pT + cH * (i / 4);
            ctx.beginPath(); ctx.moveTo(pL, y); ctx.lineTo(pL + cW, y); ctx.stroke();
        }
        ctx.fillStyle = '#94a3b8';
        ctx.font = '11px sans-serif';
        ctx.textAlign = 'right';
        for (let i = 0; i <= 4; i++) {
            const v = (1 - i / 4).toFixed(2);
            const y = pT + cH * (i / 4);
            ctx.fillText(v, pL - 5, y + 4);
        }
        const n = Math.max(this.history.length - 60, 0);
        const slice = this.history.slice(n);
        if (slice.length < 2) return;
        const stepX = cW / (slice.length - 1);
        ctx.fillStyle = 'rgba(251, 146, 60, 0.3)';
        ctx.beginPath();
        ctx.moveTo(pL, pT + cH);
        slice.forEach((p, i) => {
            const x = pL + i * stepX;
            const y = pT + cH * (1 - Math.min(p.intensity, 1));
            if (i === 0) ctx.lineTo(x, y);
            else ctx.lineTo(x, y);
        });
        ctx.lineTo(pL + (slice.length - 1) * stepX, pT + cH);
        ctx.closePath();
        ctx.fill();
        ctx.strokeStyle = '#fb923c';
        ctx.lineWidth = 2;
        ctx.beginPath();
        slice.forEach((p, i) => {
            const x = pL + i * stepX;
            const y = pT + cH * (1 - Math.min(p.intensity, 1));
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        });
        ctx.stroke();
        ctx.setLineDash([4, 4]);
        ctx.strokeStyle = 'rgba(250, 204, 21, 0.7)';
        ctx.beginPath();
        const y1 = pT + cH * (1 - 0.3);
        ctx.moveTo(pL, y1); ctx.lineTo(pL + cW, y1); ctx.stroke();
        ctx.strokeStyle = 'rgba(249, 115, 22, 0.7)';
        ctx.beginPath();
        const y2 = pT + cH * (1 - 0.6);
        ctx.moveTo(pL, y2); ctx.lineTo(pL + cW, y2); ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = '#cbd5e1';
        ctx.font = '11px sans-serif';
        ctx.textAlign = 'center';
        const l = slice.length;
        for (let i = 0; i < l; i += Math.max(1, Math.floor(l / 5))) {
            const t = new Date(slice[i].ts);
            const x = pL + i * stepX;
            ctx.fillText(String(t.getHours()).padStart(2, '0') + ':' + String(t.getMinutes()).padStart(2, '0'), x, h - 10);
        }
    }

    drawGauge(canvasId, value, thresholds, label) {
        const canvas = document.getElementById(canvasId);
        if (!canvas) return;
        const ctx = canvas.getContext('2d');
        const w = canvas.width, h = canvas.height;
        ctx.clearRect(0, 0, w, h);
        const cx = w / 2, cy = h * 0.78, r = Math.min(w, h) * 0.55;
        const start = Math.PI * 0.8, end = Math.PI * 2.2;
        const total = end - start;
        let color;
        if (value <= thresholds[0]) color = '#22c55e';
        else if (value <= thresholds[1]) color = '#eab308';
        else if (value <= thresholds[2]) color = '#f97316';
        else color = '#ef4444';
        ctx.lineWidth = 14; ctx.lineCap = 'round';
        ctx.strokeStyle = 'rgba(51, 65, 85, 0.6)';
        ctx.beginPath(); ctx.arc(cx, cy, r, start, end); ctx.stroke();
        ctx.strokeStyle = color;
        const v = Math.max(0, Math.min(value / 100, 1));
        ctx.beginPath(); ctx.arc(cx, cy, r, start, start + total * v); ctx.stroke();
        ctx.lineWidth = 1;
        for (let i = 0; i <= 10; i++) {
            const a = start + total * (i / 10);
            const x1 = cx + Math.cos(a) * (r - 12);
            const y1 = cy + Math.sin(a) * (r - 12);
            const x2 = cx + Math.cos(a) * (r + 12);
            const y2 = cy + Math.sin(a) * (r + 12);
            ctx.strokeStyle = i % 5 === 0 ? 'rgba(148,163,184,0.7)' : 'rgba(148,163,184,0.3)';
            ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
        }
        ctx.fillStyle = '#f8fafc';
        ctx.font = 'bold 28px sans-serif';
        ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
        ctx.fillText(value.toFixed(1), cx, cy - r * 0.15);
        ctx.fillStyle = '#94a3b8';
        ctx.font = '12px sans-serif';
        ctx.fillText(label, cx, cy + r * 0.2);
        ctx.font = '10px sans-serif';
        ctx.fillStyle = 'rgba(148,163,184,0.6)';
        ctx.fillText('0', cx - r * 0.85, cy + r * 0.35);
        ctx.fillText('100', cx + r * 0.85, cy + r * 0.35);
    }

    drawGauges() {
        const damage = (this.data?.damage || 0) * 100;
        const lifeLeft = Math.max(0, Math.min(100 - damage, 100));
        const health = Math.max(0, Math.min(100 - damage * 1.5 - (this.data?.intensity || 0) * 20, 100));
        this.drawGauge('gauge-life', lifeLeft, [70, 40, 20], '剩余寿命');
        this.drawGauge('gauge-health', health, [80, 50, 25], '健康指数');
    }
}
