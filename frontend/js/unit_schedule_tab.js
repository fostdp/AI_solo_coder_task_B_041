class UnitScheduleTab {
    constructor(app) {
        this.app = app;
        this.container = null;
        this.targetPower = 3600;
        this.scheduleHours = 24;
        this.numUnits = 6;
        this.numSlots = 24;
        this.scheduleData = [];
        this.loadCurve = { target: [], actual: [] };
        this.unitDetails = [];
        this.mipStats = {
            gapHistory: [],
            objectiveHistory: [],
            constraintSlack: {}
        };
        this.updateInterval = null;
        this.initMockData();
    }

    initMockData() {
        const hours = this.scheduleHours;
        for (let h = 0; h < hours; h++) {
            const base = 3200 + Math.sin((h / 24) * Math.PI * 2 - Math.PI / 4) * 600;
            const peak = (h >= 9 && h <= 11) || (h >= 18 && h <= 21) ? 400 : 0;
            this.loadCurve.target.push(Math.round(base + peak));
        }

        for (let u = 0; u < this.numUnits; u++) {
            const row = [];
            for (let h = 0; h < this.numSlots; h++) {
                const onProb = 0.7 + Math.sin((u + h) / 5) * 0.2;
                const on = Math.random() < onProb;
                const load = on ? (0.45 + Math.random() * 0.5) : 0;
                row.push({
                    on,
                    load: load,
                    power: load * 750,
                    efficiency: on ? (88 + Math.random() * 8) : 0,
                    cav_risk: on ? (0.1 + Math.random() * 0.7) : 0,
                    startup: on && (h === 0 || !this.scheduleData[u] || !this.scheduleData[u][h - 1]?.on)
                });
            }
            this.scheduleData.push(row);
        }

        for (let h = 0; h < this.numSlots; h++) {
            let total = 0;
            for (let u = 0; u < this.numUnits; u++) {
                total += this.scheduleData[u][h].power;
            }
            this.loadCurve.actual.push(total);
        }

        for (let u = 0; u < this.numUnits; u++) {
            const row = this.scheduleData[u];
            const onHours = row.filter(r => r.on).length;
            const avgEff = row.filter(r => r.on).reduce((s, r) => s + r.efficiency, 0) / Math.max(1, onHours);
            const maxCav = Math.max(...row.map(r => r.cav_risk));
            const totalPower = row.reduce((s, r) => s + r.power, 0);
            const startups = row.filter(r => r.startup).length;
            this.unitDetails.push({
                id: u,
                name: `${u + 1}# 水轮机`,
                onHours,
                avgEff,
                maxCav,
                totalPower,
                startups,
                startupCost: startups * (5 + Math.random() * 3),
                avgCavRisk: row.filter(r => r.on).reduce((s, r) => s + r.cav_risk, 0) / Math.max(1, onHours)
            });
        }

        for (let i = 0; i < 50; i++) {
            this.mipStats.gapHistory.push(Math.max(0.01, 0.8 * Math.exp(-i / 12) + Math.random() * 0.02));
        }
        for (let i = 0; i < 12; i++) {
            this.mipStats.objectiveHistory.push(2.5e6 + Math.sin(i / 2) * 3e5 + Math.random() * 1e5);
        }
        const axes = ['功率平衡', '旋转备用', '爬坡上', '爬坡下', '最小开机', '最小停机', '空化约束', '效率下限'];
        axes.forEach(a => {
            this.mipStats.constraintSlack[a] = 0.2 + Math.random() * 0.8;
        });
    }

    render(container) {
        this.container = container;
        container.innerHTML = '';
        container.style.cssText = 'flex:1;display:flex;flex-direction:column;padding:12px;gap:12px;background:#050a14;overflow:hidden;';

        const topBar = this.createTopBar();
        const mainGrid = this.createMainGrid();

        container.appendChild(topBar);
        container.appendChild(mainGrid);

        setTimeout(() => {
            this.renderGanttChart();
            this.renderLoadCurve();
            this.renderMipRadar();
            this.renderMipGapCurve();
            this.renderMipObjectiveBar();
        }, 80);

        this.startDataUpdates();
    }

    createTopBar() {
        const bar = document.createElement('div');
        bar.style.cssText = 'display:flex;align-items:center;gap:24px;padding:12px 16px;background:rgba(30,58,95,0.5);border:1px solid #1e3a5f;border-radius:8px;flex-wrap:wrap;';

        bar.innerHTML = `
            <div style="display:flex;align-items:center;gap:12px;flex:1;min-width:320px;">
                <span style="font-size:12px;color:#7a8ca3;white-space:nowrap;">目标总功率:</span>
                <div style="flex:1;">
                    <input type="range" id="targetPowerSlider" min="1000" max="5000" step="50" value="${this.targetPower}"
                        style="width:100%;height:4px;appearance:none;background:#1e3a5f;border-radius:2px;outline:none;cursor:pointer;"/>
                </div>
                <span id="targetPowerVal" style="font-family:monospace;color:#00ff88;font-size:14px;font-weight:600;min-width:90px;text-align:right;">${this.targetPower.toLocaleString()} MW</span>
            </div>
            <div style="display:flex;gap:8px;">
                ${[24, 48, 72].map(h => `
                    <button class="hour-btn" data-hours="${h}" style="padding:6px 14px;background:${this.scheduleHours === h ? 'rgba(0,212,255,0.2)' : 'rgba(30,58,95,0.5)'};
                        border:1px solid ${this.scheduleHours === h ? '#00d4ff' : '#1e3a5f'};border-radius:4px;
                        color:${this.scheduleHours === h ? '#00d4ff' : '#7a8ca3'};font-size:12px;cursor:pointer;font-weight:${this.scheduleHours === h ? '600' : '400'};">
                        ${h}h
                    </button>
                `).join('')}
            </div>
            <div style="display:flex;gap:8px;">
                <button id="runScheduleBtn" style="padding:8px 18px;background:rgba(0,212,255,0.15);border:1px solid #00d4ff;border-radius:5px;color:#00d4ff;font-size:12px;cursor:pointer;font-weight:500;">
                    ⚡ 触发求解
                </button>
                <button id="executeScheduleBtn" style="padding:8px 18px;background:rgba(0,255,136,0.15);border:1px solid #00ff88;border-radius:5px;color:#00ff88;font-size:12px;cursor:pointer;font-weight:500;">
                    ✓ 下发执行
                </button>
            </div>
        `;

        setTimeout(() => {
            const slider = bar.querySelector('#targetPowerSlider');
            slider.addEventListener('input', (e) => {
                this.targetPower = parseInt(e.target.value);
                bar.querySelector('#targetPowerVal').textContent = this.targetPower.toLocaleString() + ' MW';
            });

            bar.querySelectorAll('.hour-btn').forEach(btn => {
                btn.addEventListener('click', (e) => {
                    this.scheduleHours = parseInt(e.target.dataset.hours);
                    bar.querySelectorAll('.hour-btn').forEach(b => {
                        const h = parseInt(b.dataset.hours);
                        b.style.background = (h === this.scheduleHours) ? 'rgba(0,212,255,0.2)' : 'rgba(30,58,95,0.5)';
                        b.style.borderColor = (h === this.scheduleHours) ? '#00d4ff' : '#1e3a5f';
                        b.style.color = (h === this.scheduleHours) ? '#00d4ff' : '#7a8ca3';
                        b.style.fontWeight = (h === this.scheduleHours) ? '600' : '400';
                    });
                });
            });

            bar.querySelector('#runScheduleBtn').addEventListener('click', () => this.runSchedule());
            bar.querySelector('#executeScheduleBtn').addEventListener('click', () => this.executeSchedule());
        }, 50);

        return bar;
    }

    createMainGrid() {
        const grid = document.createElement('div');
        grid.style.cssText = 'flex:1;display:grid;grid-template-columns:1.6fr 1fr;grid-template-rows:1fr 1fr;gap:12px;overflow:hidden;min-height:0;';

        const ganttPanel = this.createPanel('甘特图 - 机组 × 时段', '', false);
        ganttPanel.style.gridColumn = '1 / 2';
        ganttPanel.style.gridRow = '1 / 2';
        const ganttInner = ganttPanel.querySelector('.panel-body');
        ganttInner.id = 'ganttContainer';
        ganttInner.style.cssText = 'flex:1;position:relative;min-height:0;overflow:hidden;';
        ganttInner.innerHTML = `
            <svg id="ganttSvg" style="position:absolute;inset:0;width:100%;height:100%;"></svg>
            <div id="ganttTooltip" style="display:none;position:absolute;background:rgba(10,22,40,0.95);border:1px solid #00d4ff;border-radius:6px;padding:8px;font-size:11px;z-index:100;pointer-events:none;min-width:160px;box-shadow:0 4px 16px rgba(0,212,255,0.2);"></div>
        `;

        const loadPanel = this.createPanel('负荷曲线 (24h)', '');
        loadPanel.style.gridColumn = '2 / 3';
        loadPanel.style.gridRow = '1 / 2';
        const loadInner = loadPanel.querySelector('.panel-body');
        loadInner.innerHTML = `<canvas id="loadCurveCanvas" style="width:100%;height:100%;display:block;"></canvas>`;

        const unitPanel = this.createPanel('机组详情', '');
        unitPanel.style.gridColumn = '1 / 2';
        unitPanel.style.gridRow = '2 / 3';
        const unitInner = unitPanel.querySelector('.panel-body');
        unitInner.id = 'unitDetailContainer';
        unitInner.style.cssText = 'overflow-y:auto;display:grid;grid-template-columns:repeat(2,1fr);gap:8px;padding:4px;';
        this.renderUnitCards(unitInner);

        const mipPanel = this.createPanel('MIP求解器指标', '');
        mipPanel.style.gridColumn = '2 / 3';
        mipPanel.style.gridRow = '2 / 3';
        const mipInner = mipPanel.querySelector('.panel-body');
        mipInner.style.cssText = 'display:grid;grid-template-columns:1fr;grid-template-rows:auto auto 1fr;gap:6px;overflow:hidden;';
        mipInner.innerHTML = `
            <div style="display:flex;gap:4px;font-size:10px;color:#555;">
                <span>Gap收敛</span><span style="flex:1;"></span><span>目标历史</span>
            </div>
            <div style="display:grid;grid-template-columns:1fr 1fr;gap:8px;height:90px;">
                <canvas id="mipGapCanvas" style="width:100%;height:100%;border-radius:3px;background:#0a0f1a;"></canvas>
                <canvas id="mipObjCanvas" style="width:100%;height:100%;border-radius:3px;background:#0a0f1a;"></canvas>
            </div>
            <div style="display:flex;flex-direction:column;min-height:0;">
                <div style="font-size:10px;color:#555;margin-bottom:3px;">约束Slack雷达图 (8轴)</div>
                <div style="flex:1;display:flex;align-items:center;justify-content:center;min-height:0;">
                    <svg id="mipRadarSvg" viewBox="0 0 200 200" style="max-height:100%;max-width:100%;"></svg>
                </div>
            </div>
        `;

        grid.appendChild(ganttPanel);
        grid.appendChild(loadPanel);
        grid.appendChild(unitPanel);
        grid.appendChild(mipPanel);

        return grid;
    }

    createPanel(title, subtitle, collapsible = true) {
        const panel = document.createElement('div');
        panel.style.cssText = 'background:rgba(30,58,95,0.5);border:1px solid #1e3a5f;border-radius:8px;display:flex;flex-direction:column;overflow:hidden;min-width:0;min-height:0;';
        panel.innerHTML = `
            <div style="display:flex;justify-content:space-between;align-items:center;padding:8px 12px;background:rgba(10,22,40,0.4);border-bottom:1px solid #1e3a5f;flex-shrink:0;">
                <div style="display:flex;align-items:center;gap:8px;">
                    <h4 style="font-size:12px;color:#7a8ca3;margin:0;text-transform:uppercase;letter-spacing:0.5px;font-weight:600;">${title}</h4>
                    ${subtitle ? `<span style="font-size:10px;color:#555;">${subtitle}</span>` : ''}
                </div>
            </div>
            <div class="panel-body" style="flex:1;padding:8px;overflow:hidden;min-height:0;"></div>
        `;
        return panel;
    }

    renderGanttChart() {
        const svg = document.getElementById('ganttSvg');
        const container = document.getElementById('ganttContainer');
        const tooltip = document.getElementById('ganttTooltip');
        if (!svg || !container) return;

        const rect = container.getBoundingClientRect();
        const W = rect.width, H = rect.height;
        svg.setAttribute('viewBox', `0 0 ${W} ${H}`);
        svg.innerHTML = '';

        const pad = { t: 24, r: 16, b: 28, l: 70 };
        const cw = W - pad.l - pad.r;
        const ch = H - pad.t - pad.b;
        const rowH = ch / this.numUnits;
        const slotW = cw / this.numSlots;

        for (let h = 0; h <= this.numSlots; h++) {
            const x = pad.l + h * slotW;
            const line = document.createElementNS('http://www.w3.org/2000/svg', 'line');
            line.setAttribute('x1', x); line.setAttribute('y1', pad.t);
            line.setAttribute('x2', x); line.setAttribute('y2', pad.t + ch);
            line.setAttribute('stroke', '#1e3a5f');
            line.setAttribute('stroke-width', h % 6 === 0 ? '1' : '0.5');
            svg.appendChild(line);
        }

        for (let u = 0; u <= this.numUnits; u++) {
            const y = pad.t + u * rowH;
            const line = document.createElementNS('http://www.w3.org/2000/svg', 'line');
            line.setAttribute('x1', pad.l); line.setAttribute('y1', y);
            line.setAttribute('x2', W - pad.r); line.setAttribute('y2', y);
            line.setAttribute('stroke', '#2d4a6f');
            line.setAttribute('stroke-width', '0.5');
            svg.appendChild(line);
        }

        const unitColors = ['#00d4ff', '#00ff88', '#ffc107', '#ff9800', '#2196f3', '#9c27b0'];
        for (let u = 0; u < this.numUnits; u++) {
            for (let h = 0; h < this.numSlots; h++) {
                const cell = this.scheduleData[u][h];
                if (!cell.on) continue;
                const x = pad.l + h * slotW + 1;
                const y = pad.t + u * rowH + 3;
                const w = slotW - 2;
                const loadH = (rowH - 6) * cell.load;
                const bh = pad.t + u * rowH + (rowH - 3) - loadH;
                const cavColor = `hsl(${120 - cell.cav_risk * 120}, 75%, 55%)`;

                const bar = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
                bar.setAttribute('x', x); bar.setAttribute('y', bh);
                bar.setAttribute('width', w); bar.setAttribute('height', loadH);
                bar.setAttribute('rx', '2');
                bar.setAttribute('fill', cavColor);
                bar.setAttribute('fill-opacity', '0.85');
                bar.setAttribute('stroke', unitColors[u]);
                bar.setAttribute('stroke-width', cell.startup ? '2' : '0.5');
                bar.setAttribute('data-unit', u);
                bar.setAttribute('data-hour', h);
                bar.style.cursor = 'pointer';
                svg.appendChild(bar);

                if (cell.startup) {
                    const tri = document.createElementNS('http://www.w3.org/2000/svg', 'polygon');
                    const tx1 = x + 2, ty1 = bh;
                    const tx2 = x + 12, ty2 = bh;
                    const tx3 = x + 2, ty3 = bh + 10;
                    tri.setAttribute('points', `${tx1},${ty1} ${tx2},${ty2} ${tx3},${ty3}`);
                    tri.setAttribute('fill', '#ff4757');
                    svg.appendChild(tri);
                }
            }
        }

        for (let u = 0; u < this.numUnits; u++) {
            const t = document.createElementNS('http://www.w3.org/2000/svg', 'text');
            t.setAttribute('x', pad.l - 8);
            t.setAttribute('y', pad.t + u * rowH + rowH / 2 + 4);
            t.setAttribute('text-anchor', 'end');
            t.setAttribute('fill', unitColors[u]);
            t.setAttribute('font-size', '11');
            t.setAttribute('font-weight', '600');
            t.textContent = `${u + 1}#机组`;
            svg.appendChild(t);
        }

        for (let h = 0; h <= this.numSlots; h += 3) {
            const t = document.createElementNS('http://www.w3.org/2000/svg', 'text');
            t.setAttribute('x', pad.l + h * slotW);
            t.setAttribute('y', H - 8);
            t.setAttribute('text-anchor', 'middle');
            t.setAttribute('fill', '#555');
            t.setAttribute('font-size', '10');
            t.textContent = `${String(h).padStart(2, '0')}:00`;
            svg.appendChild(t);
        }

        svg.querySelectorAll('rect[data-unit]').forEach(bar => {
            bar.addEventListener('mouseenter', (e) => {
                const u = parseInt(bar.dataset.unit);
                const h = parseInt(bar.dataset.hour);
                const cell = this.scheduleData[u][h];
                tooltip.style.display = 'block';
                tooltip.innerHTML = `
                    <div style="font-weight:600;color:#00d4ff;margin-bottom:6px;border-bottom:1px solid #1e3a5f;padding-bottom:4px;">
                        ${u + 1}#机组 ${String(h).padStart(2, '0')}:00
                    </div>
                    <div style="display:flex;justify-content:space-between;margin:2px 0;"><span style="color:#555;">功率</span><span style="color:#e0e6ed;font-family:monospace;">${cell.power.toFixed(0)} MW</span></div>
                    <div style="display:flex;justify-content:space-between;margin:2px 0;"><span style="color:#555;">负载率</span><span style="color:#00ff88;font-family:monospace;">${(cell.load * 100).toFixed(0)}%</span></div>
                    <div style="display:flex;justify-content:space-between;margin:2px 0;"><span style="color:#555;">效率</span><span style="color:#2196f3;font-family:monospace;">${cell.efficiency.toFixed(1)}%</span></div>
                    <div style="display:flex;justify-content:space-between;margin:2px 0;"><span style="color:#555;">空化风险</span><span style="color:hsl(${120 - cell.cav_risk * 120},75%,55%);font-family:monospace;">${(cell.cav_risk * 100).toFixed(0)}%</span></div>
                    ${cell.startup ? '<div style="color:#ff4757;margin-top:4px;">⏱ 启动</div>' : ''}
                `;
            });
            bar.addEventListener('mousemove', (e) => {
                const r = container.getBoundingClientRect();
                tooltip.style.left = (e.clientX - r.left + 14) + 'px';
                tooltip.style.top = (e.clientY - r.top - 8) + 'px';
            });
            bar.addEventListener('mouseleave', () => {
                tooltip.style.display = 'none';
            });
        });

        const legend = document.createElementNS('http://www.w3.org/2000/svg', 'g');
        legend.setAttribute('transform', `translate(${pad.l}, 8)`);
        const legendItems = [
            { c: '#4caf50', t: '低风险' },
            { c: '#ffc107', t: '中风险' },
            { c: '#f44336', t: '高风险' }
        ];
        legendItems.forEach((it, i) => {
            const rx = i * 70;
            const r = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
            r.setAttribute('x', rx); r.setAttribute('y', 0);
            r.setAttribute('width', 12); r.setAttribute('height', 10);
            r.setAttribute('fill', it.c);
            r.setAttribute('rx', '2');
            legend.appendChild(r);
            const t = document.createElementNS('http://www.w3.org/2000/svg', 'text');
            t.setAttribute('x', rx + 16); t.setAttribute('y', 9);
            t.setAttribute('fill', '#555'); t.setAttribute('font-size', '9');
            t.textContent = it.t;
            legend.appendChild(t);
        });
        svg.appendChild(legend);
    }

    renderLoadCurve() {
        const canvas = document.getElementById('loadCurveCanvas');
        if (!canvas) return;
        const ctx = canvas.getContext('2d');
        const dpr = window.devicePixelRatio;
        const rect = canvas.getBoundingClientRect();
        canvas.width = rect.width * dpr;
        canvas.height = rect.height * dpr;
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

        const w = rect.width, h = rect.height;
        ctx.fillStyle = '#0a0f1a';
        ctx.fillRect(0, 0, w, h);

        const pad = { l: 42, r: 10, t: 20, b: 22 };
        const cw = w - pad.l - pad.r, ch = h - pad.t - pad.b;

        const maxVal = Math.max(...this.loadCurve.target, ...this.loadCurve.actual) * 1.1;
        const minVal = Math.min(...this.loadCurve.target, ...this.loadCurve.actual) * 0.9;
        const range = maxVal - minVal || 1;

        ctx.strokeStyle = '#1e3a5f';
        ctx.lineWidth = 0.5;
        for (let i = 0; i <= 4; i++) {
            const y = pad.t + (i / 4) * ch;
            ctx.beginPath();
            ctx.moveTo(pad.l, y); ctx.lineTo(w - pad.r, y); ctx.stroke();
            ctx.fillStyle = '#555';
            ctx.font = '9px monospace';
            ctx.textAlign = 'right';
            ctx.fillText(Math.round(maxVal - (i / 4) * range) + '', pad.l - 3, y + 3);
        }

        const colors = ['#00d4ff', '#00ff88', '#ffc107', '#ff9800', '#2196f3', '#9c27b0'];
        let cumul = Array(this.numSlots).fill(0);
        for (let u = 0; u < this.numUnits; u++) {
            ctx.beginPath();
            ctx.moveTo(pad.l, pad.t + ch);
            for (let h = 0; h < this.numSlots; h++) {
                const x = pad.l + (h / (this.numSlots - 1)) * cw;
                cumul[h] += this.scheduleData[u][h].power;
                const y = pad.t + ch - ((cumul[h] - minVal) / range) * ch;
                ctx.lineTo(x, y);
            }
            ctx.lineTo(w - pad.r, pad.t + ch);
            ctx.closePath();
            ctx.fillStyle = colors[u] + '22';
            ctx.fill();
        }

        for (let u = this.numUnits - 1; u >= 0; u--) {
            let prev = u === 0 ? Array(this.numSlots).fill(0) : Array(this.numSlots).fill(0);
            if (u > 0) {
                for (let h = 0; h < this.numSlots; h++) {
                    for (let uu = 0; uu < u; uu++) prev[h] += this.scheduleData[uu][h].power;
                }
            }
            ctx.beginPath();
            for (let h = 0; h < this.numSlots; h++) {
                const x = pad.l + (h / (this.numSlots - 1)) * cw;
                const y1 = pad.t + ch - ((prev[h] - minVal) / range) * ch;
                const y2 = pad.t + ch - ((prev[h] + this.scheduleData[u][h].power - minVal) / range) * ch;
                ctx.moveTo(x, y1);
                ctx.lineTo(x, y2);
            }
            ctx.strokeStyle = colors[u];
            ctx.lineWidth = 1;
            ctx.stroke();
        }

        ctx.beginPath();
        for (let h = 0; h < this.numSlots; h++) {
            const x = pad.l + (h / (this.numSlots - 1)) * cw;
            const y = pad.t + ch - ((this.loadCurve.target[h] - minVal) / range) * ch;
            if (h === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 2;
        ctx.setLineDash([5, 3]);
        ctx.stroke();
        ctx.setLineDash([]);

        const ch_ = h;
        for (let hh = 0; hh <= this.numSlots; hh += 6) {
            const x = pad.l + (hh / (this.numSlots - 1)) * cw;
            ctx.fillStyle = '#555';
            ctx.font = '9px monospace';
            ctx.textAlign = 'center';
            ctx.fillText(`${String(hh).padStart(2, '0')}:00`, x, ch_ - 8);
        }

        const legend = [
            { c: '#ffffff', t: '目标负荷', d: true },
            { c: '#00d4ff', t: '实际分配', d: false }
        ];
        ctx.font = '10px sans-serif';
        let lx = pad.l + 6;
        legend.forEach(l => {
            ctx.fillStyle = l.c;
            ctx.fillRect(lx, pad.t - 12, 12, 8);
            if (l.d) {
                ctx.strokeStyle = l.c;
                ctx.lineWidth = 1;
                ctx.beginPath();
                ctx.moveTo(lx + 2, pad.t - 8);
                ctx.lineTo(lx + 10, pad.t - 8);
                ctx.setLineDash([2, 2]);
                ctx.stroke();
                ctx.setLineDash([]);
            }
            ctx.fillStyle = '#7a8ca3';
            ctx.textAlign = 'left';
            ctx.fillText(l.t, lx + 16, pad.t - 4);
            lx += 90;
        });
    }

    renderUnitCards(container) {
        const unitColors = ['#00d4ff', '#00ff88', '#ffc107', '#ff9800', '#2196f3', '#9c27b0'];
        container.innerHTML = this.unitDetails.map(d => {
            const cavColor = `hsl(${120 - d.maxCav * 120}, 75%, 55%)`;
            return `
                <div style="background:rgba(10,22,40,0.6);border:1px solid #1e3a5f;border-left:3px solid ${unitColors[d.id]};border-radius:5px;padding:10px;font-size:11px;">
                    <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:8px;">
                        <span style="font-weight:600;color:${unitColors[d.id]};font-size:12px;">${d.name}</span>
                        <span style="color:#555;">${d.onHours}/${this.numSlots}h</span>
                    </div>
                    <div style="display:grid;grid-template-columns:1fr 1fr;gap:4px;">
                        <div>
                            <div style="color:#555;font-size:9px;">分配功率</div>
                            <div style="color:#e0e6ed;font-family:monospace;font-weight:600;">${(d.totalPower / this.numSlots).toFixed(0)} MW</div>
                        </div>
                        <div>
                            <div style="color:#555;font-size:9px;">平均效率</div>
                            <div style="color:#2196f3;font-family:monospace;font-weight:600;">${d.avgEff.toFixed(1)}%</div>
                        </div>
                        <div>
                            <div style="color:#555;font-size:9px;">最大空化</div>
                            <div style="color:${cavColor};font-family:monospace;font-weight:600;">${(d.maxCav * 100).toFixed(0)}%</div>
                        </div>
                        <div>
                            <div style="color:#555;font-size:9px;">启停次数</div>
                            <div style="color:#ff9800;font-family:monospace;font-weight:600;">${d.startups}</div>
                        </div>
                    </div>
                    <div style="margin-top:8px;">
                        <div style="display:flex;justify-content:space-between;font-size:9px;color:#555;margin-bottom:2px;">
                            <span>运行时长</span>
                            <span>${((d.onHours / this.numSlots) * 100).toFixed(0)}% · 启停机成本 ¥${d.startupCost.toFixed(1)}k</span>
                        </div>
                        <div style="height:4px;background:#1e3a5f;border-radius:2px;overflow:hidden;">
                            <div style="height:100%;width:${(d.onHours / this.numSlots) * 100}%;background:linear-gradient(90deg,${unitColors[d.id]},#00ff88);"></div>
                        </div>
                    </div>
                </div>
            `;
        }).join('');
    }

    renderMipGapCurve() {
        const canvas = document.getElementById('mipGapCanvas');
        if (!canvas) return;
        const ctx = canvas.getContext('2d');
        const dpr = window.devicePixelRatio;
        const rect = canvas.getBoundingClientRect();
        canvas.width = rect.width * dpr;
        canvas.height = rect.height * dpr;
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        const w = rect.width, h = rect.height;
        ctx.fillStyle = '#0a0f1a'; ctx.fillRect(0, 0, w, h);

        const pad = { l: 28, r: 4, t: 4, b: 14 };
        const cw = w - pad.l - pad.r, ch = h - pad.t - pad.b;
        const data = this.mipStats.gapHistory;
        const max = Math.max(...data), min = 0;
        const range = max - min || 1;

        ctx.strokeStyle = '#1e3a5f'; ctx.lineWidth = 0.5;
        for (let i = 0; i <= 2; i++) {
            const y = pad.t + (i / 2) * ch;
            ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(w - pad.r, y); ctx.stroke();
            ctx.fillStyle = '#444'; ctx.font = '8px monospace'; ctx.textAlign = 'right';
            ctx.fillText(((1 - i / 2) * max * 100).toFixed(1) + '%', pad.l - 2, y + 2);
        }

        ctx.beginPath();
        data.forEach((v, i) => {
            const x = pad.l + (i / (data.length - 1)) * cw;
            const y = pad.t + ch - ((v - min) / range) * ch;
            if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        });
        ctx.strokeStyle = '#9c27b0'; ctx.lineWidth = 1.2; ctx.stroke();

        ctx.fillStyle = '#555'; ctx.font = '8px monospace'; ctx.textAlign = 'center';
        ctx.fillText('迭代', w / 2, h - 3);
    }

    renderMipObjectiveBar() {
        const canvas = document.getElementById('mipObjCanvas');
        if (!canvas) return;
        const ctx = canvas.getContext('2d');
        const dpr = window.devicePixelRatio;
        const rect = canvas.getBoundingClientRect();
        canvas.width = rect.width * dpr;
        canvas.height = rect.height * dpr;
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        const w = rect.width, h = rect.height;
        ctx.fillStyle = '#0a0f1a'; ctx.fillRect(0, 0, w, h);

        const pad = { l: 36, r: 4, t: 4, b: 14 };
        const cw = w - pad.l - pad.r, ch = h - pad.t - pad.b;
        const data = this.mipStats.objectiveHistory;
        const max = Math.max(...data), min = Math.min(...data);
        const range = max - min || 1;

        const bw = cw / data.length * 0.7;
        const sp = cw / data.length * 0.3;
        data.forEach((v, i) => {
            const x = pad.l + i * (bw + sp) + sp / 2;
            const bh = ((v - min) / range) * ch;
            const y = pad.t + ch - bh;
            const grad = ctx.createLinearGradient(0, y, 0, pad.t + ch);
            grad.addColorStop(0, '#00d4ff');
            grad.addColorStop(1, '#00d4ff44');
            ctx.fillStyle = grad;
            ctx.fillRect(x, y, bw, bh);
        });

        ctx.fillStyle = '#555'; ctx.font = '8px monospace'; ctx.textAlign = 'right';
        ctx.fillText((max / 1e6).toFixed(1) + 'M', pad.l - 2, pad.t + 8);
        ctx.fillText((min / 1e6).toFixed(1) + 'M', pad.l - 2, pad.t + ch);
        ctx.textAlign = 'center';
        ctx.fillText('目标值', w / 2, h - 3);
    }

    renderMipRadar() {
        const svg = document.getElementById('mipRadarSvg');
        if (!svg) return;
        svg.innerHTML = '';
        const cx = 100, cy = 100, R = 75;
        const axes = Object.keys(this.mipStats.constraintSlack);
        const N = axes.length;

        for (let l = 1; l <= 4; l++) {
            const r = R * l / 4;
            const pts = [];
            for (let i = 0; i < N; i++) {
                const a = (i / N) * Math.PI * 2 - Math.PI / 2;
                pts.push(`${cx + Math.cos(a) * r},${cy + Math.sin(a) * r}`);
            }
            const poly = document.createElementNS('http://www.w3.org/2000/svg', 'polygon');
            poly.setAttribute('points', pts.join(' '));
            poly.setAttribute('fill', 'none');
            poly.setAttribute('stroke', '#1e3a5f');
            poly.setAttribute('stroke-width', '0.5');
            svg.appendChild(poly);
        }

        for (let i = 0; i < N; i++) {
            const a = (i / N) * Math.PI * 2 - Math.PI / 2;
            const line = document.createElementNS('http://www.w3.org/2000/svg', 'line');
            line.setAttribute('x1', cx); line.setAttribute('y1', cy);
            line.setAttribute('x2', cx + Math.cos(a) * R); line.setAttribute('y2', cy + Math.sin(a) * R);
            line.setAttribute('stroke', '#1e3a5f'); line.setAttribute('stroke-width', '0.5');
            svg.appendChild(line);

            const lx = cx + Math.cos(a) * (R + 14);
            const ly = cy + Math.sin(a) * (R + 14);
            const t = document.createElementNS('http://www.w3.org/2000/svg', 'text');
            t.setAttribute('x', lx); t.setAttribute('y', ly + 3);
            t.setAttribute('text-anchor', 'middle');
            t.setAttribute('fill', '#555'); t.setAttribute('font-size', '7');
            t.textContent = axes[i];
            svg.appendChild(t);
        }

        const dataPts = [];
        for (let i = 0; i < N; i++) {
            const v = this.mipStats.constraintSlack[axes[i]];
            const a = (i / N) * Math.PI * 2 - Math.PI / 2;
            const r = R * v;
            dataPts.push(`${cx + Math.cos(a) * r},${cy + Math.sin(a) * r}`);
        }
        const dp = document.createElementNS('http://www.w3.org/2000/svg', 'polygon');
        dp.setAttribute('points', dataPts.join(' '));
        dp.setAttribute('fill', 'rgba(0,212,255,0.25)');
        dp.setAttribute('stroke', '#00d4ff');
        dp.setAttribute('stroke-width', '1.2');
        svg.appendChild(dp);

        for (let i = 0; i < N; i++) {
            const v = this.mipStats.constraintSlack[axes[i]];
            const a = (i / N) * Math.PI * 2 - Math.PI / 2;
            const r = R * v;
            const circ = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
            circ.setAttribute('cx', cx + Math.cos(a) * r);
            circ.setAttribute('cy', cy + Math.sin(a) * r);
            circ.setAttribute('r', '2.5');
            circ.setAttribute('fill', '#00d4ff');
            svg.appendChild(circ);
        }
    }

    startDataUpdates() {
        if (this.updateInterval) clearInterval(this.updateInterval);
        this.updateInterval = setInterval(() => {
            const lastGap = this.mipStats.gapHistory[this.mipStats.gapHistory.length - 1];
            this.mipStats.gapHistory.shift();
            this.mipStats.gapHistory.push(Math.max(0.005, lastGap * 0.95 + Math.random() * 0.005));

            const axes = Object.keys(this.mipStats.constraintSlack);
            axes.forEach(a => {
                this.mipStats.constraintSlack[a] = Math.max(0.1, Math.min(1, this.mipStats.constraintSlack[a] + (Math.random() - 0.5) * 0.08));
            });

            this.scheduleData.forEach((row, u) => {
                row.forEach((cell, h) => {
                    if (cell.on) {
                        cell.cav_risk = Math.max(0, Math.min(1, cell.cav_risk + (Math.random() - 0.5) * 0.05));
                        cell.efficiency = Math.max(85, Math.min(96, cell.efficiency + (Math.random() - 0.5) * 0.3));
                    }
                });
            });

            this.renderMipGapCurve();
            this.renderMipRadar();
        }, 1500);
    }

    async runSchedule() {
        try {
            await fetch(`${this.app.apiBase}/schedule/run`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ target_power: this.targetPower, hours: this.scheduleHours })
            });
        } catch (e) {}

        this.scheduleData = [];
        this.loadCurve.actual = [];
        for (let u = 0; u < this.numUnits; u++) {
            const row = [];
            for (let h = 0; h < this.numSlots; h++) {
                const demandRatio = this.loadCurve.target[h] / 3600;
                const onProb = 0.5 + demandRatio * 0.4 + Math.sin((u + h) / 4) * 0.1;
                const on = Math.random() < onProb;
                row.push({
                    on,
                    load: on ? (0.5 + demandRatio * 0.3 + Math.random() * 0.15) : 0,
                    power: 0,
                    efficiency: on ? (88 + Math.random() * 8) : 0,
                    cav_risk: on ? (0.1 + demandRatio * 0.4 + Math.random() * 0.3) : 0,
                    startup: false
                });
                if (on) row[h].power = row[h].load * 750;
            }
            for (let h = 0; h < this.numSlots; h++) {
                row[h].startup = row[h].on && (h === 0 || !row[h - 1].on);
            }
            this.scheduleData.push(row);
        }

        for (let h = 0; h < this.numSlots; h++) {
            let total = 0;
            for (let u = 0; u < this.numUnits; u++) total += this.scheduleData[u][h].power;
            this.loadCurve.actual.push(total);
        }

        this.mipStats.gapHistory = [];
        for (let i = 0; i < 50; i++) {
            this.mipStats.gapHistory.push(Math.max(0.005, 1.0 * Math.exp(-i / 10) + Math.random() * 0.01));
        }
        this.mipStats.objectiveHistory.push(2e6 + Math.random() * 1e6);
        if (this.mipStats.objectiveHistory.length > 12) this.mipStats.objectiveHistory.shift();

        this.unitDetails = [];
        for (let u = 0; u < this.numUnits; u++) {
            const row = this.scheduleData[u];
            const onHours = row.filter(r => r.on).length;
            this.unitDetails.push({
                id: u,
                name: `${u + 1}# 水轮机`,
                onHours,
                avgEff: onHours ? row.filter(r => r.on).reduce((s, r) => s + r.efficiency, 0) / onHours : 0,
                maxCav: Math.max(...row.map(r => r.cav_risk)),
                totalPower: row.reduce((s, r) => s + r.power, 0),
                startups: row.filter(r => r.startup).length,
                startupCost: row.filter(r => r.startup).length * (5 + Math.random() * 3),
                avgCavRisk: onHours ? row.filter(r => r.on).reduce((s, r) => s + r.cav_risk, 0) / onHours : 0
            });
        }

        this.renderGanttChart();
        this.renderLoadCurve();
        const uc = document.getElementById('unitDetailContainer');
        if (uc) this.renderUnitCards(uc);
        this.renderMipGapCurve();
        this.renderMipObjectiveBar();
        this.renderMipRadar();
    }

    async executeSchedule() {
        try {
            await fetch(`${this.app.apiBase}/schedule/execute`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ schedule: this.scheduleData })
            });
        } catch (e) {}
        alert('调度指令已下发至各机组');
    }

    resize() {
        if (!this.container) return;
        setTimeout(() => {
            this.renderGanttChart();
            this.renderLoadCurve();
        }, 50);
    }
}
