class MPCControlTab {
    constructor(app) {
        this.app = app;
        this.container = null;
        this.units = [];
        this.mpcWeights = { W_eff: 0.4, W_cav: 0.3, W_power: 0.2, W_du: 0.1 };
        this.avoidCavitation = true;
        this.objectiveHistory = [];
        this.controlIncrements = [];
        this.predictionHorizon = 20;
        this.predictionStates = [];
        this.updateInterval = null;
        this.guideVaneRotation = 0;
        this.initMockData();
    }

    initMockData() {
        const modes = ['manual', 'efficiency', 'avoid_cav', 'mpc_optimal'];
        const modeNames = ['手动', '效率优先', '避空化', 'MPC最优'];
        for (let i = 0; i < 6; i++) {
            this.units.push({
                id: i,
                name: `${i + 1}#机组`,
                guide_vane: 30 + Math.random() * 40,
                power: 500 + Math.random() * 300,
                target_power: 600 + Math.random() * 200,
                mode: modes[Math.floor(Math.random() * 4)],
                mode_name: modeNames[Math.floor(Math.random() * 4)],
                efficiency_pred: 85 + Math.random() * 10,
                cav_risk_pred: Math.random() * 0.8
            });
        }
        for (let i = 0; i < 60; i++) {
            this.objectiveHistory.push(50 + Math.random() * 30);
        }
        for (let i = 0; i < 6; i++) {
            this.controlIncrements.push((Math.random() - 0.5) * 10);
        }
        for (let i = 0; i < this.predictionHorizon; i++) {
            this.predictionStates.push({
                step: i + 1,
                power: 650 + Math.sin(i / 3) * 80 + Math.random() * 20,
                cav_risk: 0.3 + Math.sin(i / 5) * 0.2 + Math.random() * 0.1
            });
        }
    }

    render(container) {
        this.container = container;
        container.innerHTML = '';
        container.style.cssText = 'flex:1;display:flex;flex-direction:column;padding:12px;gap:12px;background:#050a14;overflow:hidden;';

        const mainRow = document.createElement('div');
        mainRow.style.cssText = 'flex:1;display:flex;gap:12px;overflow:hidden;min-height:0;';

        const leftPanel = this.createLeftPanel();
        const centerPanel = this.createCenterPanel();
        const rightPanel = this.createRightPanel();

        mainRow.appendChild(leftPanel);
        mainRow.appendChild(centerPanel);
        mainRow.appendChild(rightPanel);

        const bottomPanel = this.createBottomPanel();

        container.appendChild(mainRow);
        container.appendChild(bottomPanel);

        this.renderObjectiveChart();
        this.renderControlIncrements();
        this.renderPredictionStates();
        this.startAnimations();
    }

    createLeftPanel() {
        const panel = document.createElement('div');
        panel.style.cssText = 'width:340px;display:flex;flex-direction:column;gap:8px;overflow-y:auto;padding-right:4px;';
        panel.innerHTML = `<h3 style="font-size:14px;color:#7a8ca3;margin:0 0 4px 4px;text-transform:uppercase;letter-spacing:0.5px;">机组控制面板</h3>`;

        this.units.forEach((unit, idx) => {
            const card = document.createElement('div');
            card.style.cssText = 'background:rgba(30,58,95,0.5);border:1px solid #1e3a5f;border-radius:8px;padding:12px;';

            const modeColors = {
                'manual': '#ff9800',
                'efficiency': '#4caf50',
                'avoid_cav': '#2196f3',
                'mpc_optimal': '#00d4ff'
            };

            card.innerHTML = `
                <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:10px;">
                    <span style="font-weight:600;color:#e0e6ed;font-size:13px;">${unit.name}</span>
                    <span style="font-size:10px;padding:2px 8px;border-radius:10px;background:${modeColors[unit.mode]}22;color:${modeColors[unit.mode]};">${unit.mode_name}</span>
                </div>
                <div style="margin-bottom:8px;">
                    <div style="display:flex;justify-content:space-between;font-size:11px;color:#7a8ca3;margin-bottom:3px;">
                        <span>导叶开度</span><span style="color:#00d4ff;font-family:monospace;">${unit.guide_vane.toFixed(1)}%</span>
                    </div>
                    <input type="range" min="0" max="100" step="0.1" value="${unit.guide_vane}" 
                        data-idx="${idx}" data-field="guide_vane"
                        style="width:100%;height:4px;appearance:none;background:#1e3a5f;border-radius:2px;outline:none;cursor:pointer;"/>
                </div>
                <div style="margin-bottom:8px;">
                    <div style="display:flex;justify-content:space-between;font-size:11px;color:#7a8ca3;margin-bottom:3px;">
                        <span>目标功率</span><span style="color:#00ff88;font-family:monospace;">${unit.target_power.toFixed(0)} MW</span>
                    </div>
                    <input type="range" min="0" max="1000" step="1" value="${unit.target_power}" 
                        data-idx="${idx}" data-field="target_power"
                        style="width:100%;height:4px;appearance:none;background:#1e3a5f;border-radius:2px;outline:none;cursor:pointer;"/>
                </div>
                <div style="margin-bottom:10px;">
                    <div style="font-size:11px;color:#7a8ca3;margin-bottom:3px;">控制模式</div>
                    <select data-idx="${idx}" data-field="mode"
                        style="width:100%;padding:6px;background:rgba(10,22,40,0.8);border:1px solid #1e3a5f;border-radius:4px;color:#e0e6ed;font-size:12px;cursor:pointer;">
                        <option value="manual">手动</option>
                        <option value="efficiency">效率优先</option>
                        <option value="avoid_cav">避空化</option>
                        <option value="mpc_optimal">MPC最优</option>
                    </select>
                </div>
                <div style="display:flex;gap:8px;">
                    <div style="flex:1;">
                        <div style="font-size:10px;color:#7a8ca3;margin-bottom:3px;text-align:center;">效率预测</div>
                        <svg viewBox="0 0 100 55" width="100%" height="55">
                            <defs>
                                <linearGradient id="effGrad${idx}" x1="0" y1="0" x2="1" y2="0">
                                    <stop offset="0%" stop-color="#4caf50"/>
                                    <stop offset="100%" stop-color="#00ff88"/>
                                </linearGradient>
                            </defs>
                            <path d="M 10 50 A 40 40 0 0 1 90 50" fill="none" stroke="#1e3a5f" stroke-width="6"/>
                            <path d="M 10 50 A 40 40 0 0 1 ${10 + 80 * ((unit.efficiency_pred - 80) / 20)} ${50 - 40 * Math.sin(Math.PI * ((unit.efficiency_pred - 80) / 20))}" 
                                fill="none" stroke="url(#effGrad${idx})" stroke-width="6" stroke-linecap="round"/>
                            <text x="50" y="42" text-anchor="middle" fill="#00ff88" font-size="11" font-weight="700" font-family="monospace">${unit.efficiency_pred.toFixed(1)}%</text>
                        </svg>
                    </div>
                    <div style="flex:1;">
                        <div style="font-size:10px;color:#7a8ca3;margin-bottom:3px;text-align:center;">空化风险</div>
                        <svg viewBox="0 0 100 55" width="100%" height="55">
                            <defs>
                                <linearGradient id="cavGrad${idx}" x1="0" y1="0" x2="1" y2="0">
                                    <stop offset="0%" stop-color="#4caf50"/>
                                    <stop offset="50%" stop-color="#ffc107"/>
                                    <stop offset="100%" stop-color="#f44336"/>
                                </linearGradient>
                            </defs>
                            <path d="M 10 50 A 40 40 0 0 1 90 50" fill="none" stroke="#1e3a5f" stroke-width="6"/>
                            <path d="M 10 50 A 40 40 0 0 1 ${10 + 80 * unit.cav_risk_pred} ${50 - 40 * Math.sin(Math.PI * unit.cav_risk_pred)}" 
                                fill="none" stroke="url(#cavGrad${idx})" stroke-width="6" stroke-linecap="round"/>
                            <text x="50" y="42" text-anchor="middle" fill="${unit.cav_risk_pred > 0.6 ? '#f44336' : unit.cav_risk_pred > 0.3 ? '#ffc107' : '#4caf50'}" 
                                font-size="11" font-weight="700" font-family="monospace">${(unit.cav_risk_pred * 100).toFixed(0)}%</text>
                        </svg>
                    </div>
                </div>
            `;

            card.querySelectorAll('input[type="range"]').forEach(slider => {
                slider.addEventListener('input', (e) => {
                    const idx = parseInt(e.target.dataset.idx);
                    const field = e.target.dataset.field;
                    this.units[idx][field] = parseFloat(e.target.value);
                    const valueSpan = e.target.parentElement.querySelector('span:last-child');
                    if (field === 'guide_vane') valueSpan.textContent = this.units[idx][field].toFixed(1) + '%';
                    else valueSpan.textContent = this.units[idx][field].toFixed(0) + ' MW';
                    this.sendCommand(idx);
                });
            });

            const select = card.querySelector('select');
            select.value = unit.mode;
            select.addEventListener('change', (e) => {
                const idx = parseInt(e.target.dataset.idx);
                const modeMap = { 'manual': '手动', 'efficiency': '效率优先', 'avoid_cav': '避空化', 'mpc_optimal': 'MPC最优' };
                this.units[idx].mode = e.target.value;
                this.units[idx].mode_name = modeMap[e.target.value];
                card.querySelector('span[style*="font-size:10px"]').textContent = modeMap[e.target.value];
                this.sendCommand(idx);
            });

            panel.appendChild(card);
        });

        return panel;
    }

    createCenterPanel() {
        const panel = document.createElement('div');
        panel.style.cssText = 'flex:1;display:flex;flex-direction:column;background:rgba(10,22,40,0.5);border:1px solid #1e3a5f;border-radius:8px;padding:12px;min-width:0;';
        panel.innerHTML = `
            <h3 style="font-size:14px;color:#7a8ca3;margin:0 0 8px 0;text-transform:uppercase;letter-spacing:0.5px;">导叶机构动态</h3>
            <div style="flex:1;position:relative;min-height:0;">
                <svg id="guideVaneSvg" viewBox="0 0 500 500" style="width:100%;height:100%;">
                    <defs>
                        <radialGradient id="hubGradient" cx="50%" cy="50%" r="50%">
                            <stop offset="0%" stop-color="#2d4a6f"/>
                            <stop offset="100%" stop-color="#1e3a5f"/>
                        </radialGradient>
                        <linearGradient id="flowGradient" x1="0%" y1="0%" x2="100%" y2="0%">
                            <stop offset="0%" stop-color="#00d4ff" stop-opacity="0"/>
                            <stop offset="50%" stop-color="#00d4ff" stop-opacity="0.6"/>
                            <stop offset="100%" stop-color="#00d4ff" stop-opacity="0"/>
                        </linearGradient>
                    </defs>
                    <circle cx="250" cy="250" r="230" fill="none" stroke="#1e3a5f" stroke-width="2"/>
                    <circle cx="250" cy="250" r="180" fill="none" stroke="#1e3a5f" stroke-width="2" stroke-dasharray="4 4"/>
                    <g id="guideVaneGroup"></g>
                    <circle cx="250" cy="250" r="100" fill="url(#hubGradient)" stroke="#00d4ff" stroke-width="2"/>
                    <circle cx="250" cy="250" r="30" fill="#0a1628" stroke="#00d4ff" stroke-width="1.5"/>
                    <text x="250" y="254" text-anchor="middle" fill="#00d4ff" font-size="11" font-family="monospace">转轮</text>
                    <g id="flowArrows"></g>
                </svg>
            </div>
            <div style="display:flex;justify-content:center;gap:24px;margin-top:8px;font-size:12px;">
                <span style="color:#7a8ca3;">平均开度: <span id="avgGuideVane" style="color:#00d4ff;font-family:monospace;">0%</span></span>
                <span style="color:#7a8ca3;">总流量: <span id="totalFlow" style="color:#00ff88;font-family:monospace;">0 m³/s</span></span>
            </div>
        `;
        this.renderGuideVanes(panel);
        return panel;
    }

    renderGuideVanes(panel) {
        const group = panel.querySelector('#guideVaneGroup');
        const flowGroup = panel.querySelector('#flowArrows');
        if (!group) return;
        group.innerHTML = '';
        flowGroup.innerHTML = '';

        const cx = 250, cy = 250;
        const innerR = 110, outerR = 170;
        const numVaneb = 24;

        const avgOpen = this.units.reduce((s, u) => s + u.guide_vane, 0) / this.units.length;
        const openAngle = (avgOpen / 100) * 35;
        this.guideVaneRotation = openAngle;

        for (let i = 0; i < numVaneb; i++) {
            const baseAngle = (i / numVaneb) * Math.PI * 2;
            const pivotX = cx + Math.cos(baseAngle) * (innerR + outerR) / 2;
            const pivotY = cy + Math.sin(baseAngle) * (innerR + outerR) / 2;

            const vaneLength = (outerR - innerR) * 1.2;
            const tangentAngle = baseAngle + Math.PI / 2;
            const rotRad = (openAngle * Math.PI / 180);

            const x1 = pivotX - Math.cos(tangentAngle - rotRad) * vaneLength / 2;
            const y1 = pivotY - Math.sin(tangentAngle - rotRad) * vaneLength / 2;
            const x2 = pivotX + Math.cos(tangentAngle - rotRad) * vaneLength / 2;
            const y2 = pivotY + Math.sin(tangentAngle - rotRad) * vaneLength / 2;

            const line = document.createElementNS('http://www.w3.org/2000/svg', 'line');
            line.setAttribute('x1', x1);
            line.setAttribute('y1', y1);
            line.setAttribute('x2', x2);
            line.setAttribute('y2', y2);
            line.setAttribute('stroke', '#00d4ff');
            line.setAttribute('stroke-width', '4');
            line.setAttribute('stroke-linecap', 'round');
            group.appendChild(line);

            const pivot = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
            pivot.setAttribute('cx', pivotX);
            pivot.setAttribute('cy', pivotY);
            pivot.setAttribute('r', '3');
            pivot.setAttribute('fill', '#ffc107');
            group.appendChild(pivot);
        }

        for (let i = 0; i < 8; i++) {
            const angle = (i / 8) * Math.PI * 2 + openAngle * Math.PI / 180 * 0.3;
            const arrowR = 210;
            const ax = cx + Math.cos(angle) * arrowR;
            const ay = cy + Math.sin(angle) * arrowR;
            const tipX = cx + Math.cos(angle) * (arrowR - 25);
            const tipY = cy + Math.sin(angle) * (arrowR - 25);
            const perp = angle + Math.PI / 2;
            const w1x = tipX + Math.cos(perp) * 6;
            const w1y = tipY + Math.sin(perp) * 6;
            const w2x = tipX - Math.cos(perp) * 6;
            const w2y = tipY - Math.sin(perp) * 6;

            const arrow = document.createElementNS('http://www.w3.org/2000/svg', 'path');
            arrow.setAttribute('d', `M ${ax} ${ay} L ${tipX} ${tipY} M ${w1x} ${w1y} L ${tipX} ${tipY} L ${w2x} ${w2y}`);
            arrow.setAttribute('stroke', '#00d4ff');
            arrow.setAttribute('stroke-width', '2');
            arrow.setAttribute('fill', 'none');
            arrow.setAttribute('opacity', '0.6');
            arrow.classList.add('flow-arrow');
            flowGroup.appendChild(arrow);
        }

        const avgSpan = panel.querySelector('#avgGuideVane');
        const flowSpan = panel.querySelector('#totalFlow');
        if (avgSpan) avgSpan.textContent = avgOpen.toFixed(1) + '%';
        if (flowSpan) flowSpan.textContent = (avgOpen * 3.5).toFixed(1) + ' m³/s';
    }

    createRightPanel() {
        const panel = document.createElement('div');
        panel.style.cssText = 'width:380px;display:flex;flex-direction:column;gap:10px;overflow-y:auto;padding-right:4px;';
        panel.innerHTML = `
            <h3 style="font-size:14px;color:#7a8ca3;margin:0 0 0 4px;text-transform:uppercase;letter-spacing:0.5px;">MPC性能指标</h3>
            <div style="background:rgba(30,58,95,0.5);border:1px solid #1e3a5f;border-radius:8px;padding:10px;">
                <div style="font-size:12px;color:#7a8ca3;margin-bottom:6px;">目标函数 (60s滑窗)</div>
                <canvas id="objCanvas" width="340" height="100" style="width:100%;height:100px;border-radius:4px;background:#0a0f1a;"></canvas>
            </div>
            <div style="background:rgba(30,58,95,0.5);border:1px solid #1e3a5f;border-radius:8px;padding:10px;">
                <div style="font-size:12px;color:#7a8ca3;margin-bottom:6px;">控制增量 Δu (6台机组)</div>
                <canvas id="ctrlCanvas" width="340" height="90" style="width:100%;height:90px;border-radius:4px;background:#0a0f1a;"></canvas>
            </div>
            <div style="background:rgba(30,58,95,0.5);border:1px solid #1e3a5f;border-radius:8px;padding:10px;">
                <div style="font-size:12px;color:#7a8ca3;margin-bottom:6px;">预测状态 Np=${this.predictionHorizon}步</div>
                <canvas id="predCanvas" width="340" height="120" style="width:100%;height:120px;border-radius:4px;background:#0a0f1a;"></canvas>
            </div>
        `;
        return panel;
    }

    createBottomPanel() {
        const panel = document.createElement('div');
        panel.style.cssText = 'height:auto;background:rgba(30,58,95,0.5);border:1px solid #1e3a5f;border-radius:8px;padding:12px 16px;display:flex;align-items:center;gap:32px;flex-wrap:wrap;';

        const toggleChecked = this.avoidCavitation ? 'checked' : '';

        panel.innerHTML = `
            <div style="display:flex;align-items:center;gap:10px;">
                <span style="font-size:12px;color:#7a8ca3;">避空化使能:</span>
                <label style="position:relative;display:inline-block;width:44px;height:22px;">
                    <input type="checkbox" id="avoidCavToggle" ${toggleChecked} style="opacity:0;width:0;height:0;"/>
                    <span style="position:absolute;cursor:pointer;inset:0;background:#1e3a5f;transition:.3s;border-radius:22px;border:1px solid #2d4a6f;">
                        <span style="position:absolute;content:'';height:16px;width:16px;left:${this.avoidCavitation ? '24px' : '2px'};bottom:2px;background:white;transition:.3s;border-radius:50%;${this.avoidCavitation ? 'background:#00d4ff' : ''}"></span>
                    </span>
                </label>
                <span id="avoidCavStatus" style="font-size:12px;font-family:monospace;color:${this.avoidCavitation ? '#00d4ff' : '#7a8ca3'};">${this.avoidCavitation ? 'ON' : 'OFF'}</span>
            </div>
            <button id="emergencyStopBtn" style="padding:10px 28px;background:linear-gradient(135deg,#f44336,#d32f2f);border:1px solid #f44336;border-radius:6px;color:white;font-size:13px;font-weight:600;cursor:pointer;letter-spacing:1px;box-shadow:0 2px 12px rgba(244,67,54,0.4);">
                ⚠ 紧急停机
            </button>
            <div style="flex:1;display:flex;gap:20px;flex-wrap:wrap;min-width:400px;">
                ${this.createWeightSlider('W_eff', '效率权重', this.mpcWeights.W_eff, '#00ff88')}
                ${this.createWeightSlider('W_cav', '空化权重', this.mpcWeights.W_cav, '#ff9800')}
                ${this.createWeightSlider('W_power', '功率权重', this.mpcWeights.W_power, '#2196f3')}
                ${this.createWeightSlider('W_du', 'Δu权重', this.mpcWeights.W_du, '#9c27b0')}
            </div>
        `;

        setTimeout(() => {
            const toggle = panel.querySelector('#avoidCavToggle');
            toggle.addEventListener('change', (e) => {
                this.avoidCavitation = e.target.checked;
                const indicator = panel.querySelector('#avoidCavStatus');
                indicator.textContent = this.avoidCavitation ? 'ON' : 'OFF';
                indicator.style.color = this.avoidCavitation ? '#00d4ff' : '#7a8ca3';
                const knob = panel.querySelector('span[style*="position:absolute;content"]');
                if (knob) {
                    knob.style.left = this.avoidCavitation ? '24px' : '2px';
                    knob.style.background = this.avoidCavitation ? '#00d4ff' : 'white';
                }
            });

            const stopBtn = panel.querySelector('#emergencyStopBtn');
            stopBtn.addEventListener('click', () => {
                if (confirm('确认紧急停机所有机组？')) {
                    this.units.forEach(u => {
                        u.guide_vane = 0;
                        u.mode = 'manual';
                        u.mode_name = '手动';
                    });
                    this.render(this.container);
                }
            });

            panel.querySelectorAll('input[type="range"].weight-slider').forEach(slider => {
                slider.addEventListener('input', (e) => {
                    const key = e.target.dataset.key;
                    this.mpcWeights[key] = parseFloat(e.target.value);
                    const valSpan = document.getElementById('weightVal_' + key);
                    if (valSpan) valSpan.textContent = this.mpcWeights[key].toFixed(2);
                });
            });
        }, 50);

        return panel;
    }

    createWeightSlider(key, label, value, color) {
        return `
            <div style="flex:1;min-width:140px;">
                <div style="display:flex;justify-content:space-between;font-size:11px;color:#7a8ca3;margin-bottom:3px;">
                    <span>${label}</span>
                    <span id="weightVal_${key}" style="color:${color};font-family:monospace;">${value.toFixed(2)}</span>
                </div>
                <input type="range" class="weight-slider" data-key="${key}" min="0" max="1" step="0.01" value="${value}"
                    style="width:100%;height:3px;appearance:none;background:#1e3a5f;border-radius:2px;outline:none;cursor:pointer;"/>
            </div>
        `;
    }

    renderObjectiveChart() {
        const canvas = document.getElementById('objCanvas');
        if (!canvas) return;
        const ctx = canvas.getContext('2d');
        const w = canvas.width, h = canvas.height;
        ctx.fillStyle = '#0a0f1a';
        ctx.fillRect(0, 0, w, h);

        const pad = { l: 30, r: 10, t: 10, b: 20 };
        const cw = w - pad.l - pad.r, ch = h - pad.t - pad.b;

        ctx.strokeStyle = '#1e3a5f';
        ctx.lineWidth = 0.5;
        for (let i = 0; i <= 4; i++) {
            const y = pad.t + (i / 4) * ch;
            ctx.beginPath();
            ctx.moveTo(pad.l, y);
            ctx.lineTo(w - pad.r, y);
            ctx.stroke();
        }

        const data = this.objectiveHistory;
        const max = Math.max(...data) * 1.1, min = 0;

        const grad = ctx.createLinearGradient(0, pad.t, 0, h - pad.b);
        grad.addColorStop(0, 'rgba(0,212,255,0.5)');
        grad.addColorStop(1, 'rgba(0,212,255,0.05)');
        ctx.beginPath();
        ctx.moveTo(pad.l, h - pad.b);
        data.forEach((v, i) => {
            const x = pad.l + (i / (data.length - 1)) * cw;
            const y = pad.t + ch - ((v - min) / (max - min)) * ch;
            ctx.lineTo(x, y);
        });
        ctx.lineTo(w - pad.r, h - pad.b);
        ctx.closePath();
        ctx.fillStyle = grad;
        ctx.fill();

        ctx.beginPath();
        data.forEach((v, i) => {
            const x = pad.l + (i / (data.length - 1)) * cw;
            const y = pad.t + ch - ((v - min) / (max - min)) * ch;
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        });
        ctx.strokeStyle = '#00d4ff';
        ctx.lineWidth = 1.5;
        ctx.stroke();

        ctx.fillStyle = '#666';
        ctx.font = '9px monospace';
        ctx.textAlign = 'right';
        for (let i = 0; i <= 2; i++) {
            const y = pad.t + (i / 2) * ch;
            ctx.fillText(((max - min) * (1 - i / 2)).toFixed(0), pad.l - 3, y + 3);
        }
        ctx.textAlign = 'center';
        ctx.fillText('-60s', pad.l, h - 5);
        ctx.fillText('now', w - pad.r, h - 5);
    }

    renderControlIncrements() {
        const canvas = document.getElementById('ctrlCanvas');
        if (!canvas) return;
        const ctx = canvas.getContext('2d');
        const w = canvas.width, h = canvas.height;
        ctx.fillStyle = '#0a0f1a';
        ctx.fillRect(0, 0, w, h);

        const pad = { l: 30, r: 10, t: 8, b: 20 };
        const cw = w - pad.l - pad.r, ch = h - pad.t - pad.b;

        ctx.strokeStyle = '#1e3a5f';
        ctx.beginPath();
        ctx.moveTo(pad.l, pad.t + ch / 2);
        ctx.lineTo(w - pad.r, pad.t + ch / 2);
        ctx.stroke();

        const barW = cw / this.controlIncrements.length * 0.6;
        const gap = cw / this.controlIncrements.length * 0.4;
        const maxAbs = Math.max(...this.controlIncrements.map(v => Math.abs(v)), 1);

        const colors = ['#00d4ff', '#00ff88', '#ffc107', '#ff9800', '#2196f3', '#9c27b0'];
        this.controlIncrements.forEach((v, i) => {
            const x = pad.l + i * (barW + gap) + gap / 2;
            const barH = (Math.abs(v) / maxAbs) * (ch / 2 - 4);
            const y = pad.t + ch / 2 + (v >= 0 ? -barH : 0);
            ctx.fillStyle = colors[i % colors.length];
            ctx.fillRect(x, y, barW, barH);
            ctx.fillStyle = '#888';
            ctx.font = '9px sans-serif';
            ctx.textAlign = 'center';
            ctx.fillText(`${i + 1}#`, x + barW / 2, h - 5);
        });

        ctx.fillStyle = '#666';
        ctx.font = '9px monospace';
        ctx.textAlign = 'right';
        ctx.fillText('+' + maxAbs.toFixed(0), pad.l - 3, pad.t + 6);
        ctx.fillText('0', pad.l - 3, pad.t + ch / 2 + 3);
        ctx.fillText('-' + maxAbs.toFixed(0), pad.l - 3, h - pad.b);
    }

    renderPredictionStates() {
        const canvas = document.getElementById('predCanvas');
        if (!canvas) return;
        const ctx = canvas.getContext('2d');
        const w = canvas.width, h = canvas.height;
        ctx.fillStyle = '#0a0f1a';
        ctx.fillRect(0, 0, w, h);

        const pad = { l: 35, r: 35, t: 10, b: 22 };
        const cw = w - pad.l - pad.r, ch = (h - pad.t - pad.b) / 2 - 4;

        const drawChart = (dataKey, yLabel, color, yOffset) => {
            const data = this.predictionStates.map(s => s[dataKey]);
            const max = Math.max(...data) * 1.1;
            const min = Math.min(...data) * 0.9;
            const range = max - min || 1;

            for (let i = 0; i <= 3; i++) {
                const y = pad.t + yOffset + (i / 3) * ch;
                ctx.strokeStyle = '#1e3a5f';
                ctx.lineWidth = 0.5;
                ctx.beginPath();
                ctx.moveTo(pad.l, y);
                ctx.lineTo(w - pad.r, y);
                ctx.stroke();
                ctx.fillStyle = '#555';
                ctx.font = '8px monospace';
                ctx.textAlign = 'right';
                ctx.fillText(((max - min) * (1 - i / 3) + min).toFixed(dataKey === 'cav_risk' ? 2 : 0), pad.l - 2, y + 2);
            }

            const barW = cw / this.predictionHorizon * 0.75;
            this.predictionStates.forEach((s, i) => {
                const v = s[dataKey];
                const x = pad.l + i * (cw / this.predictionHorizon) + (cw / this.predictionHorizon - barW) / 2;
                const barH = ((v - min) / range) * ch;
                const y = pad.t + yOffset + ch - barH;
                const intensity = dataKey === 'cav_risk' ? v : (i / this.predictionHorizon);
                ctx.fillStyle = dataKey === 'cav_risk'
                    ? `hsl(${120 - intensity * 120}, 70%, 50%)`
                    : color;
                ctx.fillRect(x, y, barW, barH);
            });

            ctx.fillStyle = '#7a8ca3';
            ctx.font = '9px sans-serif';
            ctx.textAlign = 'left';
            ctx.fillText(yLabel, pad.l, pad.t + yOffset + 9);
        };

        drawChart('power', '功率 (MW)', '#00d4ff', 0);
        drawChart('cav_risk', '空化风险', '#ff9800', ch + 8);

        ctx.fillStyle = '#555';
        ctx.font = '8px monospace';
        ctx.textAlign = 'center';
        for (let i = 0; i < this.predictionHorizon; i += 4) {
            const x = pad.l + i * (cw / this.predictionHorizon) + (cw / this.predictionHorizon) / 2;
            ctx.fillText('k+' + (i + 1), x, h - 8);
        }
    }

    startAnimations() {
        if (this.updateInterval) clearInterval(this.updateInterval);
        this.updateInterval = setInterval(() => {
            this.objectiveHistory.shift();
            const last = this.objectiveHistory[this.objectiveHistory.length - 1];
            this.objectiveHistory.push(Math.max(20, Math.min(90, last + (Math.random() - 0.5) * 8)));

            this.controlIncrements = this.controlIncrements.map(v => Math.max(-8, Math.min(8, v + (Math.random() - 0.5) * 2)));

            this.predictionStates.forEach((s, i) => {
                s.power = 650 + Math.sin((Date.now() / 2000 + i) / 3) * 80 + (Math.random() - 0.5) * 20;
                s.cav_risk = Math.max(0, Math.min(1, 0.3 + Math.sin((Date.now() / 3000 + i) / 5) * 0.2 + (Math.random() - 0.5) * 0.1));
            });

            this.units.forEach(u => {
                u.efficiency_pred = Math.max(80, Math.min(98, u.efficiency_pred + (Math.random() - 0.5) * 0.5));
                u.cav_risk_pred = Math.max(0, Math.min(1, u.cav_risk_pred + (Math.random() - 0.5) * 0.05));
            });

            this.renderObjectiveChart();
            this.renderControlIncrements();
            this.renderPredictionStates();

            if (this.container) {
                this.renderGuideVanes(this.container);
                this.updateUnitCards();
            }
        }, 1000);
    }

    updateUnitCards() {
        if (!this.container) return;
        const cards = this.container.querySelectorAll('[style*="border:1px solid #1e3a5f"] > div > svg');
    }

    async sendCommand(idx) {
        const unit = this.units[idx];
        try {
            await fetch(`${this.app.apiBase}/control/command`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    turbineId: unit.id,
                    guide_vane: unit.guide_vane,
                    target_power: unit.target_power,
                    mode: unit.mode
                })
            });
        } catch (e) {
        }
    }

    resize() {
        if (!this.container) return;
        this.renderObjectiveChart();
        this.renderControlIncrements();
        this.renderPredictionStates();
        this.renderGuideVanes(this.container);
    }
}
