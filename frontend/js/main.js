class TurbineMonitorApp {
    constructor() {
        this.apiBase = 'http://localhost:8080/api';
        this.currentTurbineId = 0;
        this.turbineViewer = null;
        this.waterfallChart = null;
        this.trendChart = null;
        this.turbines = [];
        this.alarms = [];
        this.updateInterval = null;
        this.waterfallInterval = null;
        this.mpcControlTab = null;
        this.robotRepairTab = null;
        this.unitScheduleTab = null;
        this.acousticDiagnosisTab = null;
        
        this.init();
    }

    init() {
        this.initTurbineViewer();
        this.initWaterfallChart();
        this.initTrendChart();
        this.initNewTabs();
        this.initEventListeners();
        this.loadTurbines();
        this.startDataUpdates();
        this.startTimeUpdate();
    }

    initNewTabs() {
        this.mpcControlTab = new MPCControlTab(this);
        this.robotRepairTab = new RobotRepairTab(this);
        this.unitScheduleTab = new UnitScheduleTab(this);
        this.acousticDiagnosisTab = new AcousticDiagnosisTab(this);
    }

    initTurbineViewer() {
        this.turbineViewer = new TurbineViewer('turbineCanvas');
        window.bladeSelectionCallback = (bladeId, turbineId) => {
            this.showBladeModal(bladeId, turbineId);
            this.updateBladeDetail(bladeId, turbineId);
        };
    }

    initWaterfallChart() {
        this.waterfallChart = new WaterfallChart('waterfallCanvas');
    }

    initTrendChart() {
        const canvas = document.getElementById('trendCanvas');
        if (canvas) {
            this.trendChart = new TrendChart(canvas);
        }
    }

    initEventListeners() {
        document.querySelectorAll('.tab-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const tabId = e.target.dataset.tab;
                this.switchTab(tabId);
            });
        });

        document.querySelectorAll('.filter-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const filter = e.target.dataset.filter;
                this.filterAlarms(filter);
            });
        });

        document.getElementById('pauseBtn').addEventListener('click', () => {
            const paused = this.waterfallChart.pause();
            document.getElementById('pauseBtn').textContent = paused ? '继续' : '暂停';
        });

        document.getElementById('clearBtn').addEventListener('click', () => {
            this.waterfallChart.clear();
        });

        document.getElementById('closeModal').addEventListener('click', () => {
            document.getElementById('bladeModal').style.display = 'none';
        });

        document.getElementById('bladeModal').addEventListener('click', (e) => {
            if (e.target.id === 'bladeModal') {
                document.getElementById('bladeModal').style.display = 'none';
            }
        });
    }

    switchTab(tabId) {
        document.querySelectorAll('.tab-btn').forEach(btn => {
            btn.classList.toggle('active', btn.dataset.tab === tabId);
        });
        document.querySelectorAll('.tab-content').forEach(content => {
            content.classList.toggle('active', content.id === tabId);
        });

        if (tabId === 'turbine-view') {
            setTimeout(() => this.turbineViewer.resize(), 100);
        } else if (tabId === 'waterfall-view') {
            setTimeout(() => this.waterfallChart.resize(), 100);
            this.startWaterfallUpdates();
        } else if (tabId === 'alarm-view') {
            this.loadAlarms();
        } else if (tabId === 'control-view') {
            const container = document.getElementById('controlTabContainer');
            if (container) this.mpcControlTab.render(container);
            setTimeout(() => this.mpcControlTab.resize(), 150);
        } else if (tabId === 'robot-view') {
            const container = document.getElementById('robotTabContainer');
            if (container) this.robotRepairTab.render(container);
            setTimeout(() => this.robotRepairTab.resize(), 150);
        } else if (tabId === 'schedule-view') {
            const container = document.getElementById('scheduleTabContainer');
            if (container) this.unitScheduleTab.render(container);
            setTimeout(() => this.unitScheduleTab.resize(), 150);
        } else if (tabId === 'diagnosis-view') {
            const container = document.getElementById('diagnosisTabContainer');
            if (container) this.acousticDiagnosisTab.render(container);
            setTimeout(() => this.acousticDiagnosisTab.resize(), 150);
        }
    }

    async loadTurbines() {
        try {
            const response = await fetch(`${this.apiBase}/turbines`);
            if (response.ok) {
                this.turbines = await response.json();
                this.renderTurbineList();
            } else {
                this.generateMockTurbines();
                this.renderTurbineList();
            }
        } catch (e) {
            this.generateMockTurbines();
            this.renderTurbineList();
        }
    }

    generateMockTurbines() {
        this.turbines = [];
        const modes = ['manual', 'efficiency', 'avoid_cav', 'mpc_optimal'];
        for (let i = 0; i < 6; i++) {
            const guideVane = 30 + Math.random() * 40;
            const targetPower = 600 + Math.random() * 200;
            const mode = modes[Math.floor(Math.random() * 4)];
            this.turbines.push({
                id: i,
                name: `${i + 1}# 水轮机`,
                status: 'running',
                power: 700 + Math.random() * 50,
                efficiency: 92 + Math.random() * 3,
                cavitationStage: Math.floor(Math.random() * 2),
                maxCavitation: 0.1 + Math.random() * 0.3,
                maxVibration: 0.05 + Math.random() * 0.15,
                remainingLife: 95 + Math.random() * 5,
                control: {
                    guide_vane: guideVane,
                    power: targetPower,
                    mode: mode,
                    efficiency_pred: 85 + Math.random() * 10,
                    cav_risk_pred: Math.random() * 0.8
                }
            });
        }
    }

    renderTurbineList() {
        const container = document.getElementById('turbineList');
        container.innerHTML = '';

        this.turbines.forEach((turbine, idx) => {
            const item = document.createElement('div');
            item.className = 'turbine-item';
            if (idx === this.currentTurbineId) {
                item.classList.add('active');
            }

            const stageClasses = ['normal', 'incipient', 'critical', 'developed'];
            const stageNames = ['正常', '初生', '临界', '发展'];
            const stageClass = stageClasses[turbine.cavitationStage || 0];
            const stageName = stageNames[turbine.cavitationStage || 0];

            item.innerHTML = `
                <div class="turbine-header">
                    <span class="turbine-name">${turbine.name}</span>
                    <span class="status-dot ${stageClass}"></span>
                </div>
                <div class="turbine-stats">
                    <div>功率: ${turbine.power.toFixed(0)} MW</div>
                    <div>空化: <span class="${stageClass}">${stageName}</span></div>
                    <div>寿命: ${turbine.remainingLife.toFixed(1)}%</div>
                </div>
            `;

            item.addEventListener('click', () => {
                this.selectTurbine(idx);
            });

            container.appendChild(item);
        });
    }

    selectTurbine(turbineId) {
        this.currentTurbineId = turbineId;
        this.turbineViewer.setTurbine(turbineId);
        this.renderTurbineList();
        this.updateTurbineInfo();
    }

    updateTurbineInfo() {
        const turbine = this.turbines[this.currentTurbineId];
        if (!turbine) return;

        document.getElementById('currentTurbineName').textContent = turbine.name;
        
        const stageClasses = ['normal', 'incipient', 'critical', 'developed'];
        const stageNames = ['正常', '初生空化', '临界空化', '发展空化'];
        const stage = turbine.cavitationStage || 0;
        
        const statusBadge = document.getElementById('cavitationStatus');
        statusBadge.className = `status-badge ${stageClasses[stage]}`;
        statusBadge.textContent = stageNames[stage];

        document.getElementById('maxCavitation').textContent = turbine.maxCavitation.toFixed(2);
        document.getElementById('maxVibration').textContent = turbine.maxVibration.toFixed(2) + ' g';
        document.getElementById('remainingLife').textContent = turbine.remainingLife.toFixed(1) + '%';

        this.updateLifeGauge(turbine.remainingLife);
    }

    updateLifeGauge(value) {
        const circumference = 2 * Math.PI * 40;
        const offset = circumference - (value / 100) * circumference;
        document.getElementById('lifeArc').setAttribute('stroke-dashoffset', offset);
        document.getElementById('lifeValue').textContent = value.toFixed(1) + '%';
    }

    showBladeModal(bladeId, turbineId) {
        const turbine = this.turbines[turbineId];
        const bladeData = this.turbineViewer.cavitationData[bladeId];
        const stageNames = ['正常', '初生', '临界', '发展'];

        document.getElementById('modalTitle').textContent = `${turbine.name} - 叶片 #${bladeId + 1}`;
        
        const modalBody = document.getElementById('modalBody');
        modalBody.innerHTML = `
            <div class="modal-section">
                <h4>空化信息</h4>
                <div class="info-grid">
                    <div class="info-item">
                        <span class="label">当前强度</span>
                        <span class="value">${(bladeData.intensity * 100).toFixed(1)}%</span>
                    </div>
                    <div class="info-item">
                        <span class="label">空化阶段</span>
                        <span class="value">${stageNames[bladeData.stage]}</span>
                    </div>
                    <div class="info-item">
                        <span class="label">累计损伤</span>
                        <span class="value">${(bladeData.damage * 100).toFixed(3)}%</span>
                    </div>
                    <div class="info-item">
                        <span class="label">当前应力</span>
                        <span class="value">${bladeData.stress.toFixed(1)} MPa</span>
                    </div>
                </div>
            </div>
            <div class="modal-section">
                <h4>空化历史趋势（最近24小时）</h4>
                <canvas id="modalTrendCanvas" width="500" height="180"></canvas>
            </div>
            <div class="modal-section">
                <h4>寿命评估</h4>
                <div class="info-grid">
                    <div class="info-item">
                        <span class="label">剩余寿命占比</span>
                        <span class="value">${(100 - bladeData.damage * 100).toFixed(2)}%</span>
                    </div>
                    <div class="info-item">
                        <span class="label">预计剩余时间</span>
                        <span class="value">${(10 / (bladeData.damage * 100 + 0.001)).toFixed(1)} 年</span>
                    </div>
                    <div class="info-item">
                        <span class="label">损伤速率</span>
                        <span class="value">${(bladeData.damage * 1e6 / 24).toFixed(1)}e-6 /h</span>
                    </div>
                    <div class="info-item">
                        <span class="label">建议检修时间</span>
                        <span class="value">${Math.max(0, Math.floor((10 / (bladeData.damage * 100 + 0.001)) * 0.8))} 个月后</span>
                    </div>
                </div>
            </div>
        `;

        const trendCanvas = document.getElementById('modalTrendCanvas');
        if (trendCanvas) {
            this.renderMiniTrend(trendCanvas, bladeData);
        }

        document.getElementById('bladeModal').style.display = 'block';
    }

    renderMiniTrend(canvas, bladeData) {
        const ctx = canvas.getContext('2d');
        const width = canvas.width;
        const height = canvas.height;

        ctx.fillStyle = '#0a0f1a';
        ctx.fillRect(0, 0, width, height);

        const data = [];
        const hours = 24;
        const points = 120;
        for (let i = 0; i < points; i++) {
            const t = i / points;
            let val = bladeData.intensity * (0.7 + Math.sin(t * Math.PI * 4) * 0.2 + Math.random() * 0.2);
            
            if (t > 0.8) {
                val += (t - 0.8) * 0.5;
            }
            
            data.push(Math.max(0, Math.min(1, val)));
        }

        const padding = { left: 50, right: 20, top: 20, bottom: 30 };
        const chartWidth = width - padding.left - padding.right;
        const chartHeight = height - padding.top - padding.bottom;

        const maxVal = Math.max(...data) * 1.1;
        const minVal = 0;

        ctx.strokeStyle = '#222';
        ctx.lineWidth = 1;
        for (let i = 0; i <= 4; i++) {
            const y = padding.top + (i / 4) * chartHeight;
            ctx.beginPath();
            ctx.moveTo(padding.left, y);
            ctx.lineTo(width - padding.right, y);
            ctx.stroke();

            ctx.fillStyle = '#888';
            ctx.font = '11px sans-serif';
            ctx.textAlign = 'right';
            ctx.fillText(((maxVal - minVal) * (1 - i / 4) * 100).toFixed(0) + '%', padding.left - 5, y + 4);
        }

        const gradient = ctx.createLinearGradient(0, padding.top, 0, height - padding.bottom);
        gradient.addColorStop(0, 'rgba(255, 80, 80, 0.8)');
        gradient.addColorStop(0.5, 'rgba(255, 200, 50, 0.5)');
        gradient.addColorStop(1, 'rgba(50, 200, 100, 0.3)');

        ctx.beginPath();
        ctx.moveTo(padding.left, height - padding.bottom);
        for (let i = 0; i < data.length; i++) {
            const x = padding.left + (i / (data.length - 1)) * chartWidth;
            const y = padding.top + chartHeight - ((data[i] - minVal) / (maxVal - minVal)) * chartHeight;
            
            if (i === 0) {
                ctx.lineTo(x, y);
            } else {
                ctx.lineTo(x, y);
            }
        }
        ctx.lineTo(width - padding.right, height - padding.bottom);
        ctx.closePath();
        ctx.fillStyle = gradient;
        ctx.fill();

        ctx.beginPath();
        for (let i = 0; i < data.length; i++) {
            const x = padding.left + (i / (data.length - 1)) * chartWidth;
            const y = padding.top + chartHeight - ((data[i] - minVal) / (maxVal - minVal)) * chartHeight;
            
            if (i === 0) {
                ctx.moveTo(x, y);
            } else {
                ctx.lineTo(x, y);
            }
        }
        ctx.strokeStyle = '#00d4ff';
        ctx.lineWidth = 2;
        ctx.stroke();

        ctx.fillStyle = '#888';
        ctx.font = '11px sans-serif';
        ctx.textAlign = 'center';
        for (let i = 0; i <= 4; i++) {
            const x = padding.left + (i / 4) * chartWidth;
            ctx.fillText(`${-hours + i * (hours / 4)}h`, x, height - padding.bottom + 20);
        }

        ctx.fillStyle = '#fff';
        ctx.fillText('时间', width / 2, height - 5);
    }

    updateBladeDetail(bladeId, turbineId) {
        const bladeData = this.turbineViewer.cavitationData[bladeId];
        const stageNames = ['正常', '初生', '临界', '发展'];

        document.getElementById('bladeDetail').innerHTML = `
            <div class="detail-item">
                <span class="label">叶片编号</span>
                <span class="value">#${bladeId + 1}</span>
            </div>
            <div class="detail-item">
                <span class="label">空化强度</span>
                <span class="value intensity-${bladeData.stage}">${(bladeData.intensity * 100).toFixed(1)}%</span>
            </div>
            <div class="detail-item">
                <span class="label">空化阶段</span>
                <span class="value status-${bladeData.stage}">${stageNames[bladeData.stage]}</span>
            </div>
            <div class="detail-item">
                <span class="label">累计损伤</span>
                <span class="value">${(bladeData.damage * 100).toFixed(3)}%</span>
            </div>
            <div class="detail-item">
                <span class="label">振动应力</span>
                <span class="value">${bladeData.stress.toFixed(1)} MPa</span>
            </div>
        `;

        if (this.trendChart) {
            const trendData = [];
            for (let i = 0; i < 60; i++) {
                const t = i / 60;
                let val = bladeData.intensity * (0.7 + Math.sin(t * Math.PI * 3) * 0.2 + Math.random() * 0.15);
                trendData.push(Math.max(0, Math.min(1, val)));
            }
            this.trendChart.render(trendData);
        }
    }

    async loadAlarms() {
        try {
            const response = await fetch(`${this.apiBase}/alarms/active`);
            if (response.ok) {
                this.alarms = await response.json();
            } else {
                this.generateMockAlarms();
            }
        } catch (e) {
            this.generateMockAlarms();
        }
        this.renderAlarms();
    }

    generateMockAlarms() {
        this.alarms = [
            {
                id: 'alarm-001',
                turbineId: 1,
                turbineName: '2# 水轮机',
                bladeId: 7,
                type: 'cavitation',
                level: 2,
                value: 0.75,
                threshold: 0.6,
                timestamp: Date.now() - 3600000,
                acknowledged: false,
                message: '叶片 #8 空化强度超限，当前0.75，阈值0.6'
            },
            {
                id: 'alarm-002',
                turbineId: 4,
                turbineName: '5# 水轮机',
                type: 'vibration',
                level: 1,
                value: 0.42,
                threshold: 0.3,
                timestamp: Date.now() - 7200000,
                acknowledged: true,
                message: '振动幅值超标，当前0.42g，阈值0.3g'
            },
            {
                id: 'alarm-003',
                turbineId: 2,
                turbineName: '3# 水轮机',
                bladeId: 12,
                type: 'cavitation',
                level: 3,
                value: 0.92,
                threshold: 0.8,
                timestamp: Date.now() - 1800000,
                acknowledged: false,
                message: '叶片 #13 发展空化，需立即停机检修'
            }
        ];
    }

    renderAlarms(filter = 'all') {
        const container = document.getElementById('alarmList');
        let filtered = this.alarms;

        if (filter === 'active') {
            filtered = this.alarms.filter(a => !a.acknowledged);
        } else if (filter === 'acknowledged') {
            filtered = this.alarms.filter(a => a.acknowledged);
        }

        if (filtered.length === 0) {
            container.innerHTML = '<div class="no-alarms">暂无告警</div>';
            return;
        }

        container.innerHTML = filtered.map(alarm => {
            const levelClasses = ['level-info', 'level-warning', 'level-severe', 'level-critical'];
            const levelNames = ['信息', '警告', '严重', '紧急'];
            const typeNames = { cavitation: '空化', vibration: '振动', life: '寿命' };
            
            const timeStr = new Date(alarm.timestamp).toLocaleString();
            const timeAgo = this.getTimeAgo(alarm.timestamp);

            return `
                <div class="alarm-item ${levelClasses[alarm.level]} ${alarm.acknowledged ? 'acknowledged' : ''}">
                    <div class="alarm-header">
                        <span class="alarm-level">${levelNames[alarm.level]}</span>
                        <span class="alarm-type">${typeNames[alarm.type] || alarm.type}</span>
                        <span class="alarm-time">${timeAgo}</span>
                    </div>
                    <div class="alarm-body">
                        <div class="alarm-source">${alarm.turbineName}${alarm.bladeId ? ` 叶片 #${alarm.bladeId + 1}` : ''}</div>
                        <div class="alarm-message">${alarm.message}</div>
                        <div class="alarm-values">
                            <span>当前值: ${alarm.value.toFixed(2)}</span>
                            <span>阈值: ${alarm.threshold.toFixed(2)}</span>
                        </div>
                    </div>
                    <div class="alarm-actions">
                        ${!alarm.acknowledged ? 
                            `<button class="btn ack-btn" data-id="${alarm.id}">确认</button>` : 
                            '<span class="ack-text">已确认</span>'}
                        <button class="btn detail-btn" data-id="${alarm.id}">详情</button>
                    </div>
                </div>
            `;
        }).join('');

        container.querySelectorAll('.ack-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                this.acknowledgeAlarm(e.target.dataset.id);
            });
        });

        document.getElementById('activeAlarms').textContent = 
            this.alarms.filter(a => !a.acknowledged).length;
    }

    filterAlarms(filter) {
        document.querySelectorAll('.filter-btn').forEach(btn => {
            btn.classList.toggle('active', btn.dataset.filter === filter);
        });
        this.renderAlarms(filter);
    }

    async acknowledgeAlarm(alarmId) {
        try {
            const response = await fetch(`${this.apiBase}/alarms/acknowledge`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ alarmId, userId: 'web-user' })
            });
        } catch (e) {
            const alarm = this.alarms.find(a => a.id === alarmId);
            if (alarm) alarm.acknowledged = true;
        }
        
        const alarm = this.alarms.find(a => a.id === alarmId);
        if (alarm) alarm.acknowledged = true;
        this.renderAlarms();
    }

    getTimeAgo(timestamp) {
        const diff = Date.now() - timestamp;
        const minutes = Math.floor(diff / 60000);
        const hours = Math.floor(diff / 3600000);
        
        if (hours > 0) return `${hours}小时前`;
        if (minutes > 0) return `${minutes}分钟前`;
        return '刚刚';
    }

    async startDataUpdates() {
        this.updateTurbineData();
        this.updateInterval = setInterval(() => {
            this.updateTurbineData();
            this.updateNewTabMockData();
        }, 1000);
    }

    updateNewTabMockData() {
        this.turbines.forEach(t => {
            if (t.control) {
                t.control.efficiency_pred = Math.max(80, Math.min(98, t.control.efficiency_pred + (Math.random() - 0.5) * 0.4));
                t.control.cav_risk_pred = Math.max(0, Math.min(1, t.control.cav_risk_pred + (Math.random() - 0.5) * 0.04));
            }
        });
        if (this.mpcControlTab && this.mpcControlTab.units) {
            this.mpcControlTab.units.forEach((u, i) => {
                const t = this.turbines[i];
                if (t && t.control) {
                    u.efficiency_pred = t.control.efficiency_pred;
                    u.cav_risk_pred = t.control.cav_risk_pred;
                }
            });
        }
    }

    async getControlStatus() {
        try {
            const response = await fetch(`${this.apiBase}/control/status`);
            if (response.ok) return await response.json();
        } catch (e) {}
        return this.turbines.map(t => ({
            id: t.id,
            guide_vane: t.control?.guide_vane || 50,
            power: t.control?.power || 700,
            mode: t.control?.mode || 'mpc_optimal',
            efficiency_pred: t.control?.efficiency_pred || 90,
            cav_risk_pred: t.control?.cav_risk_pred || 0.3
        }));
    }

    async getRobotTasks() {
        try {
            const response = await fetch(`${this.apiBase}/robot/tasks`);
            if (response.ok) return await response.json();
        } catch (e) {}
        return null;
    }

    async getScheduleCurrent() {
        try {
            const response = await fetch(`${this.apiBase}/schedule/current`);
            if (response.ok) return await response.json();
        } catch (e) {}
        return null;
    }

    async getDiagnosisPatterns() {
        try {
            const response = await fetch(`${this.apiBase}/diagnosis/patterns`);
            if (response.ok) return await response.json();
        } catch (e) {}
        return null;
    }

    async getDiagnosisLatest(limit = 50) {
        try {
            const response = await fetch(`${this.apiBase}/diagnosis/latest?limit=${limit}`);
            if (response.ok) return await response.json();
        } catch (e) {}
        return null;
    }

    async updateTurbineData() {
        try {
            const response = await fetch(`${this.apiBase}/cavitation?turbineId=${this.currentTurbineId}`);
            if (response.ok) {
                const data = await response.json();
                if (data.bladeCavitation) {
                    this.turbineViewer.updateCavitationData(data.bladeCavitation);
                }
            } else {
                this.simulateDataUpdate();
            }
        } catch (e) {
            this.simulateDataUpdate();
        }

        this.updateTurbineInfo();
    }

    simulateDataUpdate() {
        const newData = {};
        for (let i = 0; i < 15; i++) {
            const current = this.turbineViewer.cavitationData[i] || {};
            const change = (Math.random() - 0.5) * 0.02;
            const intensity = Math.max(0, Math.min(1, (current.intensity || 0.1) + change));
            
            let stage = 0;
            if (intensity > 0.8) stage = 3;
            else if (intensity > 0.6) stage = 2;
            else if (intensity > 0.3) stage = 1;

            newData[i] = {
                intensity,
                stage,
                damage: (current.damage || 0.01) + intensity * 0.0001,
                stress: (current.stress || 15) + (Math.random() - 0.5) * 5
            };
        }
        this.turbineViewer.updateCavitationData(newData);

        const turbine = this.turbines[this.currentTurbineId];
        if (turbine) {
            turbine.maxCavitation = Math.max(...Object.values(newData).map(d => d.intensity));
            turbine.maxVibration = 0.05 + Math.random() * 0.2;
            turbine.cavitationStage = Math.max(...Object.values(newData).map(d => d.stage));
            
            if (turbine.remainingLife > 90) {
                turbine.remainingLife -= 0.001;
            }
        }

        document.getElementById('cumulativeDamage').textContent = 
            Object.values(newData).reduce((sum, d) => sum + d.damage, 0).toFixed(4);
        document.getElementById('damageRate').textContent = 
            (Object.values(newData).reduce((sum, d) => sum + d.intensity, 0) * 1e-6 / 15).toExponential(2) + ' /h';
        document.getElementById('remainingTime').textContent = 
            (turbine.remainingLife / 10).toFixed(1) + ' 年';
    }

    startWaterfallUpdates() {
        if (this.waterfallInterval) {
            clearInterval(this.waterfallInterval);
        }
        this.waterfallInterval = setInterval(() => {
            this.waterfallChart.addTestData();
        }, 200);
    }

    startTimeUpdate() {
        const update = () => {
            document.getElementById('systemTime').textContent = 
                new Date().toLocaleTimeString('zh-CN');
        };
        update();
        setInterval(update, 1000);
    }
}

class TrendChart {
    constructor(canvas) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.width = canvas.width;
        this.height = canvas.height;
    }

    render(data) {
        const ctx = this.ctx;
        const width = this.width;
        const height = this.height;

        ctx.fillStyle = '#0a0f1a';
        ctx.fillRect(0, 0, width, height);

        if (data.length < 2) return;

        const padding = { left: 35, right: 10, top: 10, bottom: 25 };
        const chartWidth = width - padding.left - padding.right;
        const chartHeight = height - padding.top - padding.bottom;

        const maxVal = Math.max(...data) * 1.1;
        const minVal = 0;

        ctx.strokeStyle = '#222';
        ctx.lineWidth = 0.5;
        for (let i = 0; i <= 3; i++) {
            const y = padding.top + (i / 3) * chartHeight;
            ctx.beginPath();
            ctx.moveTo(padding.left, y);
            ctx.lineTo(width - padding.right, y);
            ctx.stroke();

            ctx.fillStyle = '#666';
            ctx.font = '10px sans-serif';
            ctx.textAlign = 'right';
            ctx.fillText(((maxVal - minVal) * (1 - i / 3) * 100).toFixed(0) + '%', padding.left - 3, y + 3);
        }

        const gradient = ctx.createLinearGradient(0, padding.top, 0, height - padding.bottom);
        gradient.addColorStop(0, 'rgba(255, 100, 100, 0.6)');
        gradient.addColorStop(0.5, 'rgba(255, 200, 50, 0.4)');
        gradient.addColorStop(1, 'rgba(50, 200, 100, 0.2)');

        ctx.beginPath();
        ctx.moveTo(padding.left, height - padding.bottom);
        for (let i = 0; i < data.length; i++) {
            const x = padding.left + (i / (data.length - 1)) * chartWidth;
            const y = padding.top + chartHeight - ((data[i] - minVal) / (maxVal - minVal)) * chartHeight;
            ctx.lineTo(x, y);
        }
        ctx.lineTo(width - padding.right, height - padding.bottom);
        ctx.closePath();
        ctx.fillStyle = gradient;
        ctx.fill();

        ctx.beginPath();
        for (let i = 0; i < data.length; i++) {
            const x = padding.left + (i / (data.length - 1)) * chartWidth;
            const y = padding.top + chartHeight - ((data[i] - minVal) / (maxVal - minVal)) * chartHeight;
            
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.strokeStyle = '#00d4ff';
        ctx.lineWidth = 1.5;
        ctx.stroke();

        ctx.fillStyle = '#666';
        ctx.font = '10px sans-serif';
        ctx.textAlign = 'center';
        for (let i = 0; i <= 3; i++) {
            const x = padding.left + (i / 3) * chartWidth;
            ctx.fillText(`${-60 + i * 20}s`, x, height - padding.bottom + 15);
        }

        ctx.fillStyle = '#888';
        ctx.fillText('时间', width / 2, height - 3);
    }
}

document.addEventListener('DOMContentLoaded', () => {
    window.app = new TurbineMonitorApp();
});
