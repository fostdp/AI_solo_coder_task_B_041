class RobotRepairTab {
    constructor(app) {
        this.app = app;
        this.container = null;
        this.viewer = null;
        this.tasks = [];
        this.robotStatus = {
            state: 'idle',
            waypointIndex: 0,
            position: { x: 0, y: 0, z: 0 },
            speed: 0,
            attitude: { roll: 0, pitch: 0, yaw: 0 },
            torque: { x: 0, y: 0, z: 0 },
            progress: 0
        };
        this.waypointPath = [];
        this.robotPosIndex = 0;
        this.robotPosSubIndex = 0;
        this.simInterval = null;
        this.currentTurbineId = 0;
        this.initMockData();
    }

    initMockData() {
        this.tasks = [
            {
                id: 'task-001',
                turbineId: 0,
                turbineName: '1# 水轮机',
                targetBlade: '叶片 #5',
                status: 'planning',
                estDuration: '2.5h',
                repairArea: '0.82 m²',
                weldMaterial: '3.2 kg',
                damageLevel: 0.72
            },
            {
                id: 'task-002',
                turbineId: 1,
                turbineName: '2# 水轮机',
                targetBlade: '叶片 #12',
                status: 'running',
                estDuration: '4.1h',
                repairArea: '1.45 m²',
                weldMaterial: '6.8 kg',
                progress: 35,
                damageLevel: 0.88
            },
            {
                id: 'task-003',
                turbineId: 2,
                turbineName: '3# 水轮机',
                targetBlade: '叶片 #3, #9',
                status: 'pending',
                estDuration: '5.8h',
                repairArea: '2.10 m²',
                weldMaterial: '9.5 kg',
                damageLevel: 0.65
            },
            {
                id: 'task-004',
                turbineId: 4,
                turbineName: '5# 水轮机',
                targetBlade: '叶片 #7',
                status: 'completed',
                estDuration: '1.8h',
                repairArea: '0.55 m²',
                weldMaterial: '2.1 kg',
                damageLevel: 0.45,
                completedAt: Date.now() - 3600000
            }
        ];

        for (let i = 0; i <= 30; i++) {
            const t = i / 30;
            const angle = t * Math.PI * 2.2;
            const radius = 0.3 + t * 0.4;
            this.waypointPath.push({
                x: Math.cos(angle) * radius,
                y: Math.sin(angle) * radius * 0.6,
                z: (t - 0.5) * 0.8
            });
        }
    }

    render(container) {
        this.container = container;
        container.innerHTML = '';
        container.style.cssText = 'flex:1;display:flex;flex-direction:column;padding:12px;gap:12px;background:#050a14;overflow:hidden;';

        const topPanel = this.createTopPanel();
        const bottomPanel = this.createBottomPanel();

        container.appendChild(topPanel);
        container.appendChild(bottomPanel);

        setTimeout(() => {
            this.initViewer();
            this.startSimulation();
        }, 100);
    }

    createTopPanel() {
        const panel = document.createElement('div');
        panel.style.cssText = 'flex:1;display:flex;background:rgba(10,22,40,0.5);border:1px solid #1e3a5f;border-radius:8px;overflow:hidden;min-height:0;';

        const viewWrap = document.createElement('div');
        viewWrap.style.cssText = 'flex:1;position:relative;min-width:0;';

        viewWrap.innerHTML = `
            <div style="position:absolute;top:8px;left:12px;z-index:10;display:flex;gap:8px;align-items:center;">
                <h3 style="font-size:13px;color:#7a8ca3;margin:0;text-transform:uppercase;letter-spacing:0.5px;">转轮剖面三维视图</h3>
                <select id="turbineSelectRobot" style="padding:4px 10px;background:rgba(10,22,40,0.8);border:1px solid #1e3a5f;border-radius:4px;color:#e0e6ed;font-size:11px;">
                    ${[0,1,2,3,4,5].map(i => `<option value="${i}" ${i===this.currentTurbineId?'selected':''}>${i+1}# 水轮机</option>`).join('')}
                </select>
            </div>
            <div style="position:absolute;top:8px;right:12px;z-index:10;display:flex;gap:16px;font-size:10px;">
                <div style="display:flex;align-items:center;gap:4px;"><span style="width:12px;height:3px;background:#2196f3;"></span><span style="color:#7a8ca3;">路径</span></div>
                <div style="display:flex;align-items:center;gap:4px;"><span style="width:10px;height:10px;background:#00ff88;border-radius:50%;box-shadow:0 0 8px #00ff88;"></span><span style="color:#7a8ca3;">机器人</span></div>
                <div style="display:flex;align-items:center;gap:4px;"><span style="width:12px;height:12px;background:linear-gradient(90deg,#2196f3,#ffc107,#f44336);"></span><span style="color:#7a8ca3;">损伤</span></div>
            </div>
            <canvas id="robot3dCanvas" style="width:100%;height:100%;display:block;"></canvas>
            <svg id="overlaySvg" style="position:absolute;inset:0;width:100%;height:100%;pointer-events:none;"></svg>
            <div style="position:absolute;bottom:12px;left:12px;z-index:10;background:rgba(10,22,40,0.85);border:1px solid #1e3a5f;border-radius:6px;padding:8px 12px;font-size:11px;">
                <div style="color:#7a8ca3;margin-bottom:3px;">机器人位置</div>
                <div style="font-family:monospace;color:#00d4ff;">
                    X: <span id="robotX">0.000</span> 
                    Y: <span id="robotY">0.000</span> 
                    Z: <span id="robotZ">0.000</span> m
                </div>
                <div style="color:#7a8ca3;margin-top:5px;margin-bottom:3px;">航点: <span id="wpIndex" style="color:#00ff88;">0</span> / ${this.waypointPath.length}</div>
            </div>
        `;

        panel.appendChild(viewWrap);

        setTimeout(() => {
            const sel = panel.querySelector('#turbineSelectRobot');
            if (sel) {
                sel.addEventListener('change', (e) => {
                    this.currentTurbineId = parseInt(e.target.value);
                });
            }
        }, 50);

        return panel;
    }

    createBottomPanel() {
        const panel = document.createElement('div');
        panel.style.cssText = 'height:300px;display:flex;gap:12px;min-height:260px;';

        const leftPanel = this.createTaskListPanel();
        const rightPanel = this.createStatusPanel();

        panel.appendChild(leftPanel);
        panel.appendChild(rightPanel);
        return panel;
    }

    createTaskListPanel() {
        const panel = document.createElement('div');
        panel.style.cssText = 'flex:1;display:flex;flex-direction:column;background:rgba(30,58,95,0.5);border:1px solid #1e3a5f;border-radius:8px;padding:12px;overflow:hidden;min-width:0;';

        panel.innerHTML = `
            <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:10px;">
                <h3 style="font-size:13px;color:#7a8ca3;margin:0;text-transform:uppercase;letter-spacing:0.5px;">检修任务列表</h3>
                <button id="planBtn" style="padding:6px 14px;background:rgba(0,212,255,0.15);border:1px solid #00d4ff;border-radius:4px;color:#00d4ff;font-size:11px;cursor:pointer;">+ 新建规划</button>
            </div>
            <div id="taskList" style="flex:1;overflow-y:auto;display:flex;flex-direction:column;gap:8px;padding-right:4px;"></div>
        `;

        setTimeout(() => {
            this.renderTaskList(panel);
            const planBtn = panel.querySelector('#planBtn');
            if (planBtn) planBtn.addEventListener('click', () => this.planNewTask());
        }, 50);

        return panel;
    }

    renderTaskList(panel) {
        const list = panel.querySelector('#taskList');
        if (!list) return;

        const statusConfig = {
            planning:   { label: '规划中',   color: '#2196f3', bg: 'rgba(33,150,243,0.15)' },
            pending:    { label: '待执行',   color: '#ffc107', bg: 'rgba(255,193,7,0.15)' },
            running:    { label: '执行中',   color: '#00ff88', bg: 'rgba(0,255,136,0.15)' },
            completed:  { label: '已完成',   color: '#4caf50', bg: 'rgba(76,175,80,0.15)' },
            cancelled:  { label: '已取消',   color: '#9e9e9e', bg: 'rgba(158,158,158,0.15)' },
            failed:     { label: '故障',     color: '#f44336', bg: 'rgba(244,67,54,0.15)' }
        };

        list.innerHTML = this.tasks.map(task => {
            const sc = statusConfig[task.status] || statusConfig.pending;
            const canStart = task.status === 'pending' || task.status === 'planning';
            const canCancel = task.status === 'running' || task.status === 'planning';
            const showProgress = task.status === 'running';
            const heatColor = `hsl(${120 - task.damageLevel * 120}, 70%, 50%)`;

            return `
                <div style="background:rgba(10,22,40,0.6);border:1px solid #1e3a5f;border-radius:6px;padding:10px;${task.status === 'completed' ? 'opacity:0.6;' : ''}">
                    <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:8px;">
                        <div style="display:flex;align-items:center;gap:8px;">
                            <span style="font-weight:600;font-size:12px;color:#e0e6ed;">${task.turbineName}</span>
                            <span style="font-size:10px;padding:1px 8px;border-radius:10px;background:${sc.bg};color:${sc.color};border:1px solid ${sc.color}44;">${sc.label}</span>
                        </div>
                        <span style="font-size:10px;color:#555;">${task.id}</span>
                    </div>
                    <div style="display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin-bottom:8px;font-size:11px;">
                        <div>
                            <div style="color:#555;">目标</div>
                            <div style="color:#e0e6ed;">${task.targetBlade}</div>
                        </div>
                        <div>
                            <div style="color:#555;">时长</div>
                            <div style="color:#00d4ff;font-family:monospace;">${task.estDuration}</div>
                        </div>
                        <div>
                            <div style="color:#555;">面积</div>
                            <div style="color:#e0e6ed;font-family:monospace;">${task.repairArea}</div>
                        </div>
                        <div>
                            <div style="color:#555;">焊材</div>
                            <div style="color:#ffc107;font-family:monospace;">${task.weldMaterial}</div>
                        </div>
                    </div>
                    <div style="margin-bottom:8px;">
                        <div style="display:flex;justify-content:space-between;font-size:10px;color:#555;margin-bottom:3px;">
                            <span>损伤程度</span>
                            <span style="color:${heatColor};font-family:monospace;">${(task.damageLevel*100).toFixed(0)}%</span>
                        </div>
                        <div style="height:4px;background:#1e3a5f;border-radius:2px;overflow:hidden;">
                            <div style="height:100%;width:${task.damageLevel*100}%;background:linear-gradient(90deg,#4caf50,#ffc107,#f44336);"></div>
                        </div>
                    </div>
                    ${showProgress ? `
                    <div style="margin-bottom:8px;">
                        <div style="display:flex;justify-content:space-between;font-size:10px;color:#555;margin-bottom:3px;">
                            <span>执行进度</span>
                            <span style="color:#00ff88;font-family:monospace;">${task.progress}%</span>
                        </div>
                        <div style="height:4px;background:#1e3a5f;border-radius:2px;overflow:hidden;">
                            <div style="height:100%;width:${task.progress}%;background:linear-gradient(90deg,#00d4ff,#00ff88);transition:width 0.3s;"></div>
                        </div>
                    </div>
                    ` : ''}
                    <div style="display:flex;gap:6px;">
                        ${canStart ? `<button class="start-task-btn" data-id="${task.id}" style="flex:1;padding:5px;background:rgba(0,255,136,0.15);border:1px solid #00ff88;color:#00ff88;font-size:11px;border-radius:4px;cursor:pointer;">开始</button>` : ''}
                        ${canCancel ? `<button class="cancel-task-btn" data-id="${task.id}" style="flex:1;padding:5px;background:rgba(244,67,54,0.15);border:1px solid #f44336;color:#f44336;font-size:11px;border-radius:4px;cursor:pointer;">取消</button>` : ''}
                        ${task.status === 'completed' ? `<div style="flex:1;padding:5px;text-align:center;color:#4caf50;font-size:11px;">✓ 完成于 ${new Date(task.completedAt).toLocaleTimeString()}</div>` : ''}
                    </div>
                </div>
            `;
        }).join('');

        list.querySelectorAll('.start-task-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const id = e.target.dataset.id;
                const task = this.tasks.find(t => t.id === id);
                if (task) {
                    task.status = 'running';
                    task.progress = 0;
                    this.robotStatus.state = 'deploy';
                    this.renderTaskList(panel);
                    this.updateRobotStatusDisplay();
                }
            });
        });

        list.querySelectorAll('.cancel-task-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const id = e.target.dataset.id;
                this.cancelTask(id);
            });
        });
    }

    createStatusPanel() {
        const panel = document.createElement('div');
        panel.style.cssText = 'width:360px;display:flex;flex-direction:column;background:rgba(30,58,95,0.5);border:1px solid #1e3a5f;border-radius:8px;padding:12px;overflow:hidden;';

        const stateConfig = {
            idle:       { label: '待机',       color: '#9e9e9e' },
            planning:   { label: '规划中',     color: '#2196f3' },
            deploy:     { label: '部署中',     color: '#ff9800' },
            inspect:    { label: '巡检中',     color: '#00d4ff' },
            grinding:   { label: '打磨中',     color: '#ffc107' },
            welding:    { label: '焊接中',     color: '#f44336' },
            returning:  { label: '返航中',     color: '#9c27b0' },
            completed:  { label: '任务完成',   color: '#00ff88' },
            fault:      { label: '故障',       color: '#ff4757' }
        };
        const sc = stateConfig[this.robotStatus.state] || stateConfig.idle;

        panel.innerHTML = `
            <h3 style="font-size:13px;color:#7a8ca3;margin:0 0 10px 0;text-transform:uppercase;letter-spacing:0.5px;">机器人状态</h3>
            <div style="background:rgba(10,22,40,0.6);border:1px solid #1e3a5f;border-radius:6px;padding:10px;margin-bottom:10px;">
                <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:8px;">
                    <span style="font-size:12px;color:#7a8ca3;">当前状态</span>
                    <span id="robotStateBadge" style="font-size:11px;padding:3px 10px;border-radius:10px;background:${sc.color}22;color:${sc.color};border:1px solid ${sc.color}44;font-weight:600;">${sc.label}</span>
                </div>
                <div style="margin-bottom:4px;">
                    <div style="display:flex;justify-content:space-between;font-size:10px;color:#555;margin-bottom:3px;">
                        <span>任务进度</span>
                        <span id="robotProgressVal" style="color:#00d4ff;font-family:monospace;">${this.robotStatus.progress}%</span>
                    </div>
                    <div style="height:6px;background:#1e3a5f;border-radius:3px;overflow:hidden;">
                        <div id="robotProgressBar" style="height:100%;width:${this.robotStatus.progress}%;background:linear-gradient(90deg,#00d4ff,#00ff88);"></div>
                    </div>
                </div>
            </div>
            <div style="display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-bottom:10px;">
                <div style="background:rgba(10,22,40,0.6);border:1px solid #1e3a5f;border-radius:6px;padding:8px;">
                    <div style="font-size:10px;color:#555;margin-bottom:3px;">速度</div>
                    <div id="speedGaugeWrap"></div>
                    <div id="speedVal" style="text-align:center;font-family:monospace;color:#00d4ff;font-size:12px;font-weight:600;">0.0 m/s</div>
                </div>
                <div style="background:rgba(10,22,40,0.6);border:1px solid #1e3a5f;border-radius:6px;padding:8px;">
                    <div style="font-size:10px;color:#555;margin-bottom:3px;">姿态</div>
                    <div id="attitudeGaugeWrap"></div>
                    <div style="text-align:center;font-family:monospace;color:#ffc107;font-size:9px;" id="attitudeVals">R:0° P:0° Y:0°</div>
                </div>
                <div style="background:rgba(10,22,40,0.6);border:1px solid #1e3a5f;border-radius:6px;padding:8px;">
                    <div style="font-size:10px;color:#555;margin-bottom:3px;">力矩</div>
                    <div id="torqueGaugeWrap"></div>
                    <div style="text-align:center;font-family:monospace;color:#9c27b0;font-size:9px;" id="torqueVals">X:0 Y:0 Z:0</div>
                </div>
            </div>
            <div style="background:rgba(10,22,40,0.6);border:1px solid #1e3a5f;border-radius:6px;padding:10px;flex:1;overflow-y:auto;">
                <div style="font-size:11px;color:#7a8ca3;margin-bottom:6px;">实时坐标 (3D)</div>
                <div style="display:grid;grid-template-columns:repeat(3,1fr);gap:6px;margin-bottom:10px;">
                    <div>
                        <div style="font-size:9px;color:#555;">X</div>
                        <div id="coordX" style="font-family:monospace;color:#00d4ff;font-size:13px;">0.000</div>
                    </div>
                    <div>
                        <div style="font-size:9px;color:#555;">Y</div>
                        <div id="coordY" style="font-family:monospace;color:#00ff88;font-size:13px;">0.000</div>
                    </div>
                    <div>
                        <div style="font-size:9px;color:#555;">Z</div>
                        <div id="coordZ" style="font-family:monospace;color:#ffc107;font-size:13px;">0.000</div>
                    </div>
                </div>
                <div style="font-size:11px;color:#7a8ca3;margin-bottom:6px;">当前航点</div>
                <div style="display:flex;align-items:center;gap:8px;">
                    <div style="font-family:monospace;color:#e0e6ed;font-size:14px;">WP #<span id="wpNum" style="color:#00d4ff;">0</span></div>
                    <div style="flex:1;height:3px;background:#1e3a5f;border-radius:2px;overflow:hidden;">
                        <div id="wpProgress" style="height:100%;width:0%;background:linear-gradient(90deg,#2196f3,#00ff88);"></div>
                    </div>
                    <div id="wpTotal" style="font-family:monospace;color:#555;font-size:11px;">/ ${this.waypointPath.length}</div>
                </div>
            </div>
        `;

        setTimeout(() => this.initMiniGauges(panel), 50);
        return panel;
    }

    initMiniGauges(panel) {
        const speedWrap = panel.querySelector('#speedGaugeWrap');
        const attWrap = panel.querySelector('#attitudeGaugeWrap');
        const torWrap = panel.querySelector('#torqueGaugeWrap');

        if (speedWrap) {
            speedWrap.innerHTML = `
                <svg viewBox="0 0 60 35" width="100%" height="32">
                    <path d="M8 30 A 22 22 0 0 1 52 30" fill="none" stroke="#1e3a5f" stroke-width="4"/>
                    <path id="speedArc" d="M8 30 A 22 22 0 0 1 8 30" fill="none" stroke="#00d4ff" stroke-width="4" stroke-linecap="round"/>
                </svg>
            `;
        }
        if (attWrap) {
            attWrap.innerHTML = `
                <svg viewBox="0 0 60 35" width="100%" height="32">
                    <circle cx="30" cy="18" r="12" fill="none" stroke="#1e3a5f" stroke-width="2"/>
                    <line id="attRoll" x1="30" y1="18" x2="30" y2="8" stroke="#ffc107" stroke-width="2" stroke-linecap="round"/>
                    <line id="attPitch" x1="30" y1="18" x2="40" y2="18" stroke="#ff9800" stroke-width="1.5" stroke-linecap="round" opacity="0.7"/>
                </svg>
            `;
        }
        if (torWrap) {
            torWrap.innerHTML = `
                <svg viewBox="0 0 60 35" width="100%" height="32">
                    <path d="M8 30 A 22 22 0 0 1 52 30" fill="none" stroke="#1e3a5f" stroke-width="3"/>
                    <path id="torArc" d="M8 30 A 22 22 0 0 1 8 30" fill="none" stroke="#9c27b0" stroke-width="3" stroke-linecap="round"/>
                </svg>
            `;
        }
    }

    initViewer() {
        const canvas = document.getElementById('robot3dCanvas');
        if (!canvas) return;
        this.viewer = new Simple3DViewer(canvas);
        this.viewer.setDamageMap(this.generateDamageMap());
        this.viewer.setWaypoints(this.waypointPath);
    }

    generateDamageMap() {
        const map = {};
        for (let b = 0; b < 15; b++) {
            map[b] = [];
            for (let s = 0; s < 20; s++) {
                const x = s / 19;
                const base = Math.exp(-Math.pow((x - 0.6) * 3, 2)) * 0.8;
                map[b].push(Math.max(0, Math.min(1, base * (0.5 + Math.random() * 0.5) + (b === 4 ? 0.3 : 0))));
            }
        }
        return map;
    }

    startSimulation() {
        if (this.simInterval) clearInterval(this.simInterval);
        this.simInterval = setInterval(() => {
            this.advanceRobot();
            this.updateTaskProgress();
            this.updateDisplays();
        }, 100);
    }

    advanceRobot() {
        const total = this.waypointPath.length;
        if (total < 2) return;

        this.robotPosSubIndex += 0.05;
        if (this.robotPosSubIndex >= 1) {
            this.robotPosSubIndex = 0;
            this.robotPosIndex = (this.robotPosIndex + 1) % (total - 1);
        }

        const i = this.robotPosIndex;
        const t = this.robotPosSubIndex;
        const p1 = this.waypointPath[i];
        const p2 = this.waypointPath[i + 1];

        this.robotStatus.position = {
            x: p1.x + (p2.x - p1.x) * t,
            y: p1.y + (p2.y - p1.y) * t,
            z: p1.z + (p2.z - p1.z) * t
        };

        this.robotStatus.waypointIndex = i;
        this.robotStatus.speed = 0.8 + Math.sin(Date.now() / 500) * 0.2;
        this.robotStatus.attitude = {
            roll: Math.sin(Date.now() / 700) * 8,
            pitch: Math.cos(Date.now() / 900) * 5,
            yaw: (Math.atan2(p2.y - p1.y, p2.x - p1.x) * 180 / Math.PI) % 360
        };
        this.robotStatus.torque = {
            x: 5 + Math.sin(Date.now() / 300) * 3,
            y: 4 + Math.cos(Date.now() / 400) * 2.5,
            z: 6 + Math.sin(Date.now() / 500) * 3.5
        };

        if (this.viewer) {
            this.viewer.setRobotPosition(this.robotStatus.position);
        }
    }

    updateTaskProgress() {
        const runningTask = this.tasks.find(t => t.status === 'running');
        if (runningTask) {
            runningTask.progress = Math.min(100, runningTask.progress + 0.1);
            this.robotStatus.progress = runningTask.progress;
            if (runningTask.progress < 15) this.robotStatus.state = 'deploy';
            else if (runningTask.progress < 35) this.robotStatus.state = 'inspect';
            else if (runningTask.progress < 60) this.robotStatus.state = 'grinding';
            else if (runningTask.progress < 90) this.robotStatus.state = 'welding';
            else if (runningTask.progress < 100) this.robotStatus.state = 'returning';
            else {
                runningTask.status = 'completed';
                runningTask.completedAt = Date.now();
                this.robotStatus.state = 'completed';
                setTimeout(() => { if(this.container) this.renderTaskList(this.container); }, 100);
            }
        } else if (this.robotStatus.state !== 'fault' && this.robotStatus.state !== 'completed') {
            this.robotStatus.state = 'idle';
            this.robotStatus.progress = Math.max(0, this.robotStatus.progress - 0.05);
        }
    }

    updateDisplays() {
        const rx = document.getElementById('robotX');
        const ry = document.getElementById('robotY');
        const rz = document.getElementById('robotZ');
        if (rx) rx.textContent = this.robotStatus.position.x.toFixed(3);
        if (ry) ry.textContent = this.robotStatus.position.y.toFixed(3);
        if (rz) rz.textContent = this.robotStatus.position.z.toFixed(3);

        const cx = document.getElementById('coordX');
        const cy = document.getElementById('coordY');
        const cz = document.getElementById('coordZ');
        if (cx) cx.textContent = this.robotStatus.position.x.toFixed(3);
        if (cy) cy.textContent = this.robotStatus.position.y.toFixed(3);
        if (cz) cz.textContent = this.robotStatus.position.z.toFixed(3);

        const wpi = document.getElementById('wpIndex');
        const wpn = document.getElementById('wpNum');
        const wpp = document.getElementById('wpProgress');
        if (wpi) wpi.textContent = this.robotStatus.waypointIndex;
        if (wpn) wpn.textContent = this.robotStatus.waypointIndex;
        if (wpp) wpp.style.width = ((this.robotStatus.waypointIndex + this.robotPosSubIndex) / (this.waypointPath.length - 1) * 100) + '%';

        this.updateRobotStatusDisplay();
        this.updateGauges();

        const sv = document.getElementById('speedVal');
        if (sv) sv.textContent = this.robotStatus.speed.toFixed(2) + ' m/s';

        const av = document.getElementById('attitudeVals');
        if (av) av.textContent = `R:${this.robotStatus.attitude.roll.toFixed(1)}° P:${this.robotStatus.attitude.pitch.toFixed(1)}° Y:${this.robotStatus.attitude.yaw.toFixed(1)}°`;

        const tv = document.getElementById('torqueVals');
        if (tv) tv.textContent = `X:${this.robotStatus.torque.x.toFixed(1)} Y:${this.robotStatus.torque.y.toFixed(1)} Z:${this.robotStatus.torque.z.toFixed(1)} N·m`;
    }

    updateRobotStatusDisplay() {
        const stateConfig = {
            idle:       { label: '待机',       color: '#9e9e9e' },
            planning:   { label: '规划中',     color: '#2196f3' },
            deploy:     { label: '部署中',     color: '#ff9800' },
            inspect:    { label: '巡检中',     color: '#00d4ff' },
            grinding:   { label: '打磨中',     color: '#ffc107' },
            welding:    { label: '焊接中',     color: '#f44336' },
            returning:  { label: '返航中',     color: '#9c27b0' },
            completed:  { label: '任务完成',   color: '#00ff88' },
            fault:      { label: '故障',       color: '#ff4757' }
        };
        const sc = stateConfig[this.robotStatus.state] || stateConfig.idle;

        const badge = document.getElementById('robotStateBadge');
        if (badge) {
            badge.textContent = sc.label;
            badge.style.background = sc.color + '22';
            badge.style.color = sc.color;
            badge.style.borderColor = sc.color + '44';
        }

        const pv = document.getElementById('robotProgressVal');
        const pb = document.getElementById('robotProgressBar');
        if (pv) pv.textContent = this.robotStatus.progress.toFixed(0) + '%';
        if (pb) pb.style.width = this.robotStatus.progress + '%';
    }

    updateGauges() {
        const speedArc = document.getElementById('speedArc');
        if (speedArc) {
            const frac = Math.min(1, this.robotStatus.speed / 2);
            const ang = frac * Math.PI;
            const cx = 30, cy = 30, r = 22;
            const x2 = cx + Math.cos(Math.PI - ang) * (-r);
            const y2 = cy + Math.sin(Math.PI - ang) * (-r);
            const ex = cx + r * Math.cos(Math.PI + ang);
            const ey = cy - r * Math.sin(ang);
            speedArc.setAttribute('d', `M${cx-r} ${cy} A ${r} ${r} 0 0 1 ${ex} ${ey}`);
        }

        const attRoll = document.getElementById('attRoll');
        if (attRoll) {
            const cx = 30, cy = 18, r = 12;
            const ang = (this.robotStatus.attitude.roll * Math.PI / 180);
            attRoll.setAttribute('x2', cx + Math.sin(ang) * r);
            attRoll.setAttribute('y2', cy - Math.cos(ang) * r);
        }

        const torArc = document.getElementById('torArc');
        if (torArc) {
            const tmag = Math.sqrt(this.robotStatus.torque.x**2 + this.robotStatus.torque.y**2 + this.robotStatus.torque.z**2);
            const frac = Math.min(1, tmag / 20);
            const cx = 30, cy = 30, r = 22;
            const ang = frac * Math.PI;
            const ex = cx + r * Math.cos(Math.PI + ang);
            const ey = cy - r * Math.sin(ang);
            torArc.setAttribute('d', `M${cx-r} ${cy} A ${r} ${r} 0 0 1 ${ex} ${ey}`);
        }
    }

    async planNewTask() {
        try {
            await fetch(`${this.app.apiBase}/robot/plan/${this.currentTurbineId}`, { method: 'POST' });
        } catch (e) {}

        const newTask = {
            id: 'task-' + String(Date.now()).slice(-6),
            turbineId: this.currentTurbineId,
            turbineName: `${this.currentTurbineId + 1}# 水轮机`,
            targetBlade: `叶片 #${Math.floor(Math.random() * 15) + 1}`,
            status: 'planning',
            estDuration: (1.5 + Math.random() * 5).toFixed(1) + 'h',
            repairArea: (0.3 + Math.random() * 2).toFixed(2) + ' m²',
            weldMaterial: (1 + Math.random() * 10).toFixed(1) + ' kg',
            damageLevel: 0.3 + Math.random() * 0.6
        };

        setTimeout(() => {
            newTask.status = 'pending';
            this.robotStatus.state = 'idle';
            if (this.container) this.renderTaskList(this.container);
        }, 2000);

        this.tasks.unshift(newTask);
        this.robotStatus.state = 'planning';
        if (this.container) this.renderTaskList(this.container);
        this.updateRobotStatusDisplay();
    }

    async cancelTask(taskId) {
        try {
            await fetch(`${this.app.apiBase}/robot/cancel/${taskId}`, { method: 'POST' });
        } catch (e) {}
        const task = this.tasks.find(t => t.id === taskId);
        if (task) {
            task.status = 'cancelled';
        }
        if (this.robotStatus.state !== 'completed') this.robotStatus.state = 'idle';
        if (this.container) this.renderTaskList(this.container);
        this.updateRobotStatusDisplay();
    }

    resize() {
        if (this.viewer) this.viewer.resize();
    }
}

class Simple3DViewer {
    constructor(canvas) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.damageMap = {};
        this.waypoints = [];
        this.robotPos = { x: 0, y: 0, z: 0 };
        this.angle = 0;
        this.init();
    }

    init() {
        this.resize();
        window.addEventListener('resize', () => this.resize());
        this.animate();
    }

    resize() {
        const rect = this.canvas.parentElement.getBoundingClientRect();
        this.canvas.width = rect.width * window.devicePixelRatio;
        this.canvas.height = rect.height * window.devicePixelRatio;
        this.canvas.style.width = rect.width + 'px';
        this.canvas.style.height = rect.height + 'px';
        this.w = rect.width;
        this.h = rect.height;
    }

    setDamageMap(map) { this.damageMap = map; }
    setWaypoints(wp) { this.waypoints = wp; }
    setRobotPosition(pos) { this.robotPos = pos; }

    project(x, y, z) {
        const cx = this.w / 2, cy = this.h / 2;
        const scale = Math.min(this.w, this.h) * 0.35;
        const a = this.angle;
        const cosA = Math.cos(a), sinA = Math.sin(a);
        const x1 = x * cosA - z * sinA;
        const z1 = x * sinA + z * cosA;
        const y1 = y;
        const persp = 2.5 / (2.5 + z1);
        return {
            x: cx + x1 * scale * persp,
            y: cy + y1 * scale * persp * 0.8,
            z: z1,
            depth: z1
        };
    }

    animate() {
        this.angle += 0.005;
        this.render();
        requestAnimationFrame(() => this.animate());
    }

    render() {
        const ctx = this.ctx;
        const w = this.w * window.devicePixelRatio;
        const h = this.h * window.devicePixelRatio;
        const dpr = window.devicePixelRatio;
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

        const grad = ctx.createRadialGradient(this.w/2, this.h/2, 0, this.w/2, this.h/2, this.w);
        grad.addColorStop(0, '#0a1628');
        grad.addColorStop(1, '#050a14');
        ctx.fillStyle = grad;
        ctx.fillRect(0, 0, this.w, this.h);

        const blades = 15;
        const bladeShapes = [];
        for (let b = 0; b < blades; b++) {
            const ang = (b / blades) * Math.PI * 2;
            const damage = this.damageMap[b] || Array(20).fill(0.1);
            const segs = [];
            for (let s = 0; s < 19; s++) {
                const t1 = s / 19, t2 = (s + 1) / 19;
                const radIn = 0.25, radOut = 0.85;
                const r1 = radIn + (radOut - radIn) * t1;
                const r2 = radIn + (radOut - radIn) * t2;
                const twist1 = t1 * 0.6;
                const twist2 = t2 * 0.6;
                const thickness = (t1 < 0.05 ? t1 / 0.05 : 1) * (t1 > 0.95 ? (1 - t1) / 0.05 : 1) * 0.04;
                const pts = [
                    this.project(Math.cos(ang + twist1) * r1 + Math.cos(ang + Math.PI/2 + twist1) * thickness,
                                 -0.3 + t1 * 0.6,
                                 Math.sin(ang + twist1) * r1 + Math.sin(ang + Math.PI/2 + twist1) * thickness),
                    this.project(Math.cos(ang + twist2) * r2 + Math.cos(ang + Math.PI/2 + twist2) * thickness,
                                 -0.3 + t2 * 0.6,
                                 Math.sin(ang + twist2) * r2 + Math.sin(ang + Math.PI/2 + twist2) * thickness),
                    this.project(Math.cos(ang + twist2) * r2 - Math.cos(ang + Math.PI/2 + twist2) * thickness,
                                 -0.3 + t2 * 0.6,
                                 Math.sin(ang + twist2) * r2 - Math.sin(ang + Math.PI/2 + twist2) * thickness),
                    this.project(Math.cos(ang + twist1) * r1 - Math.cos(ang + Math.PI/2 + twist1) * thickness,
                                 -0.3 + t1 * 0.6,
                                 Math.sin(ang + twist1) * r1 - Math.sin(ang + Math.PI/2 + twist1) * thickness)
                ];
                const dmg = (damage[s] + damage[s+1]) / 2;
                segs.push({ pts, dmg, depth: pts.reduce((s,p)=>s+p.depth,0)/4 });
            }
            bladeShapes.push(...segs);
        }

        bladeShapes.sort((a, b) => b.depth - a.depth);
        bladeShapes.forEach(s => {
            const d = s.dmg;
            const r = Math.min(1, 0.2 + d * 1.5);
            const g = Math.min(1, 0.6 + (1 - d) * 0.4 - Math.max(0, d - 0.5) * 0.6);
            const bl = Math.min(1, 0.9 + (1 - d) * 0.3 - d * 0.9);
            const depthFactor = 0.5 + (1 - (s.depth + 2) / 4) * 0.5;
            ctx.fillStyle = `rgba(${Math.floor(r*255)},${Math.floor(g*255)},${Math.floor(bl*255)},${0.7 * depthFactor + 0.2})`;
            ctx.strokeStyle = `rgba(${Math.floor(r*220)},${Math.floor(g*220)},${Math.floor(bl*220)},${0.3 * depthFactor})`;
            ctx.lineWidth = 0.5;
            ctx.beginPath();
            s.pts.forEach((p, i) => {
                if (i === 0) ctx.moveTo(p.x, p.y);
                else ctx.lineTo(p.x, p.y);
            });
            ctx.closePath();
            ctx.fill();
            ctx.stroke();
        });

        const hub = [];
        for (let i = 0; i < 32; i++) {
            const a = (i / 32) * Math.PI * 2;
            hub.push(this.project(Math.cos(a) * 0.22, 0, Math.sin(a) * 0.22));
        }
        const hubGrad = ctx.createRadialGradient(this.w/2, this.h/2, 0, this.w/2, this.h/2, this.w*0.2);
        hubGrad.addColorStop(0, '#2d4a6f');
        hubGrad.addColorStop(1, '#1a2d45');
        ctx.fillStyle = hubGrad;
        ctx.strokeStyle = '#00d4ff88';
        ctx.lineWidth = 1;
        ctx.beginPath();
        hub.forEach((p, i) => { if (i === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y); });
        ctx.closePath();
        ctx.fill();
        ctx.stroke();

        if (this.waypoints.length > 1) {
            const proj = this.waypoints.map(p => this.project(p.x * 0.8, p.y * 0.5 - 0.1, p.z * 0.8));
            ctx.strokeStyle = '#2196f3';
            ctx.lineWidth = 2;
            ctx.setLineDash([4, 3]);
            ctx.beginPath();
            proj.forEach((p, i) => { if (i === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y); });
            ctx.stroke();
            ctx.setLineDash([]);

            const hueOffset = (Date.now() / 20) % 360;
            proj.forEach((p, i) => {
                const hue = (i / proj.length * 360 + hueOffset) % 360;
                ctx.fillStyle = `hsla(${hue}, 90%, 60%, 0.8)`;
                ctx.beginPath();
                ctx.arc(p.x, p.y, i % 5 === 0 ? 3 : 1.5, 0, Math.PI * 2);
                ctx.fill();
            });
        }

        const rp = this.project(this.robotPos.x * 0.8, this.robotPos.y * 0.5 - 0.1, this.robotPos.z * 0.8);
        const pulse = 1 + Math.sin(Date.now() / 150) * 0.3;
        ctx.fillStyle = 'rgba(0,255,136,0.15)';
        ctx.beginPath();
        ctx.arc(rp.x, rp.y, 16 * pulse, 0, Math.PI * 2);
        ctx.fill();
        const rg = ctx.createRadialGradient(rp.x, rp.y, 0, rp.x, rp.y, 8);
        rg.addColorStop(0, '#ffffff');
        rg.addColorStop(0.3, '#00ff88');
        rg.addColorStop(1, '#00aa55');
        ctx.fillStyle = rg;
        ctx.beginPath();
        ctx.arc(rp.x, rp.y, 7, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = '#00ff88';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(rp.x, rp.y, 10, 0, Math.PI * 2);
        ctx.stroke();
    }
}
