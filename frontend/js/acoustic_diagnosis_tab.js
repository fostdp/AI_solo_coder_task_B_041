class AcousticDiagnosisTab {
    constructor(app) {
        this.app = app;
        this.container = null;
        this.patterns = [];
        this.tsneSamples = [];
        this.latestDiagnoses = [];
        this.selectedPattern = null;
        this.updateInterval = null;
        this.categoryConfig = {
            cloud:    { name: '云状空化',  color: '#2196f3' },
            sheet:    { name: '片状空化',  color: '#4caf50' },
            super:    { name: '超空化',    color: '#f44336' },
            vortex:   { name: '涡空化',    color: '#ff9800' },
            unknown:  { name: '未知类型',  color: '#9c27b0' },
            other:    { name: '其他',      color: '#78909c' }
        };
        this.initMockData();
    }

    initMockData() {
        const categories = Object.keys(this.categoryConfig);
        for (let i = 0; i < 8; i++) {
            const cat = categories[i % categories.length];
            this.patterns.push({
                id: 'pat-' + String(1001 + i),
                type: cat,
                type_name: this.categoryConfig[cat].name,
                sample_count: 50 + Math.floor(Math.random() * 500),
                silhouette: 0.4 + Math.random() * 0.55,
                expert_validated: Math.random() > 0.3,
                centroid: this.generateEmbedding(cat),
                created_at: Date.now() - Math.random() * 86400000 * 30
            });
        }

        for (let i = 0; i < 180; i++) {
            const cat = categories[Math.floor(Math.random() * categories.length)];
            const base = this.generateEmbedding(cat);
            this.tsneSamples.push({
                id: i,
                category: cat,
                category_name: this.categoryConfig[cat].name,
                x: base[0] + (Math.random() - 0.5) * 0.08,
                y: base[1] + (Math.random() - 0.5) * 0.08,
                embedding: base.map(v => v + (Math.random() - 0.5) * 0.15),
                pattern_id: this.patterns.find(p => p.type === cat)?.id,
                turbine_id: Math.floor(Math.random() * 6),
                sensor_id: `H${String(Math.floor(Math.random() * 12) + 1).padStart(2, '0')}`
            });
        }

        for (let i = 0; i < 30; i++) {
            const cat = categories[Math.floor(Math.random() * categories.length)];
            const known = Math.random() > 0.2;
            this.latestDiagnoses.push({
                id: 'diag-' + String(Date.now() - i * 60000).slice(-8),
                turbine_id: Math.floor(Math.random() * 6),
                turbine_name: `${Math.floor(Math.random() * 6) + 1}# 水轮机`,
                sensor_id: `H${String(Math.floor(Math.random() * 12) + 1).padStart(2, '0')}`,
                type: cat,
                type_name: this.categoryConfig[cat].name,
                confidence: 0.65 + Math.random() * 0.34,
                timestamp: Date.now() - i * 60000 - Math.random() * 30000,
                is_known: known,
                pattern_id: known ? this.patterns[Math.floor(Math.random() * this.patterns.length)].id : null,
                embedding: this.generateEmbedding(cat)
            });
        }
        this.latestDiagnoses.sort((a, b) => b.timestamp - a.timestamp);
    }

    generateEmbedding(category) {
        const catIndex = Object.keys(this.categoryConfig).indexOf(category);
        const baseAngle = (catIndex / Object.keys(this.categoryConfig).length) * Math.PI * 2;
        const baseRadius = 0.4 + (catIndex % 3) * 0.15;
        const pc1 = Math.cos(baseAngle) * baseRadius;
        const pc2 = Math.sin(baseAngle) * baseRadius;
        const emb = new Array(32).fill(0);
        emb[0] = pc1 + (Math.random() - 0.5) * 0.1;
        emb[1] = pc2 + (Math.random() - 0.5) * 0.1;
        for (let i = 2; i < 32; i++) {
            emb[i] = Math.sin(i * 0.5 + baseAngle) * 0.3 * (1 - i / 32) + (Math.random() - 0.5) * 0.15;
        }
        return emb;
    }

    render(container) {
        this.container = container;
        container.innerHTML = '';
        container.style.cssText = 'flex:1;display:grid;grid-template-columns:1fr 1fr;grid-template-rows:1fr 1fr;gap:12px;padding:12px;background:#050a14;overflow:hidden;min-height:0;';

        const tsnePanel = this.createTsnePanel();
        const patternsPanel = this.createPatternsPanel();
        const latestPanel = this.createLatestPanel();
        const analysisPanel = this.createAnalysisPanel();

        tsnePanel.style.gridColumn = '1 / 2'; tsnePanel.style.gridRow = '1 / 2';
        patternsPanel.style.gridColumn = '2 / 3'; patternsPanel.style.gridRow = '1 / 2';
        latestPanel.style.gridColumn = '1 / 2'; latestPanel.style.gridRow = '2 / 3';
        analysisPanel.style.gridColumn = '2 / 3'; analysisPanel.style.gridRow = '2 / 3';

        container.appendChild(tsnePanel);
        container.appendChild(patternsPanel);
        container.appendChild(latestPanel);
        container.appendChild(analysisPanel);

        setTimeout(() => {
            this.renderTsnePlot();
            if (!this.selectedPattern && this.patterns.length > 0) {
                this.selectPattern(this.patterns[0].id);
            }
        }, 100);

        this.startLiveUpdates();
    }

    createPanel(title, extraHeader = '') {
        const panel = document.createElement('div');
        panel.style.cssText = 'background:rgba(30,58,95,0.5);border:1px solid #1e3a5f;border-radius:8px;display:flex;flex-direction:column;overflow:hidden;min-width:0;min-height:0;';
        panel.innerHTML = `
            <div style="display:flex;justify-content:space-between;align-items:center;padding:8px 12px;background:rgba(10,22,40,0.4);border-bottom:1px solid #1e3a5f;flex-shrink:0;">
                <div style="display:flex;align-items:center;gap:8px;">
                    <h4 style="font-size:12px;color:#7a8ca3;margin:0;text-transform:uppercase;letter-spacing:0.5px;font-weight:600;">${title}</h4>
                </div>
                ${extraHeader}
            </div>
            <div class="panel-body" style="flex:1;padding:8px;overflow:hidden;min-height:0;"></div>
        `;
        return panel;
    }

    createTsnePanel() {
        const panel = this.createPanel('t-SNE 聚类视图 (2D)', `<span style="font-size:10px;color:#555;">样本: ${this.tsneSamples.length}</span>`);
        const body = panel.querySelector('.panel-body');
        body.style.cssText = 'flex:1;padding:6px;position:relative;min-height:0;';
        body.innerHTML = `
            <canvas id="tsneCanvas" style="width:100%;height:100%;display:block;border-radius:4px;background:#0a0f1a;"></canvas>
            <div id="tsneTooltip" style="display:none;position:absolute;background:rgba(10,22,40,0.95);border:1px solid #00d4ff;border-radius:6px;padding:8px;font-size:11px;z-index:100;pointer-events:none;min-width:160px;box-shadow:0 4px 16px rgba(0,212,255,0.2);"></div>
            <div style="position:absolute;bottom:8px;left:8px;display:flex;flex-wrap:wrap;gap:6px;max-width:60%;background:rgba(10,22,40,0.7);padding:6px 8px;border-radius:4px;border:1px solid #1e3a5f;">
                ${Object.entries(this.categoryConfig).map(([k, v]) => `
                    <div style="display:flex;align-items:center;gap:3px;font-size:9px;">
                        <span style="width:8px;height:8px;border-radius:50%;background:${v.color};"></span>
                        <span style="color:#888;">${v.name}</span>
                    </div>
                `).join('')}
            </div>
        `;

        setTimeout(() => {
            const canvas = body.querySelector('#tsneCanvas');
            canvas.addEventListener('mousemove', (e) => this.handleTsneHover(e));
            canvas.addEventListener('mouseleave', () => {
                body.querySelector('#tsneTooltip').style.display = 'none';
            });
        }, 50);

        return panel;
    }

    renderTsnePlot() {
        const canvas = document.getElementById('tsneCanvas');
        if (!canvas) return;
        const dpr = window.devicePixelRatio;
        const rect = canvas.getBoundingClientRect();
        canvas.width = rect.width * dpr;
        canvas.height = rect.height * dpr;
        const ctx = canvas.getContext('2d');
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        const w = rect.width, h = rect.height;

        const bg = ctx.createRadialGradient(w/2, h/2, 0, w/2, h/2, w);
        bg.addColorStop(0, '#0d1a2e');
        bg.addColorStop(1, '#0a0f1a');
        ctx.fillStyle = bg;
        ctx.fillRect(0, 0, w, h);

        const pad = 30;
        const cw = w - pad * 2, ch = h - pad * 2;

        ctx.strokeStyle = '#1e3a5f55';
        ctx.lineWidth = 0.5;
        for (let i = 0; i <= 6; i++) {
            const x = pad + (i / 6) * cw;
            const y = pad + (i / 6) * ch;
            ctx.beginPath(); ctx.moveTo(x, pad); ctx.lineTo(x, h - pad); ctx.stroke();
            ctx.beginPath(); ctx.moveTo(pad, y); ctx.lineTo(w - pad, y); ctx.stroke();
        }

        const mapX = v => pad + (v + 1) / 2 * cw;
        const mapY = v => pad + (1 - (v + 1) / 2) * ch;

        this.tsneSamples.forEach(s => {
            const x = mapX(s.x), y = mapY(s.y);
            const isSel = this.selectedPattern && s.pattern_id === this.selectedPattern.id;
            ctx.fillStyle = this.categoryConfig[s.category].color + (isSel ? 'ff' : '88');
            ctx.beginPath();
            ctx.arc(x, y, isSel ? 3.5 : 2.2, 0, Math.PI * 2);
            ctx.fill();
            if (isSel) {
                ctx.strokeStyle = '#ffffff';
                ctx.lineWidth = 0.8;
                ctx.stroke();
            }
        });

        const categoryGroups = {};
        this.tsneSamples.forEach(s => {
            if (!categoryGroups[s.category]) categoryGroups[s.category] = [];
            categoryGroups[s.category].push(s);
        });

        Object.keys(categoryGroups).forEach(cat => {
            const samples = categoryGroups[cat];
            const cx = samples.reduce((s, v) => s + v.x, 0) / samples.length;
            const cy = samples.reduce((s, v) => s + v.y, 0) / samples.length;
            const px = mapX(cx), py = mapY(cy);
            const color = this.categoryConfig[cat].color;

            ctx.beginPath();
            ctx.arc(px, py, 10, 0, Math.PI * 2);
            ctx.strokeStyle = color + 'aa';
            ctx.lineWidth = 2;
            ctx.stroke();
            ctx.beginPath();
            ctx.arc(px, py, 10, 0, Math.PI * 2);
            ctx.fillStyle = color + '22';
            ctx.fill();

            ctx.beginPath();
            ctx.arc(px, py, 4, 0, Math.PI * 2);
            ctx.fillStyle = color;
            ctx.fill();
            ctx.strokeStyle = '#ffffff';
            ctx.lineWidth = 1;
            ctx.stroke();
        });

        ctx.fillStyle = '#444';
        ctx.font = '9px monospace';
        ctx.textAlign = 'center';
        ctx.fillText('t-SNE 1', w / 2, h - 8);
        ctx.save();
        ctx.translate(10, h / 2);
        ctx.rotate(-Math.PI / 2);
        ctx.fillText('t-SNE 2', 0, 0);
        ctx.restore();
    }

    handleTsneHover(e) {
        const canvas = document.getElementById('tsneCanvas');
        const tooltip = document.getElementById('tsneTooltip');
        if (!canvas || !tooltip) return;
        const rect = canvas.getBoundingClientRect();
        const mx = e.clientX - rect.left;
        const my = e.clientY - rect.top;
        const w = rect.width, h = rect.height;
        const pad = 30;
        const cw = w - pad * 2, ch = h - pad * 2;
        const invX = v => (v - pad) / cw * 2 - 1;
        const invY = v => 1 - (v - pad) / ch * 2 - 1;
        const mxData = invX(mx), myData = invY(h - my);

        let closest = null, minDist = 0.08;
        this.tsneSamples.forEach(s => {
            const d = Math.hypot(s.x - mxData, s.y - (-myData));
            if (d < minDist) { minDist = d; closest = s; }
        });

        if (closest) {
            tooltip.style.display = 'block';
            tooltip.style.left = (mx + 12) + 'px';
            tooltip.style.top = (my - 10) + 'px';
            const conf = 0.7 + Math.abs(closest.x + closest.y) * 0.2;
            tooltip.innerHTML = `
                <div style="font-weight:600;color:${this.categoryConfig[closest.category].color};margin-bottom:6px;border-bottom:1px solid #1e3a5f;padding-bottom:4px;">
                    ${closest.category_name}
                </div>
                <div style="display:flex;justify-content:space-between;margin:2px 0;"><span style="color:#555;">样本ID</span><span style="color:#e0e6ed;font-family:monospace;">#${String(closest.id).padStart(4,'0')}</span></div>
                <div style="display:flex;justify-content:space-between;margin:2px 0;"><span style="color:#555;">机组</span><span style="color:#00d4ff;">${closest.turbine_id + 1}#</span></div>
                <div style="display:flex;justify-content:space-between;margin:2px 0;"><span style="color:#555;">传感器</span><span style="color:#e0e6ed;font-family:monospace;">${closest.sensor_id}</span></div>
                <div style="display:flex;justify-content:space-between;margin:2px 0;"><span style="color:#555;">置信度</span><span style="color:#00ff88;font-family:monospace;">${(conf*100).toFixed(1)}%</span></div>
            `;
        } else {
            tooltip.style.display = 'none';
        }
    }

    createPatternsPanel() {
        const panel = this.createPanel('模式库', `<span style="font-size:10px;color:#555;">共 ${this.patterns.length} 类</span>`);
        const body = panel.querySelector('.panel-body');
        body.style.cssText = 'flex:1;padding:6px;overflow-y:auto;min-height:0;display:flex;flex-direction:column;gap:6px;';
        body.id = 'patternList';
        this.renderPatternList(body);
        return panel;
    }

    renderPatternList(container) {
        container.innerHTML = this.patterns.map(p => {
            const isSel = this.selectedPattern && this.selectedPattern.id === p.id;
            const catCfg = this.categoryConfig[p.type];
            const silColor = p.silhouette > 0.7 ? '#4caf50' : p.silhouette > 0.5 ? '#ffc107' : '#ff9800';
            return `
                <div class="pattern-card" data-id="${p.id}" style="background:rgba(10,22,40,${isSel ? '0.9' : '0.6'});border:1px solid ${isSel ? catCfg.color : '#1e3a5f'};border-radius:6px;padding:8px;cursor:pointer;transition:all 0.15s ease;">
                    <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:6px;">
                        <div style="display:flex;align-items:center;gap:6px;">
                            <span style="width:10px;height:10px;border-radius:50%;background:${catCfg.color};box-shadow:0 0 6px ${catCfg.color};"></span>
                            <span style="font-weight:600;color:${catCfg.color};font-size:12px;">${p.type_name}</span>
                        </div>
                        ${p.expert_validated
                            ? '<span style="font-size:9px;padding:1px 6px;border-radius:8px;background:rgba(0,255,136,0.15);color:#00ff88;border:1px solid #00ff8844;">✓ 专家已验证</span>'
                            : '<span style="font-size:9px;padding:1px 6px;border-radius:8px;background:rgba(255,193,7,0.15);color:#ffc107;border:1px solid #ffc10744;">待验证</span>'}
                    </div>
                    <div style="display:grid;grid-template-columns:repeat(3,1fr);gap:4px;font-size:10px;margin-bottom:6px;">
                        <div>
                            <div style="color:#555;">样本数</div>
                            <div style="color:#e0e6ed;font-family:monospace;font-weight:600;">${p.sample_count}</div>
                        </div>
                        <div>
                            <div style="color:#555;">轮廓系数</div>
                            <div style="color:${silColor};font-family:monospace;font-weight:600;">${p.silhouette.toFixed(3)}</div>
                        </div>
                        <div>
                            <div style="color:#555;">编号</div>
                            <div style="color:#7a8ca3;font-family:monospace;">${p.id.slice(-4)}</div>
                        </div>
                    </div>
                    <div style="display:flex;justify-content:flex-end;">
                        <button class="view-pattern-btn" data-id="${p.id}" style="padding:4px 10px;background:rgba(0,212,255,0.15);border:1px solid #00d4ff;border-radius:4px;color:#00d4ff;font-size:10px;cursor:pointer;">
                            查看详情 →
                        </button>
                    </div>
                </div>
            `;
        }).join('');

        container.querySelectorAll('.pattern-card').forEach(card => {
            card.addEventListener('click', (e) => {
                if (!e.target.classList.contains('view-pattern-btn')) {
                    this.selectPattern(card.dataset.id);
                }
            });
        });
        container.querySelectorAll('.view-pattern-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                e.stopPropagation();
                this.selectPattern(btn.dataset.id);
            });
        });
    }

    createLatestPanel() {
        const panel = this.createPanel('实时诊断流', `<span style="font-size:10px;color:#555;" id="latestCount">共 ${this.latestDiagnoses.length} 条</span>`);
        const body = panel.querySelector('.panel-body');
        body.style.cssText = 'flex:1;padding:4px 6px;overflow-y:auto;min-height:0;display:flex;flex-direction:column;gap:3px;';
        body.id = 'latestList';
        this.renderLatestList(body);
        return panel;
    }

    renderLatestList(container) {
        container.innerHTML = this.latestDiagnoses.slice(0, 50).map(d => {
            const catCfg = this.categoryConfig[d.type];
            const confColor = d.confidence > 0.9 ? '#00ff88' : d.confidence > 0.8 ? '#00d4ff' : '#ffc107';
            const time = new Date(d.timestamp);
            const timeStr = `${String(time.getHours()).padStart(2,'0')}:${String(time.getMinutes()).padStart(2,'0')}:${String(time.getSeconds()).padStart(2,'0')}`;
            return `
                <div class="diag-entry" style="background:rgba(10,22,40,0.5);border-left:3px solid ${catCfg.color};border-radius:4px;padding:5px 8px;display:grid;grid-template-columns:auto 1fr auto;gap:8px;align-items:center;font-size:11px;">
                    <div style="font-family:monospace;color:#555;font-size:9px;min-width:55px;">${timeStr}</div>
                    <div style="min-width:0;overflow:hidden;">
                        <div style="display:flex;align-items:center;gap:5px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;">
                            <span style="color:#7a8ca3;">${d.turbine_name}</span>
                            <span style="color:#444;">·</span>
                            <span style="font-family:monospace;color:#555;">${d.sensor_id}</span>
                            <span style="display:inline-block;padding:0 5px;border-radius:8px;background:${catCfg.color}22;color:${catCfg.color};font-size:9px;font-weight:600;">${d.type_name}</span>
                            ${!d.is_known ? '<span style="font-size:8px;padding:0 4px;border-radius:6px;background:rgba(156,39,176,0.2);color:#9c27b0;">新</span>' : ''}
                        </div>
                    </div>
                    <div style="font-family:monospace;color:${confColor};font-weight:600;min-width:40px;text-align:right;">${(d.confidence*100).toFixed(0)}%</div>
                </div>
            `;
        }).join('');
    }

    createAnalysisPanel() {
        const panel = this.createPanel('模式深度分析', `
            <div id="analTitle" style="font-size:10px;color:#555;display:flex;align-items:center;gap:6px;">
                <span style="width:8px;height:8px;border-radius:50%;background:#9c27b0;"></span>
                <span>未选中</span>
            </div>
        `);
        const body = panel.querySelector('.panel-body');
        body.style.cssText = 'flex:1;padding:6px;overflow-y:auto;min-height:0;display:flex;flex-direction:column;gap:8px;';
        body.id = 'analysisBody';
        body.innerHTML = `
            <div style="color:#555;font-size:11px;text-align:center;padding:30px 10px;">请从左侧选择一个空化模式</div>
        `;
        return panel;
    }

    selectPattern(patternId) {
        this.selectedPattern = this.patterns.find(p => p.id === patternId);
        if (!this.selectedPattern) return;

        const titleEl = document.getElementById('analTitle');
        if (titleEl) {
            const cc = this.categoryConfig[this.selectedPattern.type];
            titleEl.innerHTML = `
                <span style="width:8px;height:8px;border-radius:50%;background:${cc.color};"></span>
                <span style="color:${cc.color};">${cc.name} · ${this.selectedPattern.id.slice(-4)}</span>
            `;
        }

        const pl = document.getElementById('patternList');
        if (pl) this.renderPatternList(pl);
        this.renderTsnePlot();
        this.renderPatternAnalysis();
    }

    renderPatternAnalysis() {
        const body = document.getElementById('analysisBody');
        if (!body || !this.selectedPattern) return;
        const p = this.selectedPattern;
        const catCfg = this.categoryConfig[p.type];
        const simPatterns = this.patterns
            .filter(pp => pp.id !== p.id)
            .map(pp => ({
                ...pp,
                similarity: this.cosineSim(p.centroid, pp.centroid)
            }))
            .sort((a, b) => b.similarity - a.similarity)
            .slice(0, 4);

        body.innerHTML = `
            <div>
                <div style="font-size:10px;color:#555;margin-bottom:4px;">32维 Embedding 平行坐标</div>
                <canvas id="parallelCanvas" style="width:100%;height:90px;display:block;border-radius:4px;background:#0a0f1a;"></canvas>
            </div>
            <div>
                <div style="font-size:10px;color:#555;margin-bottom:4px;">Top4 相似度匹配</div>
                <canvas id="simBarCanvas" style="width:100%;height:70px;display:block;border-radius:4px;background:#0a0f1a;"></canvas>
            </div>
            <div>
                <div style="font-size:10px;color:#555;margin-bottom:4px;">典型声纹波形 (正弦叠加模拟)</div>
                <canvas id="waveCanvas" style="width:100%;height:70px;display:block;border-radius:4px;background:#0a0f1a;"></canvas>
            </div>
            <div>
                <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:4px;">
                    <span style="font-size:10px;color:#555;">专家标注</span>
                    <button id="saveLabelBtn" style="padding:3px 10px;background:rgba(0,255,136,0.15);border:1px solid #00ff88;border-radius:3px;color:#00ff88;font-size:10px;cursor:pointer;">保存标注</button>
                </div>
                <textarea id="labelTextarea" rows="3" placeholder="输入专家标注内容..." style="width:100%;padding:6px;background:rgba(10,22,40,0.8);border:1px solid #1e3a5f;border-radius:4px;color:#e0e6ed;font-size:11px;resize:vertical;font-family:inherit;">${p.expert_validated ? '【已验证】该模式为典型' + catCfg.name + '特征，中心频段3-8kHz，伴随周期性冲击结构。建议监测同类型机组对应测点。' : ''}</textarea>
            </div>
        `;

        setTimeout(() => {
            this.renderParallelCoords(p);
            this.renderSimilarityBars(simPatterns);
            this.renderWaveform(p);

            const saveBtn = body.querySelector('#saveLabelBtn');
            if (saveBtn) {
                saveBtn.addEventListener('click', () => this.saveExpertLabel());
            }
        }, 50);
    }

    cosineSim(a, b) {
        let dot = 0, na = 0, nb = 0;
        for (let i = 0; i < a.length; i++) {
            dot += a[i] * b[i]; na += a[i] * a[i]; nb += b[i] * b[i];
        }
        return dot / (Math.sqrt(na) * Math.sqrt(nb) + 1e-10);
    }

    renderParallelCoords(pattern) {
        const canvas = document.getElementById('parallelCanvas');
        if (!canvas) return;
        const dpr = window.devicePixelRatio;
        const rect = canvas.getBoundingClientRect();
        canvas.width = rect.width * dpr;
        canvas.height = rect.height * dpr;
        const ctx = canvas.getContext('2d');
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        const w = rect.width, h = rect.height;
        ctx.fillStyle = '#0a0f1a'; ctx.fillRect(0, 0, w, h);

        const pad = { l: 20, r: 10, t: 12, b: 14 };
        const cw = w - pad.l - pad.r, ch = h - pad.t - pad.b;
        const N = 32;

        for (let i = 0; i < N; i++) {
            const x = pad.l + (i / (N - 1)) * cw;
            ctx.strokeStyle = '#1e3a5f'; ctx.lineWidth = 0.5;
            ctx.beginPath(); ctx.moveTo(x, pad.t); ctx.lineTo(x, h - pad.b); ctx.stroke();
        }

        const samples = this.tsneSamples.filter(s => s.pattern_id === pattern.id).slice(0, 15);
        samples.forEach((s, idx) => {
            ctx.strokeStyle = this.categoryConfig[s.category].color + (0.2 + idx * 0.02);
            ctx.lineWidth = 0.8;
            ctx.beginPath();
            for (let i = 0; i < N; i++) {
                const v = s.embedding[i];
                const x = pad.l + (i / (N - 1)) * cw;
                const y = pad.t + ch / 2 - v * ch * 0.45;
                if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
            }
            ctx.stroke();
        });

        const cc = this.categoryConfig[pattern.type].color;
        ctx.strokeStyle = cc; ctx.lineWidth = 2;
        ctx.beginPath();
        for (let i = 0; i < N; i++) {
            const v = pattern.centroid[i];
            const x = pad.l + (i / (N - 1)) * cw;
            const y = pad.t + ch / 2 - v * ch * 0.45;
            if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        }
        ctx.stroke();

        for (let i = 0; i < N; i++) {
            const v = pattern.centroid[i];
            const x = pad.l + (i / (N - 1)) * cw;
            const y = pad.t + ch / 2 - v * ch * 0.45;
            ctx.fillStyle = cc;
            ctx.beginPath();
            ctx.arc(x, y, 1.8, 0, Math.PI * 2);
            ctx.fill();
        }

        for (let i = 0; i < N; i += 4) {
            const x = pad.l + (i / (N - 1)) * cw;
            ctx.fillStyle = '#444'; ctx.font = '7px monospace'; ctx.textAlign = 'center';
            ctx.fillText('d' + (i + 1), x, h - 3);
        }
    }

    renderSimilarityBars(items) {
        const canvas = document.getElementById('simBarCanvas');
        if (!canvas) return;
        const dpr = window.devicePixelRatio;
        const rect = canvas.getBoundingClientRect();
        canvas.width = rect.width * dpr;
        canvas.height = rect.height * dpr;
        const ctx = canvas.getContext('2d');
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        const w = rect.width, h = rect.height;
        ctx.fillStyle = '#0a0f1a'; ctx.fillRect(0, 0, w, h);

        const pad = { l: 75, r: 30, t: 4, b: 4 };
        const cw = w - pad.l - pad.r, ch = h - pad.t - pad.b;
        const rowH = ch / items.length;

        items.forEach((item, i) => {
            const y = pad.t + i * rowH + 3;
            const bh = rowH - 8;
            const bw = cw * item.similarity;
            const cc = this.categoryConfig[item.type].color;

            ctx.fillStyle = '#555'; ctx.font = '9px sans-serif'; ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
            ctx.fillText(item.type_name, pad.l - 6, y + bh / 2);

            ctx.fillStyle = '#1e3a5f';
            ctx.fillRect(pad.l, y, cw, bh);
            const grad = ctx.createLinearGradient(pad.l, 0, pad.l + bw, 0);
            grad.addColorStop(0, cc + '88');
            grad.addColorStop(1, cc);
            ctx.fillStyle = grad;
            ctx.fillRect(pad.l, y, bw, bh);

            ctx.fillStyle = '#fff'; ctx.font = '9px monospace'; ctx.textAlign = 'left';
            ctx.fillText((item.similarity * 100).toFixed(1) + '%', pad.l + bw + 4, y + bh / 2);
        });
    }

    renderWaveform(pattern) {
        const canvas = document.getElementById('waveCanvas');
        if (!canvas) return;
        const dpr = window.devicePixelRatio;
        const rect = canvas.getBoundingClientRect();
        canvas.width = rect.width * dpr;
        canvas.height = rect.height * dpr;
        const ctx = canvas.getContext('2d');
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        const w = rect.width, h = rect.height;
        ctx.fillStyle = '#0a0f1a'; ctx.fillRect(0, 0, w, h);

        const pad = { l: 20, r: 10, t: 8, b: 8 };
        const cw = w - pad.l - pad.r, ch = h - pad.t - pad.b;

        ctx.strokeStyle = '#1e3a5f'; ctx.lineWidth = 0.5;
        ctx.beginPath();
        ctx.moveTo(pad.l, pad.t + ch / 2); ctx.lineTo(w - pad.r, pad.t + ch / 2);
        ctx.stroke();

        const catIdx = Object.keys(this.categoryConfig).indexOf(pattern.type);
        const baseFreq = 80 + catIdx * 35;
        const cc = this.categoryConfig[pattern.type].color;

        ctx.strokeStyle = cc + '44'; ctx.lineWidth = 1;
        const t = Date.now() / 1000;
        const samples = 800;
        for (let pass = 0; pass < 3; pass++) {
            ctx.beginPath();
            for (let i = 0; i < samples; i++) {
                const tt = i / samples * 0.05 + t * 0.001 + pass * 0.002;
                let v = 0;
                v += Math.sin(2 * Math.PI * baseFreq * tt) * 0.4;
                v += Math.sin(2 * Math.PI * baseFreq * 2.3 * tt + catIdx) * 0.25;
                v += Math.sin(2 * Math.PI * baseFreq * 0.5 * tt) * 0.2;
                v += (Math.random() - 0.5) * (0.1 + pattern.silhouette * 0.1);
                const x = pad.l + (i / (samples - 1)) * cw;
                const y = pad.t + ch / 2 - v * ch * 0.4 * (1 - pass * 0.2);
                if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
            }
            ctx.stroke();
        }

        ctx.strokeStyle = cc; ctx.lineWidth = 1.2;
        ctx.beginPath();
        for (let i = 0; i < samples; i++) {
            const tt = i / samples * 0.05;
            let v = 0;
            v += Math.sin(2 * Math.PI * baseFreq * tt) * 0.4;
            v += Math.sin(2 * Math.PI * baseFreq * 2.3 * tt + catIdx) * 0.25;
            v += Math.sin(2 * Math.PI * baseFreq * 0.5 * tt) * 0.2;
            const x = pad.l + (i / (samples - 1)) * cw;
            const y = pad.t + ch / 2 - v * ch * 0.4;
            if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        }
        ctx.stroke();

        ctx.fillStyle = '#444'; ctx.font = '8px monospace';
        ctx.textAlign = 'left'; ctx.textBaseline = 'top';
        ctx.fillText(`${baseFreq}Hz~`, pad.l + 2, pad.t + 2);
    }

    async saveExpertLabel() {
        const textarea = document.getElementById('labelTextarea');
        if (!textarea || !this.selectedPattern) return;
        const label = textarea.value.trim();
        if (!label) { alert('请输入标注内容'); return; }

        try {
            await fetch(`${this.app.apiBase}/diagnosis/label`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ pattern_id: this.selectedPattern.id, label, user: 'web-expert' })
            });
        } catch (e) {}

        this.selectedPattern.expert_validated = true;
        const pl = document.getElementById('patternList');
        if (pl) this.renderPatternList(pl);
        alert('专家标注已保存');
    }

    startLiveUpdates() {
        if (this.updateInterval) clearInterval(this.updateInterval);
        this.updateInterval = setInterval(() => {
            const categories = Object.keys(this.categoryConfig);
            const cat = categories[Math.floor(Math.random() * categories.length)];
            const known = Math.random() > 0.15;
            this.latestDiagnoses.unshift({
                id: 'diag-' + String(Date.now()).slice(-8),
                turbine_id: Math.floor(Math.random() * 6),
                turbine_name: `${Math.floor(Math.random() * 6) + 1}# 水轮机`,
                sensor_id: `H${String(Math.floor(Math.random() * 12) + 1).padStart(2, '0')}`,
                type: cat,
                type_name: this.categoryConfig[cat].name,
                confidence: 0.7 + Math.random() * 0.29,
                timestamp: Date.now(),
                is_known: known,
                pattern_id: known ? this.patterns[Math.floor(Math.random() * this.patterns.length)].id : null,
                embedding: this.generateEmbedding(cat)
            });
            if (this.latestDiagnoses.length > 80) this.latestDiagnoses.pop();

            const count = document.getElementById('latestCount');
            if (count) count.textContent = `共 ${this.latestDiagnoses.length} 条`;
            const list = document.getElementById('latestList');
            if (list) this.renderLatestList(list);

            if (Math.random() < 0.3) {
                this.renderWaveform(this.selectedPattern || this.patterns[0]);
            }
        }, 2000);
    }

    resize() {
        if (!this.container) return;
        setTimeout(() => {
            this.renderTsnePlot();
            if (this.selectedPattern) {
                this.renderParallelCoords(this.selectedPattern);
                this.renderPatternAnalysis();
            }
        }, 50);
    }
}
