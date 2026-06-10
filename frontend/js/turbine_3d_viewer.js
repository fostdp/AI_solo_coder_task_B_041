class Turbine3DViewer {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        this.gl = this.canvas.getContext('webgl', { antialias: true, preserveDrawingBuffer: true });
        if (!this.gl) {
            this.gl = this.canvas.getContext('experimental-webgl');
        }

        this.turbineId = 0;
        this.bladeCount = 15;
        this.bladeData = [];
        this.cavitationData = {};
        this.hoveredBlade = -1;
        this.selectedBlade = -1;
        this.rotation = 0;
        this.zoom = 1.0;
        this.offsetX = 0;
        this.offsetY = 0;
        this.isDragging = false;
        this.lastMouseX = 0;
        this.lastMouseY = 0;
        this.clickableBlades = [];
        this.onBladeHover = null;
        this.onBladeSelect = null;

        this.programs = {};
        this.buffers = {};

        this.stageColorMap = [
            { r: 0.1, g: 0.8, b: 0.3, name: 'normal', label: '正常' },
            { r: 1.0, g: 0.8, b: 0.0, name: 'incipient', label: '初生' },
            { r: 1.0, g: 0.4, b: 0.0, name: 'critical', label: '临界' },
            { r: 1.0, g: 0.1, b: 0.2, name: 'developed', label: '发展' }
        ];

        this.init();
    }

    init() {
        if (!this.gl) {
            console.error('WebGL not supported');
            return;
        }
        this.resize();
        window.addEventListener('resize', () => this.resize());
        this.initShaders();
        this.initGeometry();
        this.initEventListeners();

        for (let i = 0; i < this.bladeCount; i++) {
            this.cavitationData[i] = {
                intensity: Math.random() * 0.15,
                stage: 0,
                damage: Math.random() * 0.01,
                stress: Math.random() * 20 + 10
            };
        }
        this.animate();
    }

    resize() {
        const rect = this.canvas.parentElement.getBoundingClientRect();
        this.canvas.width = rect.width * window.devicePixelRatio;
        this.canvas.height = rect.height * window.devicePixelRatio;
        this.canvas.style.width = rect.width + 'px';
        this.canvas.style.height = rect.height + 'px';
        if (this.gl) {
            this.gl.viewport(0, 0, this.canvas.width, this.canvas.height);
        }
    }

    initShaders() {
        const gl = this.gl;
        const vsBasic = `
            attribute vec2 a_position;
            uniform mat3 u_matrix;
            void main() {
                vec3 pos = u_matrix * vec3(a_position, 1.0);
                gl_Position = vec4(pos.xy, 0.0, 1.0);
            }
        `;
        const fsBasic = `
            precision mediump float;
            uniform vec4 u_color;
            void main() {
                gl_FragColor = u_color;
            }
        `;
        this.programs.basic = this.createProgram(vsBasic, fsBasic);

        const vsBlade = `
            attribute vec2 a_position;
            attribute vec3 a_color;
            attribute float a_intensity;
            uniform mat3 u_matrix;
            varying vec3 v_color;
            varying float v_intensity;
            void main() {
                vec3 pos = u_matrix * vec3(a_position, 1.0);
                gl_Position = vec4(pos.xy, 0.0, 1.0);
                v_color = a_color;
                v_intensity = a_intensity;
            }
        `;
        const fsBlade = `
            precision mediump float;
            varying vec3 v_color;
            varying float v_intensity;
            void main() {
                vec3 color = v_color;
                float glow = v_intensity * 0.5;
                gl_FragColor = vec4(color + glow * vec3(0.3, 0.0, 0.1), 0.9);
            }
        `;
        this.programs.blade = this.createProgram(vsBlade, fsBlade);
    }

    createProgram(vs, fs) {
        const gl = this.gl;
        const program = gl.createProgram();
        gl.attachShader(program, this.createShader(gl.VERTEX_SHADER, vs));
        gl.attachShader(program, this.createShader(gl.FRAGMENT_SHADER, fs));
        gl.linkProgram(program);
        if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
            console.error('Link error:', gl.getProgramInfoLog(program));
            return null;
        }
        return program;
    }

    createShader(type, source) {
        const gl = this.gl;
        const shader = gl.createShader(type);
        gl.shaderSource(shader, source);
        gl.compileShader(shader);
        if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
            console.error('Shader error:', gl.getShaderInfoLog(shader));
            gl.deleteShader(shader);
            return null;
        }
        return shader;
    }

    initGeometry() {
        this.buffers.spiralCase = this.createSpiralCase();
        this.buffers.guideVanes = this.createGuideVanes();
        this.buffers.runnerHub = this.createRunnerHub();
        this.buffers.draftTube = this.createDraftTube();
        this.buffers.blades = this.createBlades();
    }

    createSpiralCase() {
        const gl = this.gl;
        const positions = [];
        const seg = 64;
        const inner = 0.7, outer = 0.95;
        for (let i = 0; i <= seg; i++) {
            const a = (i / seg) * Math.PI * 2 - Math.PI / 2;
            const t = i / seg;
            const r = inner + (outer - inner) * (0.5 + 0.5 * Math.sin(t * Math.PI * 3));
            positions.push(Math.cos(a) * r, Math.sin(a) * r * 0.8);
        }
        for (let i = seg; i >= 0; i--) {
            const a = (i / seg) * Math.PI * 2 - Math.PI / 2;
            const t = i / seg;
            const r = inner - 0.08 + (outer - inner) * 0.3 * (0.5 + 0.5 * Math.sin(t * Math.PI * 3));
            positions.push(Math.cos(a) * r, Math.sin(a) * r * 0.8);
        }
        const buf = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, buf);
        gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(positions), gl.STATIC_DRAW);
        return { buffer: buf, count: positions.length / 2 };
    }

    createGuideVanes() {
        const gl = this.gl;
        const positions = [];
        const n = 24;
        const inner = 0.48, outer = 0.62;
        for (let i = 0; i < n; i++) {
            const a = (i / n) * Math.PI * 2;
            const na = ((i + 0.7) / n) * Math.PI * 2;
            const x1 = Math.cos(a) * inner, y1 = Math.sin(a) * inner;
            const x2 = Math.cos(na) * outer, y2 = Math.sin(na) * outer;
            const tx = Math.cos(a + Math.PI / 2) * 0.015, ty = Math.sin(a + Math.PI / 2) * 0.015;
            positions.push(x1 + tx, y1 + ty, x2 + tx, y2 + ty, x2 - tx, y2 - ty, x1 - tx, y1 - ty, x1 + tx, y1 + ty);
        }
        const buf = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, buf);
        gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(positions), gl.STATIC_DRAW);
        return { buffer: buf, count: positions.length / 2 };
    }

    createRunnerHub() {
        const gl = this.gl;
        const positions = [];
        const seg = 32;
        for (let i = 0; i <= seg; i++) {
            const a = (i / seg) * Math.PI * 2;
            positions.push(Math.cos(a) * 0.12, Math.sin(a) * 0.12);
        }
        for (let i = seg; i >= 0; i--) {
            const a = (i / seg) * Math.PI * 2;
            positions.push(Math.cos(a) * 0.08, Math.sin(a) * 0.08);
        }
        const buf = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, buf);
        gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(positions), gl.STATIC_DRAW);
        return { buffer: buf, count: positions.length / 2 };
    }

    createDraftTube() {
        const gl = this.gl;
        const positions = [-0.15, 0, -0.4, -0.8, 0.4, -0.8, 0.15, 0];
        const buf = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, buf);
        gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(positions), gl.STATIC_DRAW);
        return { buffer: buf, count: positions.length / 2 };
    }

    createBlades() {
        const gl = this.gl;
        const meshes = [];
        const inner = 0.12, outer = 0.46;
        const rSeg = 8, aSeg = 6;
        const vertsRow = aSeg + 1;

        for (let bid = 0; bid < this.bladeCount; bid++) {
            const baseAngle = (bid / this.bladeCount) * Math.PI * 2;
            const gridPoints = [];
            const positions = [];
            const colors = [];
            const intensities = [];

            for (let i = 0; i <= rSeg; i++) {
                for (let j = 0; j <= aSeg; j++) {
                    const r = inner + (outer - inner) * (i / rSeg);
                    const twist = (i / rSeg) * 0.5;
                    const off = (j / aSeg - 0.5) * 0.25 * (1 - i / rSeg * 0.5);
                    const a = baseAngle + twist + off;
                    const x = Math.cos(a) * r, y = Math.sin(a) * r;
                    gridPoints.push({ x, y, u: i / rSeg, v: j / aSeg });
                    positions.push(x, y);
                    colors.push(0.2, 0.5, 0.8);
                    intensities.push(0.0);
                }
            }

            const indices = [];
            for (let i = 0; i < rSeg; i++) {
                for (let j = 0; j < aSeg; j++) {
                    const idx = i * vertsRow + j;
                    indices.push(idx, idx + 1, idx + vertsRow, idx + 1, idx + vertsRow + 1, idx + vertsRow);
                }
            }

            const iP = [], iC = [], iI = [];
            for (const ix of indices) {
                iP.push(positions[ix * 2], positions[ix * 2 + 1]);
                iC.push(colors[ix * 3], colors[ix * 3 + 1], colors[ix * 3 + 2]);
                iI.push(intensities[ix]);
            }

            const pb = gl.createBuffer();
            gl.bindBuffer(gl.ARRAY_BUFFER, pb);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(iP), gl.DYNAMIC_DRAW);
            const cb = gl.createBuffer();
            gl.bindBuffer(gl.ARRAY_BUFFER, cb);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(iC), gl.DYNAMIC_DRAW);
            const ib = gl.createBuffer();
            gl.bindBuffer(gl.ARRAY_BUFFER, ib);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(iI), gl.DYNAMIC_DRAW);

            const centerP = { x: 0, y: 0 };
            for (let k = 0; k < iP.length; k += 2) {
                centerP.x += iP[k]; centerP.y += iP[k + 1];
            }
            centerP.x /= (iP.length / 2); centerP.y /= (iP.length / 2);

            meshes.push({
                id: bid, baseAngle,
                positionBuffer: pb, colorBuffer: cb, intensityBuffer: ib,
                count: indices.length, centerPoints: gridPoints,
                bounds: this.computeBounds(iP), indexedPositions: iP,
                vertsPerRow: vertsRow, radialSegments: rSeg, angularSegments: aSeg
            });

            this.clickableBlades.push({
                bladeId: bid, baseAngle, center: centerP, bounds: this.computeBounds(iP)
            });
        }
        return meshes;
    }

    computeBounds(p) {
        let mnx = Infinity, mny = Infinity, mxx = -Infinity, mxy = -Infinity;
        for (let i = 0; i < p.length; i += 2) {
            mnx = Math.min(mnx, p[i]); mny = Math.min(mny, p[i + 1]);
            mxx = Math.max(mxx, p[i]); mxy = Math.max(mxy, p[i + 1]);
        }
        return { minX: mnx, minY: mny, maxX: mxx, maxY: mxy };
    }

    initEventListeners() {
        this.canvas.addEventListener('mousedown', e => {
            this.isDragging = true;
            this.lastMouseX = e.clientX; this.lastMouseY = e.clientY;
        });
        this.canvas.addEventListener('mousemove', e => this.onMouseMove(e));
        this.canvas.addEventListener('mouseup', () => this.isDragging = false);
        this.canvas.addEventListener('mouseleave', () => {
            this.isDragging = false; this.hoveredBlade = -1;
            if (this.onBladeHover) this.onBladeHover(-1, null);
        });
        this.canvas.addEventListener('click', e => this.onClick(e));
        this.canvas.addEventListener('wheel', e => {
            e.preventDefault();
            this.zoom = Math.max(0.5, Math.min(3.0, this.zoom * (e.deltaY > 0 ? 0.9 : 1.1)));
        });
    }

    getMousePos(e) {
        const rect = this.canvas.getBoundingClientRect();
        const x = (e.clientX - rect.left) / rect.width * 2 - 1;
        const y = -(e.clientY - rect.top) / rect.height * 2 + 1;
        const inv = mat3.create();
        mat3.invert(inv, this.getMatrix());
        const wp = vec3.fromValues(x, y, 1);
        vec3.transformMat3(wp, wp, inv);
        return { x: wp[0], y: wp[1] };
    }

    onMouseMove(e) {
        const pos = this.getMousePos(e);
        let hovered = -1;
        for (const blade of this.clickableBlades) {
            if (this.isPointInBlade(pos, blade)) { hovered = blade.bladeId; break; }
        }
        if (hovered !== this.hoveredBlade) {
            this.hoveredBlade = hovered;
            if (this.onBladeHover) {
                const data = hovered >= 0 ? this.cavitationData[hovered] : null;
                this.onBladeHover(hovered, data);
            }
        }
        if (this.isDragging) {
            const dx = (e.clientX - this.lastMouseX) / this.canvas.width * 2;
            const dy = (e.clientY - this.lastMouseY) / this.canvas.height * 2;
            this.offsetX += dx / this.zoom; this.offsetY -= dy / this.zoom;
            this.lastMouseX = e.clientX; this.lastMouseY = e.clientY;
        }
    }

    onClick(e) {
        const pos = this.getMousePos(e);
        for (const blade of this.clickableBlades) {
            if (this.isPointInBlade(pos, blade)) {
                this.selectedBlade = blade.bladeId;
                if (this.onBladeSelect) {
                    this.onBladeSelect(blade.bladeId, this.turbineId, this.cavitationData[blade.bladeId]);
                }
                break;
            }
        }
    }

    isPointInBlade(pt, blade) {
        const { bounds } = blade;
        if (pt.x < bounds.minX || pt.x > bounds.maxX || pt.y < bounds.minY || pt.y > bounds.maxY) return false;
        const mesh = this.buffers.blades[blade.bladeId];
        const p = mesh.indexedPositions;
        for (let i = 0; i < p.length; i += 6) {
            if (this.inTri(pt, {x: p[i], y: p[i + 1]}, {x: p[i + 2], y: p[i + 3]}, {x: p[i + 4], y: p[i + 5]})) return true;
        }
        return false;
    }

    inTri(pt, v1, v2, v3) {
        const d1 = this.sign(pt, v1, v2), d2 = this.sign(pt, v2, v3), d3 = this.sign(pt, v3, v1);
        return !((d1 < 0 || d2 < 0 || d3 < 0) && (d1 > 0 || d2 > 0 || d3 > 0));
    }

    sign(p1, p2, p3) {
        return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
    }

    getMatrix() {
        const m = mat3.create();
        mat3.translate(m, m, [this.offsetX, this.offsetY]);
        mat3.scale(m, m, [this.zoom, this.zoom]);
        mat3.rotate(m, m, this.rotation);
        return m;
    }

    getColor(intensity, stage, hovered, selected) {
        const base = this.stageColorMap[Math.min(stage, 3)];
        const t = Math.min(intensity * 2, 1);
        let r = 0.2 + (base.r - 0.2) * t;
        let g = 0.5 + (base.g - 0.5) * t;
        let b = 0.8 + (base.b - 0.8) * t;
        if (hovered) { r = Math.min(1, r + 0.2); g = Math.min(1, g + 0.2); b = Math.min(1, b + 0.2); }
        if (selected) { r = Math.min(1, r + 0.3); g = Math.min(1, g + 0.1); b = Math.min(1, b + 0.4); }
        return { r, g, b };
    }

    updateBladeColors() {
        const gl = this.gl;
        for (const mesh of this.buffers.blades) {
            const bid = mesh.id;
            const data = this.cavitationData[bid];
            const isH = bid === this.hoveredBlade;
            const isS = bid === this.selectedBlade;
            const colors = [];
            const intensities = [];
            for (let i = 0; i < mesh.count; i++) {
                const cp = mesh.centerPoints[i % mesh.centerPoints.length];
                const dfc = Math.sqrt(Math.pow(cp.u - 0.5, 2) + Math.pow(cp.v - 0.5, 2));
                const iv = Math.max(0, Math.min(1, data.intensity * (1 - dfc * 0.5) + (Math.random() - 0.5) * 0.1));
                const c = this.getColor(iv, data.stage, isH, isS);
                colors.push(c.r, c.g, c.b);
                intensities.push(iv);
            }
            gl.bindBuffer(gl.ARRAY_BUFFER, mesh.colorBuffer);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(colors), gl.DYNAMIC_DRAW);
            gl.bindBuffer(gl.ARRAY_BUFFER, mesh.intensityBuffer);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(intensities), gl.DYNAMIC_DRAW);
        }
    }

    drawPrimitive(buf, progName, matrix, color, mode) {
        const gl = this.gl;
        const p = this.programs[progName];
        gl.useProgram(p);
        gl.uniformMatrix3fv(gl.getUniformLocation(p, 'u_matrix'), false, matrix);
        if (color) gl.uniform4f(gl.getUniformLocation(p, 'u_color'), color[0], color[1], color[2], color[3]);
        const pl = gl.getAttribLocation(p, 'a_position');
        gl.bindBuffer(gl.ARRAY_BUFFER, buf.buffer);
        gl.enableVertexAttribArray(pl);
        gl.vertexAttribPointer(pl, 2, gl.FLOAT, false, 0, 0);
        gl.drawArrays(mode, 0, buf.count);
    }

    drawBlades(matrix) {
        const gl = this.gl;
        const p = this.programs.blade;
        gl.useProgram(p);
        gl.uniformMatrix3fv(gl.getUniformLocation(p, 'u_matrix'), false, matrix);
        const pl = gl.getAttribLocation(p, 'a_position');
        const cl = gl.getAttribLocation(p, 'a_color');
        const il = gl.getAttribLocation(p, 'a_intensity');
        for (const m of this.buffers.blades) {
            gl.bindBuffer(gl.ARRAY_BUFFER, m.positionBuffer);
            gl.enableVertexAttribArray(pl); gl.vertexAttribPointer(pl, 2, gl.FLOAT, false, 0, 0);
            gl.bindBuffer(gl.ARRAY_BUFFER, m.colorBuffer);
            gl.enableVertexAttribArray(cl); gl.vertexAttribPointer(cl, 3, gl.FLOAT, false, 0, 0);
            gl.bindBuffer(gl.ARRAY_BUFFER, m.intensityBuffer);
            gl.enableVertexAttribArray(il); gl.vertexAttribPointer(il, 1, gl.FLOAT, false, 0, 0);
            gl.drawArrays(gl.TRIANGLES, 0, m.count);
        }
    }

    render() {
        const gl = this.gl;
        if (!gl) return;
        gl.clearColor(0.05, 0.08, 0.12, 1.0);
        gl.clear(gl.COLOR_BUFFER_BIT);
        gl.enable(gl.BLEND);
        gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
        const m = this.getMatrix();
        this.updateBladeColors();
        this.drawPrimitive(this.buffers.draftTube, 'basic', m, [0.15, 0.25, 0.35, 0.8], gl.TRIANGLE_FAN);
        this.drawPrimitive(this.buffers.spiralCase, 'basic', m, [0.2, 0.3, 0.4, 0.9], gl.TRIANGLE_FAN);
        this.drawPrimitive(this.guideVanes, 'basic', m, [0.25, 0.35, 0.45, 0.9], gl.TRIANGLE_STRIP);
        this.drawBlades(m);
        this.drawPrimitive(this.buffers.runnerHub, 'basic', m, [0.15, 0.2, 0.3, 1.0], gl.TRIANGLE_FAN);
    }

    setCavitationData(data) {
        for (let i = 0; i < this.bladeCount; i++) {
            if (data[i]) this.cavitationData[i] = { ...this.cavitationData[i], ...data[i] };
        }
    }

    setTurbine(id) {
        this.turbineId = id;
        this.selectedBlade = -1;
    }

    animate() {
        this.rotation += 0.002;
        this.render();
        requestAnimationFrame(() => this.animate());
    }
}
