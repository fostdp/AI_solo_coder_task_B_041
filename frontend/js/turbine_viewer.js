class TurbineViewer {
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
        
        this.programs = {};
        this.buffers = {};
        
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

        const vsSource = `
            attribute vec2 a_position;
            uniform mat3 u_matrix;
            void main() {
                vec3 pos = u_matrix * vec3(a_position, 1.0);
                gl_Position = vec4(pos.xy, 0.0, 1.0);
            }
        `;

        const fsSource = `
            precision mediump float;
            uniform vec4 u_color;
            void main() {
                gl_FragColor = u_color;
            }
        `;

        this.programs.basic = this.createProgram(vsSource, fsSource);

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
                gl_FragColor = vec4(color + glow * vec3(0.3, 0, 0.1), 0.9);
            }
        `;

        this.programs.blade = this.createProgram(vsBlade, fsBlade);
    }

    createProgram(vsSource, fsSource) {
        const gl = this.gl;
        const vs = this.createShader(gl.VERTEX_SHADER, vsSource);
        const fs = this.createShader(gl.FRAGMENT_SHADER, fsSource);

        const program = gl.createProgram();
        gl.attachShader(program, vs);
        gl.attachShader(program, fs);
        gl.linkProgram(program);

        if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
            console.error('Program link error:', gl.getProgramInfoLog(program));
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
            console.error('Shader compile error:', gl.getShaderInfoLog(shader));
            gl.deleteShader(shader);
            return null;
        }

        return shader;
    }

    initGeometry() {
        const gl = this.gl;

        this.buffers.spiralCase = this.createSpiralCase();
        this.buffers.guideVanes = this.createGuideVanes();
        this.buffers.runnerHub = this.createRunnerHub();
        this.buffers.draftTube = this.createDraftTube();
        this.buffers.blades = this.createBlades();
    }

    createSpiralCase() {
        const gl = this.gl;
        const positions = [];
        const segments = 64;
        const innerR = 0.7;
        const outerR = 0.95;

        for (let i = 0; i <= segments; i++) {
            const angle = (i / segments) * Math.PI * 2 - Math.PI / 2;
            const t = i / segments;
            const r = innerR + (outerR - innerR) * (0.5 + 0.5 * Math.sin(t * Math.PI * 3));
            
            const x = Math.cos(angle) * r;
            const y = Math.sin(angle) * r * 0.8;
            positions.push(x, y);
        }

        for (let i = segments; i >= 0; i--) {
            const angle = (i / segments) * Math.PI * 2 - Math.PI / 2;
            const t = i / segments;
            const r = innerR - 0.08 + (outerR - innerR) * 0.3 * (0.5 + 0.5 * Math.sin(t * Math.PI * 3));
            
            const x = Math.cos(angle) * r;
            const y = Math.sin(angle) * r * 0.8;
            positions.push(x, y);
        }

        const buffer = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
        gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(positions), gl.STATIC_DRAW);

        return { buffer, count: positions.length / 2 };
    }

    createGuideVanes() {
        const gl = this.gl;
        const positions = [];
        const vaneCount = 24;
        const innerR = 0.48;
        const outerR = 0.62;

        for (let i = 0; i < vaneCount; i++) {
            const angle = (i / vaneCount) * Math.PI * 2;
            const nextAngle = ((i + 0.7) / vaneCount) * Math.PI * 2;

            const x1 = Math.cos(angle) * innerR;
            const y1 = Math.sin(angle) * innerR;
            const x2 = Math.cos(nextAngle) * outerR;
            const y2 = Math.sin(nextAngle) * outerR;

            const perpAngle = angle + Math.PI / 2;
            const thickness = 0.015;
            const tx = Math.cos(perpAngle) * thickness;
            const ty = Math.sin(perpAngle) * thickness;

            positions.push(
                x1 + tx, y1 + ty,
                x2 + tx, y2 + ty,
                x2 - tx, y2 - ty,
                x1 - tx, y1 - ty,
                x1 + tx, y1 + ty
            );
        }

        const buffer = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
        gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(positions), gl.STATIC_DRAW);

        return { buffer, count: positions.length / 2 };
    }

    createRunnerHub() {
        const gl = this.gl;
        const positions = [];
        const segments = 32;
        const innerR = 0.08;
        const outerR = 0.12;

        for (let i = 0; i <= segments; i++) {
            const angle = (i / segments) * Math.PI * 2;
            positions.push(Math.cos(angle) * outerR, Math.sin(angle) * outerR);
        }
        for (let i = segments; i >= 0; i--) {
            const angle = (i / segments) * Math.PI * 2;
            positions.push(Math.cos(angle) * innerR, Math.sin(angle) * innerR);
        }

        const buffer = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
        gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(positions), gl.STATIC_DRAW);

        return { buffer, count: positions.length / 2 };
    }

    createDraftTube() {
        const gl = this.gl;
        const positions = [];

        positions.push(
            -0.15, 0,
            -0.4, -0.8,
            0.4, -0.8,
            0.15, 0
        );

        const buffer = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
        gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(positions), gl.STATIC_DRAW);

        return { buffer, count: positions.length / 2 };
    }

    createBlades() {
        const gl = this.gl;
        const bladeMeshes = [];

        for (let bladeId = 0; bladeId < this.bladeCount; bladeId++) {
            const baseAngle = (bladeId / this.bladeCount) * Math.PI * 2;
            const positions = [];
            const colors = [];
            const intensities = [];
            const gridPoints = [];

            const innerR = 0.12;
            const outerR = 0.46;
            const radialSegments = 8;
            const angularSegments = 6;

            for (let i = 0; i <= radialSegments; i++) {
                for (let j = 0; j <= angularSegments; j++) {
                    const r = innerR + (outerR - innerR) * (i / radialSegments);
                    const bladeTwist = (i / radialSegments) * 0.5;
                    const angleOffset = (j / angularSegments - 0.5) * 0.25 * (1 - i / radialSegments * 0.5);
                    const angle = baseAngle + bladeTwist + angleOffset;

                    const x = Math.cos(angle) * r;
                    const y = Math.sin(angle) * r;

                    gridPoints.push({ x, y, u: i / radialSegments, v: j / angularSegments });
                    positions.push(x, y);

                    colors.push(0.2, 0.5, 0.8);
                    intensities.push(0.0);
                }
            }

            const indices = [];
            const vertsPerRow = angularSegments + 1;
            for (let i = 0; i < radialSegments; i++) {
                for (let j = 0; j < angularSegments; j++) {
                    const idx = i * vertsPerRow + j;
                    indices.push(
                        idx, idx + 1, idx + vertsPerRow,
                        idx + 1, idx + vertsPerRow + 1, idx + vertsPerRow
                    );
                }
            }

            const indexedPositions = [];
            const indexedColors = [];
            const indexedIntensities = [];

            for (const idx of indices) {
                indexedPositions.push(positions[idx * 2], positions[idx * 2 + 1]);
                indexedColors.push(colors[idx * 3], colors[idx * 3 + 1], colors[idx * 3 + 2]);
                indexedIntensities.push(intensities[idx]);
            }

            const posBuffer = gl.createBuffer();
            gl.bindBuffer(gl.ARRAY_BUFFER, posBuffer);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(indexedPositions), gl.DYNAMIC_DRAW);

            const colorBuffer = gl.createBuffer();
            gl.bindBuffer(gl.ARRAY_BUFFER, colorBuffer);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(indexedColors), gl.DYNAMIC_DRAW);

            const intensityBuffer = gl.createBuffer();
            gl.bindBuffer(gl.ARRAY_BUFFER, intensityBuffer);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(indexedIntensities), gl.DYNAMIC_DRAW);

            const centerPoints = [];
            for (let i = 0; i <= radialSegments; i++) {
                for (let j = 0; j <= angularSegments; j++) {
                    const idx = i * vertsPerRow + j;
                    centerPoints.push({
                        x: positions[idx * 2],
                        y: positions[idx * 2 + 1],
                        u: i / radialSegments,
                        v: j / angularSegments
                    });
                }
            }

            bladeMeshes.push({
                id: bladeId,
                baseAngle,
                positionBuffer: posBuffer,
                colorBuffer: colorBuffer,
                intensityBuffer: intensityBuffer,
                count: indices.length,
                centerPoints,
                bounds: this.computeBounds(indexedPositions),
                indexedPositions,
                vertsPerRow,
                radialSegments,
                angularSegments
            });

            this.clickableBlades.push({
                bladeId,
                baseAngle,
                center: this.computeCenter(indexedPositions),
                bounds: this.computeBounds(indexedPositions)
            });
        }

        return bladeMeshes;
    }

    computeBounds(positions) {
        let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
        for (let i = 0; i < positions.length; i += 2) {
            minX = Math.min(minX, positions[i]);
            minY = Math.min(minY, positions[i + 1]);
            maxX = Math.max(maxX, positions[i]);
            maxY = Math.max(maxY, positions[i + 1]);
        }
        return { minX, minY, maxX, maxY };
    }

    computeCenter(positions) {
        let cx = 0, cy = 0;
        for (let i = 0; i < positions.length; i += 2) {
            cx += positions[i];
            cy += positions[i + 1];
        }
        return { x: cx / (positions.length / 2), y: cy / (positions.length / 2) };
    }

    initEventListeners() {
        this.canvas.addEventListener('mousedown', (e) => this.onMouseDown(e));
        this.canvas.addEventListener('mousemove', (e) => this.onMouseMove(e));
        this.canvas.addEventListener('mouseup', (e) => this.onMouseUp(e));
        this.canvas.addEventListener('mouseleave', (e) => this.onMouseLeave(e));
        this.canvas.addEventListener('click', (e) => this.onClick(e));
        this.canvas.addEventListener('wheel', (e) => this.onWheel(e));
    }

    getMousePosition(e) {
        const rect = this.canvas.getBoundingClientRect();
        const x = (e.clientX - rect.left) / rect.width * 2 - 1;
        const y = -(e.clientY - rect.top) / rect.height * 2 + 1;

        const invMatrix = mat3.create();
        const matrix = this.getTransformMatrix();
        mat3.invert(invMatrix, matrix);

        const worldPos = vec3.fromValues(x, y, 1);
        vec3.transformMat3(worldPos, worldPos, invMatrix);

        return { x: worldPos[0], y: worldPos[1] };
    }

    onMouseDown(e) {
        this.isDragging = true;
        this.lastMouseX = e.clientX;
        this.lastMouseY = e.clientY;
    }

    onMouseMove(e) {
        const pos = this.getMousePosition(e);

        let hovered = -1;
        for (const blade of this.clickableBlades) {
            if (this.isPointInBlade(pos, blade)) {
                hovered = blade.bladeId;
                break;
            }
        }
        this.hoveredBlade = hovered;

        if (this.isDragging) {
            const dx = (e.clientX - this.lastMouseX) / this.canvas.width * 2;
            const dy = (e.clientY - this.lastMouseY) / this.canvas.height * 2;
            this.offsetX += dx / this.zoom;
            this.offsetY -= dy / this.zoom;
            this.lastMouseX = e.clientX;
            this.lastMouseY = e.clientY;
        }

        this.updateTooltip(e);
    }

    onMouseUp() {
        this.isDragging = false;
    }

    onMouseLeave() {
        this.isDragging = false;
        this.hoveredBlade = -1;
        const tooltip = document.getElementById('bladeTooltip');
        if (tooltip) tooltip.style.display = 'none';
    }

    onClick(e) {
        if (Math.abs(e.clientX - this.lastMouseX) < 5 && Math.abs(e.clientY - this.lastMouseY) < 5) {
            const pos = this.getMousePosition(e);
            for (const blade of this.clickableBlades) {
                if (this.isPointInBlade(pos, blade)) {
                    this.selectedBlade = blade.bladeId;
                    this.showBladeDetail(blade.bladeId);
                    break;
                }
            }
        }
    }

    onWheel(e) {
        e.preventDefault();
        const delta = e.deltaY > 0 ? 0.9 : 1.1;
        this.zoom = Math.max(0.5, Math.min(3.0, this.zoom * delta));
    }

    isPointInBlade(point, blade) {
        const { bounds } = blade;
        if (point.x < bounds.minX || point.x > bounds.maxX ||
            point.y < bounds.minY || point.y > bounds.maxY) {
            return false;
        }

        const mesh = this.buffers.blades[blade.bladeId];
        const positions = mesh.indexedPositions;
        
        for (let i = 0; i < positions.length; i += 6) {
            const x1 = positions[i], y1 = positions[i + 1];
            const x2 = positions[i + 2], y2 = positions[i + 3];
            const x3 = positions[i + 4], y3 = positions[i + 5];
            
            if (this.pointInTriangle(point, { x: x1, y: y1 }, { x: x2, y: y2 }, { x: x3, y: y3 })) {
                return true;
            }
        }
        return false;
    }

    pointInTriangle(pt, v1, v2, v3) {
        const d1 = this.sign(pt, v1, v2);
        const d2 = this.sign(pt, v2, v3);
        const d3 = this.sign(pt, v3, v1);

        const hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        const hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        return !(hasNeg && hasPos);
    }

    sign(p1, p2, p3) {
        return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
    }

    updateTooltip(e) {
        const tooltip = document.getElementById('bladeTooltip');
        if (!tooltip) return;

        if (this.hoveredBlade >= 0) {
            const data = this.cavitationData[this.hoveredBlade];
            const stageNames = ['正常', '初生', '临界', '发展'];
            tooltip.innerHTML = `
                <div class="tooltip-title">叶片 #${this.hoveredBlade + 1}</div>
                <div>空化强度: ${(data.intensity * 100).toFixed(1)}%</div>
                <div>状态: ${stageNames[data.stage]}</div>
                <div>累计损伤: ${(data.damage * 100).toFixed(2)}%</div>
                <div>应力: ${data.stress.toFixed(1)} MPa</div>
            `;
            tooltip.style.display = 'block';
            tooltip.style.left = (e.clientX + 15) + 'px';
            tooltip.style.top = (e.clientY + 15) + 'px';
        } else {
            tooltip.style.display = 'none';
        }
    }

    showBladeDetail(bladeId) {
        if (window.bladeSelectionCallback) {
            window.bladeSelectionCallback(bladeId, this.turbineId);
        }
    }

    getTransformMatrix() {
        const matrix = mat3.create();
        mat3.translate(matrix, matrix, [this.offsetX, this.offsetY]);
        mat3.scale(matrix, matrix, [this.zoom, this.zoom]);
        mat3.rotate(matrix, matrix, this.rotation);
        return matrix;
    }

    getCavitationColor(intensity, stage) {
        const stages = [
            { r: 0.1, g: 0.8, b: 0.3 },
            { r: 1.0, g: 0.8, b: 0.0 },
            { r: 1.0, g: 0.4, b: 0.0 },
            { r: 1.0, g: 0.1, b: 0.2 }
        ];

        const baseColor = stages[Math.min(stage, 3)];
        const t = Math.min(intensity * 2, 1.0);

        return {
            r: 0.2 + (baseColor.r - 0.2) * t,
            g: 0.5 + (baseColor.g - 0.5) * t,
            b: 0.8 + (baseColor.b - 0.8) * t
        };
    }

    updateBladeColors() {
        const gl = this.gl;

        for (const mesh of this.buffers.blades) {
            const bladeId = mesh.id;
            const data = this.cavitationData[bladeId];
            const isHovered = (bladeId === this.hoveredBlade);
            const isSelected = (bladeId === this.selectedBlade);

            const colors = [];
            const intensities = [];
            const vertCount = mesh.count;

            for (let i = 0; i < vertCount; i++) {
                const centerPoint = mesh.centerPoints[i % mesh.centerPoints.length];
                const distFromCenter = Math.sqrt(
                    Math.pow(centerPoint.u - 0.5, 2) + Math.pow(centerPoint.v - 0.5, 2)
                );

                const intensityVariation = data.intensity * (1 - distFromCenter * 0.5 + Math.random() * 0.1);
                const localIntensity = Math.max(0, Math.min(1, intensityVariation));

                let color = this.getCavitationColor(localIntensity, data.stage);

                if (isHovered) {
                    color.r = Math.min(1, color.r + 0.2);
                    color.g = Math.min(1, color.g + 0.2);
                    color.b = Math.min(1, color.b + 0.2);
                }
                if (isSelected) {
                    color.r = Math.min(1, color.r + 0.3);
                    color.g = Math.min(1, color.g + 0.1);
                    color.b = Math.min(1, color.b + 0.4);
                }

                colors.push(color.r, color.g, color.b);
                intensities.push(localIntensity);
            }

            gl.bindBuffer(gl.ARRAY_BUFFER, mesh.colorBuffer);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(colors), gl.DYNAMIC_DRAW);

            gl.bindBuffer(gl.ARRAY_BUFFER, mesh.intensityBuffer);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(intensities), gl.DYNAMIC_DRAW);
        }
    }

    render() {
        const gl = this.gl;
        if (!gl) return;

        gl.clearColor(0.05, 0.08, 0.12, 1.0);
        gl.clear(gl.COLOR_BUFFER_BIT);

        gl.enable(gl.BLEND);
        gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);

        const matrix = this.getTransformMatrix();

        this.updateBladeColors();

        this.drawDraftTube(matrix);
        this.drawSpiralCase(matrix);
        this.drawGuideVanes(matrix);
        this.drawBlades(matrix);
        this.drawRunnerHub(matrix);
    }

    drawDraftTube(matrix) {
        const gl = this.gl;
        const program = this.programs.basic;
        const data = this.buffers.draftTube;

        gl.useProgram(program);
        gl.uniformMatrix3fv(gl.getUniformLocation(program, 'u_matrix'), false, matrix);
        gl.uniform4f(gl.getUniformLocation(program, 'u_color'), 0.15, 0.25, 0.35, 0.8);

        const posLoc = gl.getAttribLocation(program, 'a_position');
        gl.bindBuffer(gl.ARRAY_BUFFER, data.buffer);
        gl.enableVertexAttribArray(posLoc);
        gl.vertexAttribPointer(posLoc, 2, gl.FLOAT, false, 0, 0);

        gl.drawArrays(gl.TRIANGLE_FAN, 0, data.count);
    }

    drawSpiralCase(matrix) {
        const gl = this.gl;
        const program = this.programs.basic;
        const data = this.buffers.spiralCase;

        gl.useProgram(program);
        gl.uniformMatrix3fv(gl.getUniformLocation(program, 'u_matrix'), false, matrix);
        gl.uniform4f(gl.getUniformLocation(program, 'u_color'), 0.2, 0.3, 0.4, 0.9);

        const posLoc = gl.getAttribLocation(program, 'a_position');
        gl.bindBuffer(gl.ARRAY_BUFFER, data.buffer);
        gl.enableVertexAttribArray(posLoc);
        gl.vertexAttribPointer(posLoc, 2, gl.FLOAT, false, 0, 0);

        gl.drawArrays(gl.TRIANGLE_FAN, 0, data.count);

        gl.uniform4f(gl.getUniformLocation(program, 'u_color'), 0.3, 0.5, 0.7, 1.0);
        gl.drawArrays(gl.LINE_LOOP, 0, data.count / 2);
    }

    drawGuideVanes(matrix) {
        const gl = this.gl;
        const program = this.programs.basic;
        const data = this.buffers.guideVanes;

        gl.useProgram(program);
        gl.uniformMatrix3fv(gl.getUniformLocation(program, 'u_matrix'), false, matrix);
        gl.uniform4f(gl.getUniformLocation(program, 'u_color'), 0.25, 0.35, 0.45, 0.9);

        const posLoc = gl.getAttribLocation(program, 'a_position');
        gl.bindBuffer(gl.ARRAY_BUFFER, data.buffer);
        gl.enableVertexAttribArray(posLoc);
        gl.vertexAttribPointer(posLoc, 2, gl.FLOAT, false, 0, 0);

        gl.drawArrays(gl.TRIANGLE_STRIP, 0, data.count);
    }

    drawBlades(matrix) {
        const gl = this.gl;
        const program = this.programs.blade;

        gl.useProgram(program);
        gl.uniformMatrix3fv(gl.getUniformLocation(program, 'u_matrix'), false, matrix);

        const posLoc = gl.getAttribLocation(program, 'a_position');
        const colorLoc = gl.getAttribLocation(program, 'a_color');
        const intensityLoc = gl.getAttribLocation(program, 'a_intensity');

        for (const mesh of this.buffers.blades) {
            gl.bindBuffer(gl.ARRAY_BUFFER, mesh.positionBuffer);
            gl.enableVertexAttribArray(posLoc);
            gl.vertexAttribPointer(posLoc, 2, gl.FLOAT, false, 0, 0);

            gl.bindBuffer(gl.ARRAY_BUFFER, mesh.colorBuffer);
            gl.enableVertexAttribArray(colorLoc);
            gl.vertexAttribPointer(colorLoc, 3, gl.FLOAT, false, 0, 0);

            gl.bindBuffer(gl.ARRAY_BUFFER, mesh.intensityBuffer);
            gl.enableVertexAttribArray(intensityLoc);
            gl.vertexAttribPointer(intensityLoc, 1, gl.FLOAT, false, 0, 0);

            gl.drawArrays(gl.TRIANGLES, 0, mesh.count);
        }
    }

    drawRunnerHub(matrix) {
        const gl = this.gl;
        const program = this.programs.basic;
        const data = this.buffers.runnerHub;

        gl.useProgram(program);
        gl.uniformMatrix3fv(gl.getUniformLocation(program, 'u_matrix'), false, matrix);
        gl.uniform4f(gl.getUniformLocation(program, 'u_color'), 0.15, 0.2, 0.3, 1.0);

        const posLoc = gl.getAttribLocation(program, 'a_position');
        gl.bindBuffer(gl.ARRAY_BUFFER, data.buffer);
        gl.enableVertexAttribArray(posLoc);
        gl.vertexAttribPointer(posLoc, 2, gl.FLOAT, false, 0, 0);

        gl.drawArrays(gl.TRIANGLE_FAN, 0, data.count);

        gl.uniform4f(gl.getUniformLocation(program, 'u_color'), 0.4, 0.6, 0.8, 1.0);
        gl.drawArrays(gl.LINE_LOOP, 0, data.count / 2);
    }

    updateCavitationData(cavitationData) {
        for (let bladeId = 0; bladeId < this.bladeCount; bladeId++) {
            if (cavitationData[bladeId]) {
                this.cavitationData[bladeId] = { ...this.cavitationData[bladeId], ...cavitationData[bladeId] };
            }
        }
    }

    setTurbine(turbineId) {
        this.turbineId = turbineId;
        this.selectedBlade = -1;
    }

    animate() {
        this.rotation += 0.002;
        this.render();
        requestAnimationFrame(() => this.animate());
    }
}
