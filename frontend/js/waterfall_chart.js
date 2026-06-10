class WaterfallChart {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');
        this.frequencyBins = 512;
        this.maxHistoryLines = 300;
        this.history = [];
        this.maxFreq = 25600;
        this.isPaused = false;
        this.colorMap = this.createColorMap();
        
        this.resize();
        window.addEventListener('resize', () => this.resize());
    }

    resize() {
        const rect = this.canvas.parentElement.getBoundingClientRect();
        this.canvas.width = rect.width * window.devicePixelRatio;
        this.canvas.height = rect.height * window.devicePixelRatio;
        this.canvas.style.width = rect.width + 'px';
        this.canvas.style.height = rect.height + 'px';
        this.ctx.scale(window.devicePixelRatio, window.devicePixelRatio);
        this.width = rect.width;
        this.height = rect.height;
    }

    createColorMap() {
        const colors = [];
        const steps = 256;
        
        for (let i = 0; i < steps; i++) {
            const t = i / (steps - 1);
            let r, g, b;
            
            if (t < 0.25) {
                const t2 = t / 0.25;
                r = 0;
                g = Math.floor(t2 * 128);
                b = Math.floor(64 + t2 * 191);
            } else if (t < 0.5) {
                const t2 = (t - 0.25) / 0.25;
                r = Math.floor(t2 * 128);
                g = Math.floor(128 + t2 * 127);
                b = Math.floor(255 - t2 * 128);
            } else if (t < 0.75) {
                const t2 = (t - 0.5) / 0.25;
                r = Math.floor(128 + t2 * 127);
                g = Math.floor(255 - t2 * 128);
                b = Math.floor(127 - t2 * 127);
            } else {
                const t2 = (t - 0.75) / 0.25;
                r = 255;
                g = Math.floor(127 - t2 * 127);
                b = Math.floor(t2 * 64);
            }
            
            colors.push(`rgb(${r}, ${g}, ${b})`);
        }
        
        return colors;
    }

    addSpectrum(spectrumData) {
        if (this.isPaused) return;

        if (spectrumData.length !== this.frequencyBins) {
            const resampled = new Float32Array(this.frequencyBins);
            const ratio = spectrumData.length / this.frequencyBins;
            
            for (let i = 0; i < this.frequencyBins; i++) {
                const srcIdx = Math.floor(i * ratio);
                const nextIdx = Math.min(srcIdx + 1, spectrumData.length - 1);
                const frac = (i * ratio) - srcIdx;
                resampled[i] = spectrumData[srcIdx] * (1 - frac) + spectrumData[nextIdx] * frac;
            }
            
            this.history.unshift(resampled);
        } else {
            this.history.unshift(new Float32Array(spectrumData));
        }

        if (this.history.length > this.maxHistoryLines) {
            this.history.pop();
        }

        this.render();
    }

    addTestData() {
        const spectrum = new Float32Array(this.frequencyBins);
        const time = Date.now() * 0.001;
        
        for (let i = 0; i < this.frequencyBins; i++) {
            const freq = (i / this.frequencyBins) * this.maxFreq;
            
            let amplitude = 0.1 + Math.random() * 0.1;
            
            const rotFreq = 2;
            for (let h = 1; h <= 10; h++) {
                const center = rotFreq * h;
                const width = 2;
                const peak = Math.exp(-Math.pow((freq - center) / width, 2));
                amplitude += peak * 0.5 / h;
            }
            
            const bladeFreq = rotFreq * 15;
            for (let h = 1; h <= 5; h++) {
                const center = bladeFreq * h;
                const width = 10;
                const peak = Math.exp(-Math.pow((freq - center) / width, 2));
                amplitude += peak * 0.8 / h;
            }
            
            if (Math.random() > 0.7) {
                const cavFreq = 5000 + Math.sin(time) * 2000;
                const width = 500;
                const peak = Math.exp(-Math.pow((freq - cavFreq) / width, 2));
                amplitude += peak * (0.3 + Math.sin(time * 0.5) * 0.2);
            }
            
            amplitude = Math.max(0, Math.min(1, amplitude));
            spectrum[i] = amplitude;
        }
        
        this.addSpectrum(spectrum);
    }

    render() {
        if (!this.ctx) return;

        const ctx = this.ctx;
        const width = this.width;
        const height = this.height;

        ctx.fillStyle = '#0a0f1a';
        ctx.fillRect(0, 0, width, height);

        if (this.history.length === 0) {
            ctx.fillStyle = '#555';
            ctx.font = '14px sans-serif';
            ctx.textAlign = 'center';
            ctx.fillText('等待数据...', width / 2, height / 2);
            return;
        }

        const chartLeft = 60;
        const chartTop = 20;
        const chartWidth = width - chartLeft - 20;
        const chartHeight = height - chartTop - 50;
        const lineHeight = chartHeight / this.maxHistoryLines;

        let maxVal = 0;
        let minVal = Infinity;
        for (const line of this.history) {
            for (const val of line) {
                maxVal = Math.max(maxVal, val);
                minVal = Math.min(minVal, val);
            }
        }
        if (maxVal === minVal) {
            maxVal = minVal + 1;
        }

        for (let lineIdx = 0; lineIdx < this.history.length; lineIdx++) {
            const line = this.history[lineIdx];
            const y = chartTop + lineIdx * lineHeight;
            const lineH = Math.max(1, lineHeight + 1);

            for (let i = 0; i < this.frequencyBins; i++) {
                const normalized = (line[i] - minVal) / (maxVal - minVal);
                const colorIdx = Math.floor(normalized * (this.colorMap.length - 1));
                ctx.fillStyle = this.colorMap[Math.max(0, Math.min(this.colorMap.length - 1, colorIdx))];

                const x = chartLeft + (i / this.frequencyBins) * chartWidth;
                const binWidth = Math.ceil(chartWidth / this.frequencyBins);
                ctx.fillRect(x, y, binWidth, lineH);
            }
        }

        ctx.fillStyle = '#ffffff';
        ctx.font = '12px sans-serif';
        ctx.textAlign = 'right';
        
        ctx.fillStyle = '#444';
        ctx.lineWidth = 0.5;
        for (let i = 0; i <= 5; i++) {
            const freq = (i / 5) * this.maxFreq;
            const x = chartLeft + (i / 5) * chartWidth;
            
            ctx.beginPath();
            ctx.moveTo(x, chartTop);
            ctx.lineTo(x, chartTop + chartHeight);
            ctx.stroke();
            
            ctx.fillStyle = '#888';
            ctx.textAlign = 'center';
            ctx.fillText(`${(freq / 1000).toFixed(0)} kHz`, x, chartTop + chartHeight + 20);
        }

        ctx.fillStyle = '#888';
        ctx.textAlign = 'right';
        ctx.fillText('最新', chartLeft - 10, chartTop + 15);
        ctx.fillText(`−${this.maxHistoryLines / 10}s`, chartLeft - 10, chartTop + chartHeight / 2);
        ctx.fillText(`−${this.maxHistoryLines / 5}s`, chartLeft - 10, chartTop + chartHeight);

        ctx.strokeStyle = '#333';
        ctx.lineWidth = 2;
        ctx.strokeRect(chartLeft, chartTop, chartWidth, chartHeight);

        ctx.fillStyle = '#fff';
        ctx.font = '12px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('频率 (kHz)', width / 2, height - 10);

        this.drawColorBar(chartLeft + chartWidth + 10, chartTop, 20, chartHeight, minVal, maxVal);
    }

    drawColorBar(x, y, width, height, minVal, maxVal) {
        const ctx = this.ctx;
        const steps = 64;

        for (let i = 0; i < steps; i++) {
            const t = i / steps;
            const colorIdx = Math.floor(t * (this.colorMap.length - 1));
            ctx.fillStyle = this.colorMap[colorIdx];
            ctx.fillRect(x, y + height * (1 - t) - height / steps, width, height / steps + 1);
        }

        ctx.strokeStyle = '#666';
        ctx.lineWidth = 1;
        ctx.strokeRect(x, y, width, height);

        ctx.fillStyle = '#888';
        ctx.font = '11px sans-serif';
        ctx.textAlign = 'left';
        ctx.fillText(maxVal.toFixed(2), x + width + 5, y + 10);
        ctx.fillText(((minVal + maxVal) / 2).toFixed(2), x + width + 5, y + height / 2);
        ctx.fillText(minVal.toFixed(2), x + width + 5, y + height - 2);

        ctx.save();
        ctx.translate(x + width + 40, y + height / 2);
        ctx.rotate(-Math.PI / 2);
        ctx.textAlign = 'center';
        ctx.fillText('幅值', 0, 0);
        ctx.restore();
    }

    clear() {
        this.history = [];
        this.render();
    }

    pause() {
        this.isPaused = !this.isPaused;
        return this.isPaused;
    }

    setMaxFreq(freq) {
        this.maxFreq = freq;
    }
}
