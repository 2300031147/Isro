import { useRef, useEffect } from 'react'
import { Map as MapIcon, Compass } from 'lucide-react'

export default function Map({ position }) {
    const canvasRef = useRef(null)

    useEffect(() => {
        const canvas = canvasRef.current
        if (!canvas) return
        const ctx = canvas.getContext('2d')
        const width = canvas.width
        const height = canvas.height

        // Clear
        ctx.clearRect(0, 0, width, height)

        // Grid
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.1)'
        ctx.lineWidth = 1
        const gridSize = 40
        for (let x = 0; x <= width; x += gridSize) {
            ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, height); ctx.stroke();
        }
        for (let y = 0; y <= height; y += gridSize) {
            ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke();
        }

        // Center (0,0) is middle of canvas
        const cx = width / 2
        const cy = height / 2

        // Draw Home
        ctx.fillStyle = '#4ade80'
        ctx.beginPath(); ctx.arc(cx, cy, 4, 0, Math.PI * 2); ctx.fill();

        // Draw Drone (Assuming 1px = 10cm scale for indoor flight, so x*10)
        // Invert Y because canvas Y is down
        const dx = cx + (position.x * 20)
        const dy = cy - (position.y * 20)

        ctx.shadowBlur = 10
        ctx.shadowColor = '#38bdf8'
        ctx.fillStyle = '#38bdf8'

        // Drone Icon (Arrow)
        ctx.save()
        ctx.translate(dx, dy)
        // ctx.rotate(yaw) // If we had yaw
        ctx.beginPath()
        ctx.moveTo(0, -8)
        ctx.lineTo(6, 8)
        ctx.lineTo(0, 5)
        ctx.lineTo(-6, 8)
        ctx.closePath()
        ctx.fill()
        ctx.restore()

        ctx.shadowBlur = 0

    }, [position])

    return (
        <div className="flex flex-col h-full gap-4">
            <div className="flex items-center justify-between">
                <div className="flex items-center gap-2 text-slate-400">
                    <MapIcon size={16} />
                    <span className="text-xs font-bold tracking-widest uppercase">Mission Area</span>
                </div>
                <Compass size={16} className="text-slate-500" />
            </div>

            <div className="flex-1 bg-black/20 rounded-lg border border-white/5 relative overflow-hidden">
                <canvas
                    ref={canvasRef}
                    width={400}
                    height={300}
                    className="w-full h-full object-cover"
                />
                <div className="absolute bottom-2 left-2 text-[10px] text-slate-500 font-mono">
                    SCALE: 1GRID = 2M
                </div>
            </div>
        </div>
    )
}
