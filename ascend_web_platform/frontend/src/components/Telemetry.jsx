import { Gauge, Navigation, Globe, Signal } from 'lucide-react'

export default function Telemetry({ data }) {
    const items = [
        { label: 'Altitude', value: `${data.altitude} m`, icon: <Gauge size={18} className="text-sky-400" /> },
        { label: 'Ground Speed', value: `${data.speed} m/s`, icon: <Navigation size={18} className="text-emerald-400" /> },
        { label: 'Satellites', value: data.satellites, icon: <Signal size={18} className="text-amber-400" /> },
    ]

    return (
        <div className="glass-panel p-4 flex flex-col gap-4">
            <h2 className="text-sm font-semibold text-slate-400 tracking-wider">REAL-TIME TELEMETRY</h2>

            <div className="grid grid-cols-1 gap-3">
                {items.map((item, idx) => (
                    <div key={idx} className="bg-white/5 p-3 rounded-lg flex items-center justify-between">
                        <div className="flex items-center gap-3">
                            {item.icon}
                            <span className="text-slate-300 text-sm">{item.label}</span>
                        </div>
                        <span className="font-mono text-xl font-bold">{item.value}</span>
                    </div>
                ))}

                {/* XYZ Position Compact */}
                <div className="bg-white/5 p-3 rounded-lg flex flex-col gap-2">
                    <div className="flex items-center gap-3">
                        <Globe size={18} className="text-purple-400" />
                        <span className="text-slate-300 text-sm">Local Position</span>
                    </div>
                    <div className="grid grid-cols-3 gap-2 mt-1">
                        <div className="text-center bg-black/20 rounded p-1">
                            <div className="text-[10px] text-slate-500">X</div>
                            <div className="font-mono text-sm">{data.position.x}</div>
                        </div>
                        <div className="text-center bg-black/20 rounded p-1">
                            <div className="text-[10px] text-slate-500">Y</div>
                            <div className="font-mono text-sm">{data.position.y}</div>
                        </div>
                        <div className="text-center bg-black/20 rounded p-1">
                            <div className="text-[10px] text-slate-500">Z</div>
                            <div className="font-mono text-sm">{data.position.z}</div>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    )
}
