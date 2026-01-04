import { Wifi, WifiOff, Battery, Zap } from 'lucide-react'

export default function Header({ connected, telemetry }) {
    return (
        <header className="glass-panel p-4 flex justify-between items-center h-20">
            <div className="flex items-center gap-4">
                <h1 className="text-2xl font-bold tracking-wider">
                    ASCEND <span className="text-sky-400">MISSION CONTROL</span>
                </h1>
                <div className={`flex items-center gap-2 px-3 py-1 rounded-full text-xs font-semibold ${connected ? 'bg-green-500/20 text-green-400' : 'bg-red-500/20 text-red-400'}`}>
                    {connected ? <Wifi size={14} /> : <WifiOff size={14} />}
                    {connected ? 'LINK ESTABLISHED' : 'DISCONNECTED'}
                </div>
            </div>

            <div className="flex items-center gap-8">
                <div className="flex flex-col items-end">
                    <span className="text-xs text-slate-400 uppercase tracking-widest">Flight Mode</span>
                    <span className="text-xl font-bold text-white">{telemetry.mode}</span>
                </div>

                <div className="h-10 w-px bg-white/10"></div>

                <div className="flex items-center gap-3">
                    <div className={`p-2 rounded-full ${telemetry.armed ? 'bg-red-500/20 text-red-500 animate-pulse' : 'bg-slate-700 text-slate-400'}`}>
                        <Zap size={20} />
                    </div>
                    <div className="flex flex-col">
                        <span className="text-xs text-slate-400">STATUS</span>
                        <span className={`font-bold ${telemetry.armed ? 'text-red-400' : 'text-slate-300'}`}>
                            {telemetry.armed ? 'ARMED' : 'DISARMED'}
                        </span>
                    </div>
                </div>

                <div className="flex items-center gap-3 pl-4 border-l border-white/10">
                    <Battery className={telemetry.battery < 20 ? 'text-red-500' : 'text-green-400'} />
                    <div className="flex flex-col">
                        <span className="text-2xl font-bold">{telemetry.battery}%</span>
                        <span className="text-xs text-slate-400">BATTERY</span>
                    </div>
                </div>
            </div>
        </header>
    )
}
