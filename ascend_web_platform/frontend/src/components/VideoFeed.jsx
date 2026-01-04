import { Video, Maximize } from 'lucide-react'

export default function VideoFeed({ frame }) {
    return (
        <div className="w-full h-full flex flex-col">
            <div className="absolute top-4 left-4 z-10 flex items-center gap-2 bg-black/40 backdrop-blur px-3 py-1 rounded-full border border-white/10">
                <div className="w-2 h-2 rounded-full bg-red-500 animate-pulse"></div>
                <span className="text-xs font-semibold tracking-wider">LIVE FEED</span>
            </div>

            <div className="flex-1 bg-black flex items-center justify-center relative overflow-hidden group">
                {frame ? (
                    <img
                        src={`data:image/jpeg;base64,${frame}`}
                        alt="Drone Feed"
                        className="w-full h-full object-contain"
                    />
                ) : (
                    <div className="flex flex-col items-center gap-3 text-slate-600">
                        <Video size={48} className="opacity-50" />
                        <span className="text-sm font-mono tracking-widest uppercase">Signal Lost / Waiting for Stream</span>
                    </div>
                )}

                {/* Overlay Grid */}
                <div className="absolute inset-0 pointer-events-none opacity-20"
                    style={{ backgroundImage: 'linear-gradient(rgba(255,255,255,0.1) 1px, transparent 1px), linear-gradient(90deg, rgba(255,255,255,0.1) 1px, transparent 1px)', backgroundSize: '100px 100px' }}>
                </div>
            </div>

            <div className="absolute bottom-4 right-4 z-10">
                <button className="p-2 bg-black/40 backdrop-blur rounded-lg border border-white/10 hover:bg-white/10 transition">
                    <Maximize size={18} />
                </button>
            </div>
        </div>
    )
}
