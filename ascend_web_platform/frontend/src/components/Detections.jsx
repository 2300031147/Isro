import { Target, Image as ImageIcon } from 'lucide-react'

export default function Detections() {
    // Placeholder data
    const detections = []

    return (
        <div className="glass-panel p-4 flex flex-col gap-4 flex-1 min-h-0">
            <div className="flex items-center justify-between">
                <div className="flex items-center gap-2 text-slate-400">
                    <Target size={16} className="text-accent-red" />
                    <span className="text-xs font-bold tracking-widest uppercase">Target Log</span>
                </div>
                <span className="text-xs text-slate-600">0 FOUND</span>
            </div>

            <div className="flex-1 overflow-y-auto pr-2 space-y-2 custom-scrollbar">
                {detections.length === 0 ? (
                    <div className="h-full flex flex-col items-center justify-center text-slate-600 gap-2 opacity-50">
                        <ImageIcon size={24} />
                        <span className="text-xs">NO DETECTIONS YET</span>
                    </div>
                ) : (
                    detections.map((d, i) => (
                        <div key={i} className="bg-white/5 p-2 rounded flex gap-2">
                            {/* Img placeholder */}
                            <div className="w-12 h-12 bg-black rounded" />
                            <div>
                                <div className="text-sm font-bold">ROCK_BASALT</div>
                                <div className="text-xs text-slate-400">Confidence: 98%</div>
                            </div>
                        </div>
                    ))
                )}
            </div>
        </div>
    )
}
