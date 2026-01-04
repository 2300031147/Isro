import { useState, useEffect } from 'react'
import { io } from 'socket.io-client'
import Header from './components/Header'
import Telemetry from './components/Telemetry'
import VideoFeed from './components/VideoFeed'
import Map from './components/Map'
import Detections from './components/Detections'

// Connect to backend (adjust port if needed, assuming 8000 for FastAPI)
const socket = io('http://localhost:8000')

function App() {
    const [connected, setConnected] = useState(false)
    const [telemetry, setTelemetry] = useState({
        mode: 'UNKNOWN',
        armed: false,
        battery: 0,
        altitude: 0,
        speed: 0,
        position: { x: 0, y: 0, z: 0 },
        satellites: 0
    })
    const [videoFrame, setVideoFrame] = useState(null)

    useEffect(() => {
        socket.on('connect', () => setConnected(true))
        socket.on('disconnect', () => setConnected(false))

        socket.on('telemetry', (data) => {
            setTelemetry(data)
        })

        socket.on('video_frame', (data) => {
            setVideoFrame(data)
        })

        return () => {
            socket.off('connect')
            socket.off('disconnect')
            socket.off('telemetry')
            socket.off('video_frame')
        }
    }, [])

    return (
        <div className="h-screen flex flex-col p-4 gap-4">
            <Header connected={connected} telemetry={telemetry} />

            <main className="flex-1 grid grid-cols-12 gap-4 min-h-0">
                {/* Left Column: Telemetry & Detections */}
                <div className="col-span-3 flex flex-col gap-4">
                    <Telemetry data={telemetry} />
                    <Detections />
                </div>

                {/* Middle Column: Video Feed */}
                <div className="col-span-6 flex flex-col glass-panel overflow-hidden relative">
                    <VideoFeed frame={videoFrame} />
                </div>

                {/* Right Column: Map */}
                <div className="col-span-3 glass-panel p-4">
                    <Map position={telemetry.position} />
                </div>
            </main>
        </div>
    )
}

export default App
