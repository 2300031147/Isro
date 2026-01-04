import uvicorn
import socketio
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from ros_interface import start_ros_node

# Create Socket.IO server (async mode for ASGI)
sio = socketio.AsyncServer(async_mode='asgi', cors_allowed_origins='*')
app = FastAPI()

# Wrap FastAPI with Socket.IO
socket_app = socketio.ASGIApp(sio, app)

# CORS (Allow frontend access)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global ROS Node reference
ros_node = None

@app.on_event("startup")
async def startup_event():
    global ros_node
    # Pass socketio instance to ROS node so it can emit events
    ros_node = start_ros_node(sio)

@app.get("/")
def read_root():
    return {"status": "ASCEND Backend Running"}

@app.get("/api/status")
def get_status():
    if ros_node:
        return ros_node.drone_state
    return {"error": "ROS Node not initialized"}

# Socket.IO Events
@sio.event
async def connect(sid, environ):
    print(f"Client connected: {sid}")
    # Send initial state immediately
    if ros_node:
        await sio.emit('telemetry', ros_node.drone_state, to=sid)

@sio.event
async def disconnect(sid):
    print(f"Client disconnected: {sid}")

if __name__ == "__main__":
    uvicorn.run("main:socket_app", host="0.0.0.0", port=8000, reload=True)
