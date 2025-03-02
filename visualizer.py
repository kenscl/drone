import serial
import numpy as np
import asyncio
import websockets
import json
port = '/dev/ttyUSB0'  
baudrate = 115200 

ser = serial.Serial(port, baudrate)


def read_attitude():
    last_attitude = np.array([1., 0., 0., 0.,])
    try:
        data = ser.readline().decode('utf-8').strip().replace("\x00", "")
        try:
            sensor, q, i, j, k = data.split(",")
            if str(sensor) == str('Attitude'):
                last_attitude = np.array([float(q), float(i), float(j), float(k)])
        except ValueError:
            pass
    except KeyboardInterrupt:
        pass 
    return last_attitude


async def send_data(websocket):
    while True:
        q = read_attitude()
        print(q)
        await websocket.send(json.dumps({"qw": q[0], "qi": q[1], "qj": q[2], "qk": q[3]}))

async def main():
    async with websockets.serve(send_data, "localhost", 1304):
        await asyncio.Future() 
if __name__ == "__main__":
    asyncio.run(main()) 
