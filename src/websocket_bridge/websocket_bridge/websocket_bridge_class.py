import asyncio
import websockets
import threading


class WebSocketBridge:
    def __init__(self, host="127.0.0.1", port=5000):
        self.host = host
        self.port = port
        self.clients = set()

        self.loop = None
        self.server = None

    # ----------------------------------------------------------
    # SERVER
    # ----------------------------------------------------------

    async def handler(self, websocket):
        # nuovo client
        self.clients.add(websocket)
        try:
            async for _ in websocket:
                pass  # il client non manda niente
        finally:
            self.clients.remove(websocket)

    def run_server(self):
        """
        Avvia un event loop asyncio dedicato + WebSocket server.
        Metodo bloccante (ma gira nel worker thread).
        """
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        start_server = websockets.serve(self.handler, self.host, self.port)

        self.server = self.loop.run_until_complete(start_server)
        self.loop.run_forever()

    # ----------------------------------------------------------
    # BROADCAST
    # ----------------------------------------------------------

    def broadcast(self, msg: str):
        """
        Manda msg (string) a tutti i client connessi.
        Usa call_soon_threadsafe perché l'event loop asyncio gira in un altro thread.
        """
        if not self.loop:
            return

        async def _send():
            if self.clients:
                await asyncio.gather(
                    *[client.send(msg) for client in self.clients],
                    return_exceptions=True
                )

        self.loop.call_soon_threadsafe(asyncio.create_task, _send())

    # ----------------------------------------------------------
    # STOP
    # ----------------------------------------------------------

    def stop(self):
        """
        Arresta il WebSocket server in modo pulito.
        """
        if self.loop and self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)